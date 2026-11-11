# -*- coding: utf-8 -*-
"""error_system.py — 读取路径误差注入（误差不进轨迹积分）。

引擎内 Axis.pos 永远是名义值；actual = base + repeat_offset
+ scale_ppm×value×1e-6 + map_interp(axis, value, 沿轴分量)
+ modal_vib（2026-09-08 迭代 22：modal.ModalSystem 的该轴 SISO 模态位移，
独立开关热查，未挂载/未启用为 0）。
base 默认为 value；伺服环动态下由引擎传 x_actual（伺服滞后与误差注入正交）。

- 地图：两列 CSV，bisect + 分段线性插值；区间外返回 0 且 measured=False
  （ideal_zero_unmeasured）。平移 µm→mm、旋转 µrad→rad 换算成轴原生单位。
- repeat_offset：每次"到位"（moving→停止）时用 seed 化 RNG 重采样
  （σ = repeat_3σ/3），保持到下次到位。
- 全局开关 enabled（默认开）；关掉后 actual = value + modal_vib（模态独立）。
"""
import bisect
import csv
import hashlib
import json
import os
import random
import threading

import axes_config as ac

ROOT = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(ROOT, "error_maps")


def stable_seed(*parts):
    raw = "|".join(map(str, parts)).encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:8], "big")


class AxisMap(object):
    """单张两列误差地图（位置单调递增）。"""
    __slots__ = ("axis", "component", "positions", "values",
                 "coverage", "sha256", "node_count")

    def __init__(self, axis, component, positions, values, sha256):
        self.axis = axis
        self.component = component
        self.positions = positions
        self.values = values
        self.coverage = [positions[0], positions[-1]]
        self.sha256 = sha256
        self.node_count = len(positions)

    def sample(self, x):
        """区间内分段线性插值；区间外 (0.0, False)。"""
        xs, ys = self.positions, self.values
        if x < xs[0] - 1e-12 or x > xs[-1] + 1e-12:
            return 0.0, False
        i = bisect.bisect_right(xs, x)
        if i <= 0:
            return ys[0], True
        if i >= len(xs):
            return ys[-1], True
        x0, x1 = xs[i - 1], xs[i]
        y0, y1 = ys[i - 1], ys[i]
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0), True


class ErrorSystem(object):
    def __init__(self, maps_dir=MAPS_DIR):
        self._lock = threading.RLock()
        self.enabled = True
        # 全机模态仿真（main 注入 modal.ModalSystem，2026-09-08 迭代 22）：
        # actual() 加性叠加该轴 SISO 模态位移（enabled 热查；None=无分量）
        self.modal = None
        # 组件级热仿真（main 注入 thermal.ThermalSystem，2026-09-09 迭代 23）：
        # actual()/sample6() 的 scale_ppm 加进热膨胀分量（enabled 热查）
        self.thermal = None
        self._maps = {}              # (axis, comp) -> AxisMap
        self._map_meta = {}          # (axis, comp) -> manifest 条目
        self._scale_ppm = {}         # axis -> ppm
        # 重复定位精度（原生单位 3σ）与当前偏移
        self._repeat_3sigma = dict(ac.REPEAT_3SIGMA)
        self._repeat_offset = {n: 0.0 for n in ac.AXIS_NAMES}
        self._fixed_offset = {n: 0.0 for n in ac.AXIS_NAMES}   # 固定偏移（如龙门回零残余偏摆）
        self._settle_count = {n: 0 for n in ac.AXIS_NAMES}
        for n in ac.AXIS_NAMES:
            self._repeat_offset[n] = self._sample_repeat(n)
        self._load(maps_dir)

    # ---------------- 加载 ----------------
    def _load(self, maps_dir):
        mpath = os.path.join(maps_dir, "manifest.json")
        if not os.path.exists(mpath):
            raise IOError("error_maps/manifest.json 不存在，"
                          "先运行 python runtime/gen_error_maps.py")
        with open(mpath, encoding="utf-8") as f:
            manifest = json.load(f)
        for item in manifest.get("maps", []):
            axis, comp = item["axis"], item["component"]
            if axis not in ac.NAME_TO_IDX:
                continue
            with open(os.path.join(maps_dir, item["filename"]),
                      encoding="utf-8-sig", newline="") as f:
                rows = list(csv.reader(f))
            positions, values = [], []
            for row in rows[1:]:
                if len(row) >= 2:
                    positions.append(float(row[0]))
                    values.append(float(row[1]))
            self._maps[(axis, comp)] = AxisMap(
                axis, comp, positions, values, item.get("sha256", ""))
            self._map_meta[(axis, comp)] = item
        lpath = os.path.join(maps_dir, "low_order.json")
        if os.path.exists(lpath):
            with open(lpath, encoding="utf-8") as f:
                low = json.load(f)
            for axis, cfg in low.get("axes", {}).items():
                if axis in ac.NAME_TO_IDX:
                    self._scale_ppm[axis] = float(cfg.get("scale_ppm", 0.0))

    # ---------------- repeat ----------------
    def _sample_repeat(self, axis):
        sigma = self._repeat_3sigma[axis] / 3.0
        rng = random.Random(stable_seed("repeat", axis, self._settle_count[axis]))
        return rng.gauss(0.0, sigma)

    def settle(self, axis):
        """到位（moving→停止）时由引擎调用：重采样并保持。"""
        with self._lock:
            self._settle_count[axis] += 1
            self._repeat_offset[axis] = self._sample_repeat(axis)

    def add_fixed_offset(self, axis, value):
        """设定固定偏移（如双驱龙门回零后的残余偏摆，原生单位）。
        与 repeat_offset 不同，到位重采样不会清除；每次回零重新设定。"""
        with self._lock:
            self._fixed_offset[axis] = float(value)

    # ---------------- actual 计算 ----------------
    def actual(self, axis, value, base=None):
        """-> (actual, detail)。value 为名义值（mm/rad，原生单位）；
        base 为叠加误差的基准位置（默认=value；伺服环动态下传 x_actual）。
        errormap 仍按 value 查表，repeat/scale 照旧——与伺服滞后正交。
        模态振动（2026-09-08 迭代 22）为独立开关的加性分量：enabled
        （本层开关）关闭时同样叠加（两系统正交）。"""
        if base is None:
            base = value
        comp = ac.axis_along_component(axis)
        amap = self._maps.get((axis, comp))
        with self._lock:
            enabled = self.enabled
            repeat = self._repeat_offset[axis]
            fixed = self._fixed_offset[axis]
            scale_ppm = self._scale_ppm.get(axis, 0.0)
        # 模态 SISO 分量（modal 未挂载/未启用 → 0；热查照本层开关先例）
        vib = 0.0
        modal = self.modal
        if modal is not None and modal.enabled:
            vib = modal.axis_vib(axis)
        # 热膨胀 scale 分量（thermal 未挂载/未启用 → 0；两系统开关正交，
        # 与本层 enabled 独立——同 modal vib 先例）
        th_scale = 0.0
        thermal = self.thermal
        if thermal is not None and thermal.enabled:
            th_scale = thermal.axis_scale_ppm(axis)
        map_native = 0.0
        measured = False
        coverage = None
        if amap is not None:
            map_val, measured = amap.sample(value)
            coverage = amap.coverage
            # 平移 µm->mm(×1e-3)；旋转 µrad->rad(×1e-6)
            map_native = map_val * (1e-3 if comp in ("dx", "dy", "dz") else 1e-6)
        th = value * th_scale * 1e-6
        if not enabled:
            return base + vib + th, {"measured": measured, "enabled": False,
                                "coverage": coverage, "map_error": 0.0,
                                "scale_ppm": scale_ppm, "repeat_offset": 0.0,
                                "fixed_offset": 0.0, "modal_vib": vib,
                                "thermal_scale_ppm": th_scale}
        scale_err = value * scale_ppm * 1e-6
        actual = base + repeat + fixed + scale_err + map_native + vib + th
        return actual, {"measured": measured, "enabled": True,
                        "coverage": coverage, "map_error": map_native,
                        "scale_ppm": scale_ppm, "repeat_offset": repeat,
                        "fixed_offset": fixed, "modal_vib": vib,
                        "thermal_scale_ppm": th_scale}

    # ---------------- 6 分量采样（6DOF 合成用） ----------------
    def sample6(self, axis, value):
        """采样该轴在当前位置的 6 分量地图 -> ((dx,dy,dz), (rx,ry,rz), along)。

        平移分量原生单位 mm（地图 µm×1e-3）、旋转分量 rad（µrad×1e-6）；
        区间外分量按 0（ideal_zero_unmeasured，与 actual 口径一致）。
        along = 沿轴标量项（repeat_offset + fixed_offset + scale_ppm×value×1e-6
        + 热膨胀 scale 分量（thermal ON 时，同 actual 口径），原生单位：
        直线 mm / 旋转 rad），由调用方按轴类型并到 t 或 r。
        本采样不受 enabled 开关影响——6DOF 层有独立的 six_dof_enabled 开关。
        """
        t = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, 0.0]
        for i, comp in enumerate(("dx", "dy", "dz")):
            amap = self._maps.get((axis, comp))
            if amap is not None:
                t[i] = amap.sample(value)[0] * 1e-3
        for i, comp in enumerate(("rx", "ry", "rz")):
            amap = self._maps.get((axis, comp))
            if amap is not None:
                r[i] = amap.sample(value)[0] * 1e-6
        # 热膨胀 scale 分量（2026-09-09 迭代 23；未挂载/未启用 → 0）
        th_scale = 0.0
        thermal = self.thermal
        if thermal is not None and thermal.enabled:
            th_scale = thermal.axis_scale_ppm(axis)
        with self._lock:
            along = (self._repeat_offset[axis] + self._fixed_offset[axis]
                     + value * (self._scale_ppm.get(axis, 0.0) + th_scale)
                     * 1e-6)
        return t, r, along

    # ---------------- 配置 ----------------
    def set_enabled(self, on):
        with self._lock:
            self.enabled = bool(on)

    def set_repeat_display(self, axis, value):
        """repeat 以显示单位（nm/µrad）写入；内部存 3σ 原生单位。"""
        value = float(value)
        if value < 0 or value > 1e6:
            raise ValueError("repeat 须在 [0, 1e6] nm/µrad 内")
        with self._lock:
            self._repeat_3sigma[axis] = value / ac.REPEAT_TO_DISPLAY

    def repeat_display(self, axis):
        with self._lock:
            return self._repeat_3sigma[axis] * ac.REPEAT_TO_DISPLAY

    # ---------------- 汇总 ----------------
    def maps_info(self):
        with self._lock:
            enabled = self.enabled
            axes = {}
            for name in ac.AXIS_NAMES:
                comps = {}
                for comp in ("dx", "dy", "dz", "rx", "ry", "rz"):
                    meta = self._map_meta.get((name, comp))
                    if meta:
                        comps[comp] = {
                            "node_count": meta["node_count"],
                            "coverage": meta["coverage"],
                            "error_min": meta["error_min"],
                            "error_max": meta["error_max"],
                            "sha256": meta["sha256"][:12],
                        }
                axes[name] = {
                    "class": ("precision" if name in ac.PRECISION_AXES
                              else "auxiliary"),
                    "along_component": ac.axis_along_component(name),
                    "scale_ppm": self._scale_ppm.get(name, 0.0),
                    "repeat_3sigma_display": round(self.repeat_display(name), 4),
                    "repeat_unit": ac.REPEAT_DISPLAY_UNIT[
                        "rad" if ac.axis_is_rotary(name) else "mm"],
                    "components": comps,
                }
            return {"ok": True, "enabled": enabled,
                    "interpolation": "piecewise_linear_inside_coverage",
                    "outside_coverage": "ideal_zero_unmeasured",
                    "map_count": len(self._maps), "axes": axes}
