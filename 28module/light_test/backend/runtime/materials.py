# -*- coding: utf-8 -*-
"""materials.py — 组件材质工程物性库（2026-09-06 迭代 21）。

为后续热仿真打底（本迭代只做落盘+面板展示，热漂移注入后续迭代再接）：
- 材料库：半导体设备常用工程材料 × 七维物性（热膨胀系数/杨氏模量/泊松比/
  密度/热导率/比热）+ 中文名 + 备注。数值取手册典型值（note 注明"典型值
  待供应商数据"）。内置 17 种，可在线编辑/新增/删除（被组件引用的禁删）。
- 组件映射：component_id → material_key。组件事实源为 TwinDOM 前端模型
  `html_model/d2w_front.json`（扁平 objects[]，只读消费，绝不写）；
  默认种子按渲染材质预设 presetId 映射（custom/无材质 → 未指定 null）。
- 落盘：materials.json（材料库）+ material_map.json（组件映射），
  tmp+os.replace 原子写（同 machine_constants.py 惯例）。
- 前端 JSON 缺失时组件列表为空但服务照常拉起（材料库/映射仍可用）。
- 校验：材料键命名/七维物性区间/中文名非空/删除引用检查/映射的材料键
  存在性/组件 id 存在性；update() 全量校验通过才替换+落盘，失败抛
  ValueError（errors[] 逐条定位），原数据不污染。
"""
import json
import math
import os
import re
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))
MATERIALS_JSON = os.path.join(ROOT, "materials.json")
MATERIAL_MAP_JSON = os.path.join(ROOT, "material_map.json")
# 组件事实源：TwinDOM 前端模型（只读消费；AGENTS.md 硬性规则 4 禁改）
FRONTEND_JSON = os.path.normpath(os.path.join(
    ROOT, "..", "..", "html_model", "d2w_front.json"))

# 七维物性字段 -> (最小, 最大)（合理区间；cte 下界留负膨胀材料余量）
PROP_FIELDS = {
    "cte_ppm_K": (-5.0, 60.0),          # 热膨胀系数 ppm/K
    "youngs_GPa": (1.0, 1000.0),        # 杨氏模量 GPa
    "poisson": (0.0, 0.5),              # 泊松比
    "density_kg_m3": (500.0, 23000.0),  # 密度 kg/m³
    "thermal_cond_W_mK": (0.01, 500.0),  # 热导率 W/(m·K)
    "spec_heat_J_kgK": (100.0, 5000.0),  # 比热容 J/(kg·K)
}
# 材料键命名（小写字母/数字/下划线，与 recipe/常数键风格一致）
KEY_RE = re.compile(r"^[a-z][a-z0-9_]{0,39}$")

_NOTE = "手册典型值，待供应商数据"

# 内置材料库（17 种半导体设备常用材料；数值=手册典型值室温口径，
# 各 note 标注量级依据；待供应商数据复核）
DEFAULT_MATERIALS = {
    "granite": {            # 花岗岩（大理石基座；济南青/印度黑量级）
        "name_cn": "花岗岩（大理石）",
        "cte_ppm_K": 6.5, "youngs_GPa": 50.0, "poisson": 0.25,
        "density_kg_m3": 2650.0, "thermal_cond_W_mK": 2.8,
        "spec_heat_J_kgK": 880.0,
        "note": _NOTE + "；E 40~70 取中值，ρ 2600~2700"},
    "al_6061": {            # 铝合金 6061-T6
        "name_cn": "铝合金 6061-T6",
        "cte_ppm_K": 23.6, "youngs_GPa": 68.9, "poisson": 0.33,
        "density_kg_m3": 2700.0, "thermal_cond_W_mK": 167.0,
        "spec_heat_J_kgK": 896.0,
        "note": _NOTE},
    "al_7075": {            # 铝合金 7075-T6
        "name_cn": "铝合金 7075-T6",
        "cte_ppm_K": 23.4, "youngs_GPa": 71.7, "poisson": 0.33,
        "density_kg_m3": 2810.0, "thermal_cond_W_mK": 130.0,
        "spec_heat_J_kgK": 960.0,
        "note": _NOTE},
    "ss_304": {             # 不锈钢 304
        "name_cn": "不锈钢 304",
        "cte_ppm_K": 17.3, "youngs_GPa": 193.0, "poisson": 0.29,
        "density_kg_m3": 8000.0, "thermal_cond_W_mK": 16.2,
        "spec_heat_J_kgK": 500.0,
        "note": _NOTE},
    "ss_316": {             # 不锈钢 316
        "name_cn": "不锈钢 316",
        "cte_ppm_K": 16.0, "youngs_GPa": 193.0, "poisson": 0.29,
        "density_kg_m3": 8000.0, "thermal_cond_W_mK": 16.3,
        "spec_heat_J_kgK": 500.0,
        "note": _NOTE},
    "cast_iron_ht300": {    # 灰铸铁 HT300（铸铁底座）
        "name_cn": "灰铸铁 HT300",
        "cte_ppm_K": 11.0, "youngs_GPa": 120.0, "poisson": 0.25,
        "density_kg_m3": 7200.0, "thermal_cond_W_mK": 50.0,
        "spec_heat_J_kgK": 500.0,
        "note": _NOTE + "；E 110~130 取中值，λ 46~54 取中值"},
    "alumina_al2o3": {      # 氧化铝陶瓷 96%（吸盘/结构陶瓷）
        "name_cn": "氧化铝陶瓷 96%",
        "cte_ppm_K": 7.2, "youngs_GPa": 300.0, "poisson": 0.22,
        "density_kg_m3": 3750.0, "thermal_cond_W_mK": 25.0,
        "spec_heat_J_kgK": 880.0,
        "note": _NOTE + "；99.5% 纯度的 E≈370/λ≈30 另计"},
    "sic": {                # 碳化硅（常压烧结）
        "name_cn": "碳化硅（烧结 SiC）",
        "cte_ppm_K": 4.0, "youngs_GPa": 410.0, "poisson": 0.17,
        "density_kg_m3": 3100.0, "thermal_cond_W_mK": 120.0,
        "spec_heat_J_kgK": 670.0,
        "note": _NOTE + "；CVD SiC 的 λ 可达 250+ 另计"},
    "invar_36": {           # 殷钢 4J36（低膨胀结构）
        "name_cn": "殷钢 4J36（Invar36）",
        "cte_ppm_K": 1.2, "youngs_GPa": 141.0, "poisson": 0.29,
        "density_kg_m3": 8100.0, "thermal_cond_W_mK": 10.5,
        "spec_heat_J_kgK": 515.0,
        "note": _NOTE + "；CTE 为 20~100℃ 均值"},
    "zerodur": {            # 零膨胀玻璃陶瓷（肖特 Zerodur）
        "name_cn": "零膨胀玻璃陶瓷（Zerodur）",
        "cte_ppm_K": 0.05, "youngs_GPa": 90.6, "poisson": 0.24,
        "density_kg_m3": 2530.0, "thermal_cond_W_mK": 1.64,
        "spec_heat_J_kgK": 821.0,
        "note": _NOTE + "；CTE 为 0~50℃ 级典型值（±0.05 级）"},
    "cfrp": {               # 碳纤维复合材料（准各向同性铺层）
        "name_cn": "碳纤维复合材料 CFRP（准各向同性）",
        "cte_ppm_K": 0.5, "youngs_GPa": 70.0, "poisson": 0.3,
        "density_kg_m3": 1600.0, "thermal_cond_W_mK": 1.0,
        "spec_heat_J_kgK": 900.0,
        "note": _NOTE + "；准各向同性铺层面内值，单向板 E 135~300 另计"},
    "fused_silica": {       # 熔融石英
        "name_cn": "熔融石英",
        "cte_ppm_K": 0.55, "youngs_GPa": 72.0, "poisson": 0.17,
        "density_kg_m3": 2200.0, "thermal_cond_W_mK": 1.4,
        "spec_heat_J_kgK": 741.0,
        "note": _NOTE},
    "bk7": {                # 光学玻璃 N-BK7（肖特）
        "name_cn": "光学玻璃 N-BK7",
        "cte_ppm_K": 7.1, "youngs_GPa": 82.0, "poisson": 0.206,
        "density_kg_m3": 2510.0, "thermal_cond_W_mK": 1.11,
        "spec_heat_J_kgK": 858.0,
        "note": _NOTE},
    "silicon": {            # 单晶硅 <100>
        "name_cn": "单晶硅",
        "cte_ppm_K": 2.6, "youngs_GPa": 169.0, "poisson": 0.28,
        "density_kg_m3": 2330.0, "thermal_cond_W_mK": 148.0,
        "spec_heat_J_kgK": 705.0,
        "note": _NOTE + "；E 各向异性 130~188，取 <110> 值"},
    "copper": {             # 无氧铜（散热/电极）
        "name_cn": "无氧铜",
        "cte_ppm_K": 16.7, "youngs_GPa": 117.0, "poisson": 0.34,
        "density_kg_m3": 8940.0, "thermal_cond_W_mK": 398.0,
        "spec_heat_J_kgK": 385.0,
        "note": _NOTE},
    "titanium_tc4": {       # 钛合金 TC4（Ti-6Al-4V）
        "name_cn": "钛合金 TC4（Ti-6Al-4V）",
        "cte_ppm_K": 8.6, "youngs_GPa": 110.0, "poisson": 0.34,
        "density_kg_m3": 4430.0, "thermal_cond_W_mK": 6.7,
        "spec_heat_J_kgK": 526.0,
        "note": _NOTE},
    "vacuum_black_al": {    # 黑色阳极氧化铝（真空腔可见件；物性按基材）
        "name_cn": "黑色阳极氧化铝（6061 基材）",
        "cte_ppm_K": 23.6, "youngs_GPa": 68.9, "poisson": 0.33,
        "density_kg_m3": 2700.0, "thermal_cond_W_mK": 167.0,
        "spec_heat_J_kgK": 896.0,
        "note": _NOTE + "；6061 基材+黑色阳极氧化表面处理，物性按基材计"},
}

# 渲染材质预设 presetId → 材料键默认映射（组件映射的种子；custom/无
# 材质不在表内 → 未指定 null）
PRESET_TO_MATERIAL = {
    "granite": "granite",
    "stainless-steel": "ss_304",
    "stainless": "ss_304",
    "cast-iron": "cast_iron_ht300",
    "aluminum-6061": "al_6061",
    "aluminum-7075": "al_7075",
    "silicon-carbide": "sic",
    "optical-glass": "bk7",
    "vacuum-black": "vacuum_black_al",
}


def _is_finite_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def validate_materials(mats):
    """材料库全量校验 -> 错误明细列表（空=合法）。"""
    errors = []
    if not isinstance(mats, dict):
        return ["材料库须为对象"]
    for key, m in mats.items():
        pre = "materials.%s" % key
        if not KEY_RE.match(key):
            errors.append("%s 材料键非法（须 ^[a-z][a-z0-9_]{0,39}$）: %r"
                          % (pre, key))
        if not isinstance(m, dict):
            errors.append("%s 须为对象" % pre)
            continue
        for fld in m:
            if fld.startswith("_"):
                continue
            if fld not in PROP_FIELDS and fld not in ("name_cn", "note"):
                errors.append("%s.%s 未知字段（白名单：物性 6 项+name_cn/note）"
                              % (pre, fld))
        if not isinstance(m.get("name_cn"), str) \
                or not m.get("name_cn", "").strip():
            errors.append("%s.name_cn 须为非空中文名字符串: %r"
                          % (pre, m.get("name_cn")))
        if "note" in m and not isinstance(m["note"], str):
            errors.append("%s.note 须为字符串: %r" % (pre, m["note"]))
        for fld, (lo, hi) in PROP_FIELDS.items():
            if fld not in m:
                errors.append("%s.%s 缺失" % (pre, fld))
                continue
            v = m[fld]
            if not _is_finite_num(v) or not (lo <= v <= hi):
                errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                              % (pre, fld, lo, hi, v))
    return errors


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, path)


class LoadError(ValueError):
    """材质物性文件加载/校验失败（启动拒绝拉起）。"""


# ---- 几何体体积/包围尺寸估算（2026-09-09 迭代 23，热仿真打底；只读消费
# 前端 JSON geometry，单位 mm；包围盒实心近似，镂空/抽壳件高估——已知边界） ----
ROUNDED_BOX_FILL = 0.98      # RoundedBox 圆角减料系数
EXTRUDE_FILL = 0.70          # Extrude 按包围盒×0.7（圆形/异形截面）
RING_DEFAULT_THICK_MM = 5.0  # Ring 无 height 字段（2D 特征：压圈/真空槽），
                             # 厚度按工程估值 5mm 计（待真机图纸）


def geo_volume_dims(geo):
    """geometry → (volume_mm3, dims[w,h,d]mm)；无法估算（Circle/无几何/
    字段缺失）→ (None, None)。dims 为局部坐标包围尺寸。"""
    if not isinstance(geo, dict):
        return None, None
    t = geo.get("type")
    try:
        if t in ("Box", "RoundedBox"):
            w, h, d = (float(geo[k]) for k in ("width", "height", "depth"))
            if min(w, h, d) <= 0:
                return None, None
            fill = ROUNDED_BOX_FILL if t == "RoundedBox" else 1.0
            return w * h * d * fill, [w, h, d]
        if t == "Cylinder":
            r, h = float(geo["radius"]), float(geo["height"])
            if min(r, h) <= 0:
                return None, None
            return math.pi * r * r * h, [2 * r, h, 2 * r]
        if t == "Ring":
            r_out = float(geo["outerRadius"])
            r_in = float(geo.get("innerRadius") or 0.0)
            th = float(geo.get("height") or RING_DEFAULT_THICK_MM)
            if r_out <= 0 or r_in < 0 or r_in >= r_out or th <= 0:
                return None, None
            return math.pi * (r_out ** 2 - r_in ** 2) * th, \
                [2 * r_out, th, 2 * r_out]
        if t == "Lathe":
            prof = geo.get("profile") or []
            pts = [(float(p[0]), float(p[1])) for p in prof
                   if isinstance(p, (list, tuple)) and len(p) >= 2]
            if len(pts) < 3:
                return None, None
            # 绕 z 轴旋转体（散度定理沿轮廓边界积分，方向无关取绝对值）：
            # V = π·|Σ (z_{i+1}−z_i)·(r_i²+r_i·r_{i+1}+r_{i+1}²)|/3
            acc = 0.0
            for i in range(len(pts)):
                r0, z0 = pts[i]
                r1, z1 = pts[(i + 1) % len(pts)]
                acc += (z1 - z0) * (r0 * r0 + r0 * r1 + r1 * r1)
            r_max = max(abs(p[0]) for p in pts)
            z_span = max(p[1] for p in pts) - min(p[1] for p in pts)
            vol = abs(acc) * math.pi / 3.0
            if vol <= 0 or z_span <= 0:
                return None, None
            return vol, [2 * r_max, z_span, 2 * r_max]
        if t == "Extrude":
            shp = geo.get("shape") or []
            xs = [float(p[0]) for p in shp
                  if isinstance(p, (list, tuple)) and len(p) >= 2]
            ys = [float(p[1]) for p in shp
                  if isinstance(p, (list, tuple)) and len(p) >= 2]
            dep = float(geo["depth"])
            if len(xs) < 3 or dep <= 0:
                return None, None
            w = max(xs) - min(xs)
            h = max(ys) - min(ys)
            if min(w, h) <= 0:
                return None, None
            return w * h * dep * EXTRUDE_FILL, [w, h, dep]
    except (TypeError, ValueError, KeyError, IndexError):
        return None, None
    return None, None        # Circle 及未知类型：跳过


class ComponentCatalog(object):
    """前端模型组件目录：启动时只读解析一次。文件缺失/损坏时组件列表
    为空但服务照常拉起（材料库/映射仍可用）。"""

    def __init__(self, path=FRONTEND_JSON):
        self._path = path
        self._items = []
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            for o in data.get("objects") or []:
                if not isinstance(o, dict) or not o.get("id"):
                    continue
                mat = o.get("material") or {}
                vol, dims = geo_volume_dims(o.get("geometry"))
                self._items.append({
                    "id": o["id"],
                    "name": o.get("name") or o["id"],
                    "parentId": o.get("parentId"),
                    "preset_id": mat.get("presetId"),
                    # 迭代 23：几何体积/包围尺寸估算（无几何→None）
                    "volume_mm3": vol,
                    "dims": dims,
                })
        except (OSError, ValueError) as e:
            print("[boot] 前端模型组件目录解析失败（%s）：%s；组件列表为空，"
                  "材质库/映射仍可用" % (path, e), flush=True)

    def list(self):
        return [dict(c) for c in self._items]

    def ids(self):
        return set(c["id"] for c in self._items)


class Materials(object):
    """材料库 + 组件映射注册表：线程安全读取 + 校验更新 + 原子落盘。
    catalog 用于组件映射的存在性校验与 presetId 默认种子。"""

    def __init__(self, catalog, mats_path=MATERIALS_JSON,
                 map_path=MATERIAL_MAP_JSON):
        self._catalog = catalog
        self._mats_path = mats_path
        self._map_path = map_path
        self._lock = threading.RLock()
        # ---- 材料库：文件缺失播种内置默认库 ----
        if os.path.exists(mats_path):
            with open(mats_path, encoding="utf-8") as f:
                mats = json.load(f)
        else:
            mats = json.loads(json.dumps(DEFAULT_MATERIALS))
            _atomic_write(mats_path, mats)
        errors = validate_materials(mats)
        if errors:
            raise LoadError("materials.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._mats = mats
        # ---- 组件映射：文件缺失按 presetId 种子生成；后增组件补种子 ----
        cmap = {}
        if os.path.exists(map_path):
            with open(map_path, encoding="utf-8") as f:
                cmap = json.load(f)
        if not isinstance(cmap, dict):
            raise LoadError("material_map.json 顶层须为对象")
        seeded = False
        for c in self._catalog.list():
            if c["id"] not in cmap:
                cmap[c["id"]] = PRESET_TO_MATERIAL.get(c["preset_id"])
                seeded = True
        errors = self._validate_map(cmap)
        if errors:
            raise LoadError("material_map.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._map = cmap
        if seeded or not os.path.exists(map_path):
            _atomic_write(map_path, cmap)

    def _validate_map(self, cmap, mats=None):
        """组件映射校验：组件 id 存在（目录非空时）+ 材料键存在或 null。"""
        mats = self._mats if mats is None else mats
        errors = []
        cat_ids = self._catalog.ids()
        for cid, mkey in cmap.items():
            if cat_ids and cid not in cat_ids:
                errors.append("material_map.%s 组件不存在（前端模型 objects "
                              "无此 id）" % cid)
            if mkey is None:
                continue
            if not isinstance(mkey, str) or mkey not in mats:
                errors.append("material_map.%s 材料键不存在: %r"
                              % (cid, mkey))
        return errors

    # ---- 材料库 ----
    def get_materials(self):
        with self._lock:
            return json.loads(json.dumps(self._mats))

    def update_materials(self, patch):
        """patch={"set":{键:物性对象（支持部分字段合并/新键须全字段）},
        "delete":[键]} → 全量校验 → 原子写盘 + 热替换。被组件映射引用
        的材料禁删。校验失败抛 ValueError，原数据不污染。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            mats = json.loads(json.dumps(self._mats))
            for key, props in (patch.get("set") or {}).items():
                if key in mats and isinstance(props, dict):
                    merged = mats[key]
                    merged.update(props)     # 存量材料部分字段合并
                else:
                    mats[key] = props        # 新增材料（须全字段，校验兜底）
            for key in (patch.get("delete") or []):
                if key in mats:
                    del mats[key]
            errors = validate_materials(mats)
            # 删除引用检查（按新映射状态）：被引用材料不可消失
            for cid, mkey in self._map.items():
                if mkey is not None and mkey not in mats:
                    errors.append("materials.%s 被组件 %s 引用，不可删除"
                                  "（先在组件映射中改派）" % (mkey, cid))
            if errors:
                raise ValueError("材料库校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            _atomic_write(self._mats_path, mats)
            self._mats = mats

    # ---- 组件映射 ----
    def get_map(self):
        with self._lock:
            return json.loads(json.dumps(self._map))

    def update_map(self, patch):
        """patch={component_id: material_key|null}（patch 式，只动给出的
        组件）→ 校验 → 原子写盘 + 热替换。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            cmap = json.loads(json.dumps(self._map))
            cmap.update(patch)
            errors = self._validate_map(cmap)
            if errors:
                raise ValueError("组件材质映射校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            _atomic_write(self._map_path, cmap)
            self._map = cmap

    # ---- GET 响应 ----
    def materials_public(self):
        with self._lock:
            mats = json.loads(json.dumps(self._mats))
            refs = {}
            for cid, mkey in self._map.items():
                if mkey is not None:
                    refs[mkey] = refs.get(mkey, 0) + 1
        return {"ok": True, "materials": mats, "ref_counts": refs}

    def map_public(self):
        return {"ok": True, "map": self.get_map()}

    def components_public(self):
        """组件列表 + 当前映射材料键（面板卡 2 数据源）。"""
        with self._lock:
            cmap = dict(self._map)
        comps = []
        for c in self._catalog.list():
            c["material_key"] = cmap.get(c["id"])
            comps.append(c)
        return {"ok": True, "components": comps}
