# -*- coding: utf-8 -*-
"""modal.py — 全机模态仿真（2026-09-08 迭代 22）。

结构振动模态层（与误差地图/6DOF 误差层正交、独立开关，**默认 OFF**）：

- 每轴 SISO 模态（21 轴 × {fn_hz, zeta, enable, input_gain}）：轴加速度
  基础激励的二阶共振模态 ẍ+2ζωẋ+ω²x=−a(t)，位移分量加进该轴读数
  （error_system.actual）→ 到位整定尾巴/读数抖动真实化。
- 全机 MIMO 模态（默认 9 个 {fn_hz, zeta, enable, inputs{轴:增益},
  outputs{模块:6DOF 投影}}）：多轴加速度共同激励同一结构模态，模态位移
  按观测矩阵投影到模块 6DOF 位姿（error_kinematics.compute 读取时叠加，
  视觉快照/键合 truth 管线自动吃到）→ "组件运动互相影响"（硅片台快跑
  摇门架、键合臂下压激头/台共振）。outputs 模块名与
  error_kinematics.json 模块对齐（wafer_stage/align_stage/bond_head/
  flip/film_stage），6 分量 = [dx,dy,dz (mm/mm), drx,dry,drz (rad/mm)]。
- 激励三件套：①轴加速度（engine 每 tick 由 ax.vel 差分，限幅 ±2×amax）；
  ②连续地板振动随机力（等级预设 关闭/洁净 0.1µm/典型 0.3µm/较差
  1.0µm/自定义，输出 RMS 口径：各使能 MIMO 模态 q 的 RSS ≈ 等级值，
  工程近似——非真 1/3 倍频程 VC 谱）；③键合接触脉冲（engine 力模型
  接触沿 → 带 contact:true 的模态注入速度冲量，幅度可配）。
- 解算：解析传播（精确离散二阶更新，任意步长稳定，同参考项目
  d2w_sim_semiconductor_refactor sim/modal.py 公式），1kHz tick；
  三角/指数因子按 (fn,ζ) 量化缓存，稳态每模态每 tick 仅数次乘加；
  空闲（无激励且 |q|,|qd| 低于阈值）自动归零省算力。
- 热软化：带 thermal:"head"/"wafer" 的模态 fn_eff=fn·(1−coeff·ΔT)
  （clamp [0.55,1.15]·fn），ΔT=对应加热器（PulseHeat_Up/Heat_Dw）当前
  温度−22℃——用已有加热器温度做耦合，不做热仿真本体。
- 状态暴露：per-mode/per-axis q/amp/rms（EMA τ≈0.25s）、4 通道 1kHz
  环形缓冲（8s，通道可配 "axis:<名>"/"mode:<名>"）。
- 参数落盘 runtime/modal.json（tmp+os.replace 原子写，同 materials.py
  惯例）；开关 enabled 为运行时内存态、**默认 OFF 不持久化**（保护
  回归基线；POST /api/modal {"enabled":true} 热切换，OFF 时状态清零）。
- 默认 fn/ζ/增益全部**工程估值，待 FRF 辨识**（锤击/脉冲辨识后替换），
  非真机实测。
"""
import copy
import json
import math
import os
import random
import re
import threading
from collections import deque

import axes_config as ac

ROOT = os.path.dirname(os.path.abspath(__file__))
MODAL_JSON = os.path.join(ROOT, "modal.json")

# 地板振动等级 → 输出 RMS 目标（µm；工程近似口径，见模块 docstring）
FLOOR_LEVELS = {"off": 0.0, "clean": 0.1, "typical": 0.3, "poor": 1.0}
# 模态键命名（与 recipe/常数键风格一致）
KEY_RE = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
# 校验区间
FN_LO, FN_HI = 0.0, 500.0            # fn_hz ∈ (0, 500]
ZETA_LO, ZETA_HI = 0.001, 0.5        # zeta ∈ [0.001, 0.5]
GAIN_ABS_MAX = 10.0                  # inputs 增益 / outputs 投影分量限幅
INPUT_GAIN_LO, INPUT_GAIN_HI = 0.0, 1.0    # SISO 加速度激励增益 ∈ (0, 1]
CUSTOM_RMS_HI = 100.0                # 自定义地板 RMS ∈ (0, 100] µm
IMPULSE_HI = 50.0                    # 接触脉冲速度冲量 ∈ [0, 50] mm/s
COEFF_HI = 0.01                      # 热软化系数 ∈ (0, 0.01] /℃
THERMAL_GROUPS = ("head", "wafer")
RMS_TAU_S = 0.25                     # 幅值 RMS 的 EMA 时间常数
SCOPE_S = 8.0                        # 环形缓冲时长（1kHz × 8s）
REF_TEMP_C = 22.0                    # 热软化参考温度
_HEATER_OF = {"head": "PulseHeat_Up", "wafer": "Heat_Dw"}
_ZERO6 = [0.0] * 6

# ---------------- 默认参数（全部工程估值，待 FRF 辨识） ----------------
# fn 分级：大行程直线 40~50Hz/ζ0.05；Z 轴 80~95Hz/0.08；微动对准 60~70Hz；
# 旋转 60~85Hz/0.08；键合头小轴 100~120Hz；翻转/蓝膜/顶针慢轴 15~35Hz。
# input_gain=轴加速度→模态力耦合系数（工程估值：把 a/ω² 静挠度量级压到
# µm 级，真值待锤击辨识）。
DEFAULT_AXES = {
    "WS_LS_X":    {"fn_hz": 45.0, "zeta": 0.05, "enable": True,
                   "input_gain": 0.05},
    "WS_LS_Y":    {"fn_hz": 45.0, "zeta": 0.05, "enable": True,
                   "input_gain": 0.05},
    "WS_Rz":      {"fn_hz": 70.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "E_EPIN_Z_1": {"fn_hz": 20.0, "zeta": 0.07, "enable": True,
                   "input_gain": 0.05},
    "E_EPIN_Z_2": {"fn_hz": 20.0, "zeta": 0.07, "enable": True,
                   "input_gain": 0.05},
    "E_EPIN_Z_3": {"fn_hz": 20.0, "zeta": 0.07, "enable": True,
                   "input_gain": 0.05},
    "BS_Arm_Z":   {"fn_hz": 95.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05, "thermal": "head", "contact": True},
    "BS_CAL_Rx":  {"fn_hz": 110.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "BS_CAL_Ry":  {"fn_hz": 110.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "AS_X":       {"fn_hz": 65.0, "zeta": 0.06, "enable": True,
                   "input_gain": 0.05},
    "AS_Y":       {"fn_hz": 65.0, "zeta": 0.06, "enable": True,
                   "input_gain": 0.05},
    "AS_Z":       {"fn_hz": 90.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "AS_Rx":      {"fn_hz": 85.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "AS_Ry":      {"fn_hz": 85.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "FA_X":       {"fn_hz": 40.0, "zeta": 0.05, "enable": True,
                   "input_gain": 0.05},
    "FA_Z":       {"fn_hz": 80.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "FA_Rx":      {"fn_hz": 25.0, "zeta": 0.06, "enable": True,
                   "input_gain": 0.05},
    "CH_Rz":      {"fn_hz": 60.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
    "ES_X":       {"fn_hz": 30.0, "zeta": 0.06, "enable": True,
                   "input_gain": 0.05},
    "ES_Y":       {"fn_hz": 30.0, "zeta": 0.06, "enable": True,
                   "input_gain": 0.05},
    "ES_PunchZ":  {"fn_hz": 85.0, "zeta": 0.08, "enable": True,
                   "input_gain": 0.05},
}

# 全机 MIMO 模态（9 个；inputs=激励轴:增益，outputs=模块:6DOF 投影）。
# 增益工程估值（量级依据：amax×gain/ω² 静挠度峰值 ~µm 级），待 FRF 辨识。
DEFAULT_MODES = {
    # 花岗岩基座 X/Y 摇摆：硅片台/翻转臂大行程激励，全机模块同向晃动
    "granite_sway_x": {"fn_hz": 36.0, "zeta": 0.04, "enable": True,
                       "inputs": {"WS_LS_X": 0.03, "FA_X": 0.015},
                       "outputs": {"wafer_stage": [1.0, 0, 0, 0, 0, 2e-6],
                                   "align_stage": [0.7, 0, 0, 0, 0, 1e-6],
                                   "bond_head": [0.9, 0, 0, 0, 0, 0],
                                   "flip": [0.5, 0, 0, 0, 0, 0]}},
    "granite_sway_y": {"fn_hz": 38.0, "zeta": 0.04, "enable": True,
                       "inputs": {"WS_LS_Y": 0.03},
                       "outputs": {"wafer_stage": [0, 1.0, 0, 0, 0, 0],
                                   "align_stage": [0, 0.6, 0, 0, 0, 0],
                                   "bond_head": [0, 0.8, 0, 0, 0, 0]}},
    # 门架扭摆（偏航）/横摆：双驱 X 与 Y 耦合激励，旋转分量为主
    "gantry_yaw":     {"fn_hz": 27.0, "zeta": 0.05, "enable": True,
                       "inputs": {"WS_LS_X": 0.02, "WS_LS_Y": -0.01},
                       "outputs": {"wafer_stage": [0, 0, 0, 0, 0, 1e-6],
                                   "align_stage": [0.3, 0, 0, 0, 0, 8e-7],
                                   "bond_head": [0.4, 0, 0, 0, 0, 6e-7]}},
    "gantry_sway_y":  {"fn_hz": 31.0, "zeta": 0.05, "enable": True,
                       "inputs": {"FA_X": 0.025, "WS_LS_Y": 0.015},
                       "outputs": {"bond_head": [0, 1.0, 0, 0, 0, 0],
                                   "flip": [0, 0.8, 0, 0, 0, 0],
                                   "wafer_stage": [0, 0.4, 0, 0, 0, 0]}},
    # 键合头 Z 向弹跳：臂下压/接触激励（热软化 head，接触脉冲目标）
    "head_z_bounce":  {"fn_hz": 58.0, "zeta": 0.06, "enable": True,
                       "inputs": {"BS_Arm_Z": 0.05},
                       "outputs": {"bond_head": [0, 0, 1.0, 1e-6, 0, 0],
                                   "wafer_stage": [0, 0, 0.3, 0, 0, 0]},
                       "thermal": "head", "contact": True},
    # 对准台桥架：对准台 X/Y 微动激励（上/下相机随台动）
    "align_bridge":   {"fn_hz": 52.0, "zeta": 0.05, "enable": True,
                       "inputs": {"AS_X": 0.04, "AS_Y": 0.04},
                       "outputs": {"align_stage": [0.8, 0.8, 0, 0, 0, 2e-6],
                                   "bond_head": [0.3, 0.3, 0, 0, 0, 0]}},
    # 蓝膜台框架（低刚度外包模块）
    "frame_film":     {"fn_hz": 18.0, "zeta": 0.06, "enable": True,
                       "inputs": {"ES_X": 0.01, "ES_Y": 0.01},
                       "outputs": {"film_stage": [1.0, 1.0, 0, 0, 0, 0]}},
    # 硅片台俯仰摇滚（热软化 wafer，接触脉冲目标）
    "wafer_stage_rock": {"fn_hz": 44.0, "zeta": 0.05, "enable": True,
                         "inputs": {"WS_LS_X": 0.015, "WS_LS_Y": 0.015},
                         "outputs": {"wafer_stage": [0, 0, 0.5, 2e-6, 2e-6, 0]},
                         "thermal": "wafer", "contact": True},
    # 翻转臂摆动（翻转/伸缩激励）
    "flip_arm_sway":  {"fn_hz": 22.0, "zeta": 0.06, "enable": True,
                       "inputs": {"FA_Rx": 0.005, "FA_X": 0.01,
                                  "FA_Z": 0.005},
                       "outputs": {"flip": [0.6, 0.6, 0.4, 0, 0, 3e-6]}},
}

DEFAULT_PARAMS = {
    "_note": "全机模态仿真参数（modal.py 加载，启动校验）。fn/zeta/增益全部"
             "工程估值，待 FRF（锤击/脉冲）辨识替换；开关 enabled 是运行时"
             "内存态（POST /api/modal），不写入本文件。floor.level: off/"
             "clean(0.1µm)/typical(0.3µm)/poor(1.0µm)/custom（输出 RMS "
             "口径工程近似，非真 VC 谱）；contact_pulse=键合接触脉冲；"
             "thermal_softening=热软化（fn_eff=fn·(1−coeff·ΔT)，clamp "
             "[0.55,1.15]·fn，ΔT=加热器当前温度−22℃）；scope_channels="
             "4 通道环形缓冲选择（axis:<轴名>/mode:<模态名>）",
    "axes": DEFAULT_AXES,
    "modes": DEFAULT_MODES,
    "floor": {"level": "typical", "custom_rms_um": 0.3},
    "contact_pulse": {"enable": True, "impulse_mm_s": 0.2},
    "thermal_softening": {"enable": True, "coeff_per_c": 0.0012},
    "scope_channels": ["axis:WS_LS_X", "mode:granite_sway_x",
                       "mode:head_z_bounce", "axis:BS_Arm_Z"],
}


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def _num(errors, path, v, lo, hi, lo_open=True):
    """有限数值 + 区间（默认 (lo, hi]；lo_open=False 时 [lo, hi]）。"""
    if not _is_num(v):
        errors.append("%s 须为有限数值: %r" % (path, v))
        return
    if (lo_open and not (lo < v <= hi)) or (not lo_open and not (lo <= v <= hi)):
        errors.append("%s 超出范围 %s%s, %s]: %r"
                      % (path, "(" if lo_open else "[", lo, hi, v))


def _bool(errors, path, v):
    if not isinstance(v, bool):
        errors.append("%s 须为布尔: %r" % (path, v))


def validate_params(cfg, module_names=None):
    """模态参数全量校验 → 错误明细列表（空=合法）。module_names 非空时
    outputs 模块键做存在性校验（与 error_kinematics.json 模块对齐）。"""
    errors = []
    if not isinstance(cfg, dict):
        return ["模态参数须为对象"]
    for k in cfg:
        if k not in ("axes", "modes", "floor", "contact_pulse",
                     "thermal_softening", "scope_channels") \
                and not str(k).startswith("_"):
            errors.append("modal 未知顶层字段: %r" % k)
    # ---- 每轴 SISO ----
    axes = cfg.get("axes")
    if not isinstance(axes, dict):
        errors.append("modal.axes 缺字段或须为对象")
    else:
        for name in sorted(axes):
            pre = "modal.axes.%s" % name
            if name not in ac.NAME_TO_IDX:
                errors.append("%s 未知轴名" % pre)
                continue
            e = axes[name]
            if not isinstance(e, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in e:
                if kk not in ("fn_hz", "zeta", "enable", "input_gain",
                              "thermal", "contact"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            _num(errors, pre + ".fn_hz", e.get("fn_hz"), FN_LO, FN_HI)
            _num(errors, pre + ".zeta", e.get("zeta"), ZETA_LO, ZETA_HI,
                 lo_open=False)
            _bool(errors, pre + ".enable", e.get("enable"))
            _num(errors, pre + ".input_gain", e.get("input_gain"),
                 INPUT_GAIN_LO, INPUT_GAIN_HI)
            if e.get("thermal") is not None \
                    and e.get("thermal") not in THERMAL_GROUPS:
                errors.append("%s.thermal 须为 null/%s: %r"
                              % (pre, "/".join(THERMAL_GROUPS),
                                 e.get("thermal")))
            if "contact" in e:
                _bool(errors, pre + ".contact", e.get("contact"))
    # ---- 全机 MIMO ----
    modes = cfg.get("modes")
    if not isinstance(modes, dict):
        errors.append("modal.modes 缺字段或须为对象")
    else:
        for name in sorted(modes):
            pre = "modal.modes.%s" % name
            if not KEY_RE.match(name):
                errors.append("%s 模态键非法（须 ^[a-z][a-z0-9_]{0,39}$）"
                              % pre)
            m = modes[name]
            if not isinstance(m, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in m:
                if kk not in ("fn_hz", "zeta", "enable", "inputs",
                              "outputs", "thermal", "contact"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            _num(errors, pre + ".fn_hz", m.get("fn_hz"), FN_LO, FN_HI)
            _num(errors, pre + ".zeta", m.get("zeta"), ZETA_LO, ZETA_HI,
                 lo_open=False)
            _bool(errors, pre + ".enable", m.get("enable"))
            if m.get("thermal") is not None \
                    and m.get("thermal") not in THERMAL_GROUPS:
                errors.append("%s.thermal 须为 null/%s: %r"
                              % (pre, "/".join(THERMAL_GROUPS),
                                 m.get("thermal")))
            if "contact" in m:
                _bool(errors, pre + ".contact", m.get("contact"))
            ins = m.get("inputs")
            if not isinstance(ins, dict):
                errors.append("%s.inputs 缺字段或须为对象" % pre)
            else:
                for ax, g in ins.items():
                    if ax not in ac.NAME_TO_IDX:
                        errors.append("%s.inputs.%s 未知轴名" % (pre, ax))
                    elif not _is_num(g) or abs(g) > GAIN_ABS_MAX:
                        errors.append("%s.inputs.%s 增益须在 ±%s: %r"
                                      % (pre, ax, GAIN_ABS_MAX, g))
            outs = m.get("outputs")
            if not isinstance(outs, dict):
                errors.append("%s.outputs 缺字段或须为对象" % pre)
            else:
                for mod, vec in outs.items():
                    if module_names and mod not in module_names:
                        errors.append("%s.outputs.%s 未知模块（"
                                      "error_kinematics.json 无此模块）"
                                      % (pre, mod))
                    if not (isinstance(vec, (list, tuple)) and len(vec) == 6
                            and all(_is_num(x) and abs(x) <= GAIN_ABS_MAX
                                    for x in vec)):
                        errors.append("%s.outputs.%s 须为 6 分量向量"
                                      "（各分量 ±%s）: %r"
                                      % (pre, mod, GAIN_ABS_MAX, vec))
    # ---- 地板振动 ----
    fl = cfg.get("floor")
    if not isinstance(fl, dict):
        errors.append("modal.floor 缺字段或须为对象")
    else:
        for kk in fl:
            if kk not in ("level", "custom_rms_um"):
                errors.append("modal.floor 未知字段: %r" % kk)
        lv = fl.get("level")
        if lv not in tuple(FLOOR_LEVELS) + ("custom",):
            errors.append("modal.floor.level 非法: %r（可选 %s/custom）"
                          % (lv, "/".join(FLOOR_LEVELS)))
        _num(errors, "modal.floor.custom_rms_um", fl.get("custom_rms_um"),
             0.0, CUSTOM_RMS_HI)
    # ---- 接触脉冲 ----
    cp = cfg.get("contact_pulse")
    if not isinstance(cp, dict):
        errors.append("modal.contact_pulse 缺字段或须为对象")
    else:
        for kk in cp:
            if kk not in ("enable", "impulse_mm_s"):
                errors.append("modal.contact_pulse 未知字段: %r" % kk)
        _bool(errors, "modal.contact_pulse.enable", cp.get("enable"))
        _num(errors, "modal.contact_pulse.impulse_mm_s",
             cp.get("impulse_mm_s"), 0.0, IMPULSE_HI, lo_open=False)
    # ---- 热软化 ----
    ts = cfg.get("thermal_softening")
    if not isinstance(ts, dict):
        errors.append("modal.thermal_softening 缺字段或须为对象")
    else:
        for kk in ts:
            if kk not in ("enable", "coeff_per_c"):
                errors.append("modal.thermal_softening 未知字段: %r" % kk)
        _bool(errors, "modal.thermal_softening.enable", ts.get("enable"))
        _num(errors, "modal.thermal_softening.coeff_per_c",
             ts.get("coeff_per_c"), 0.0, COEFF_HI)
    # ---- 示波器通道 ----
    sch = cfg.get("scope_channels")
    if not isinstance(sch, list) or not (1 <= len(sch) <= 4):
        errors.append("modal.scope_channels 须为 1~4 项数组: %r" % (sch,))
    else:
        mode_keys = modes if isinstance(modes, dict) else {}
        for i, chn in enumerate(sch):
            ok = isinstance(chn, str) and (
                (chn.startswith("axis:") and chn[5:] in ac.NAME_TO_IDX)
                or (chn.startswith("mode:") and chn[5:] in mode_keys))
            if not ok:
                errors.append("modal.scope_channels[%d] 须为 axis:<轴名>/"
                              "mode:<模态名>: %r" % (i, chn))
    return errors


def _fill_defaults(cfg):
    """加载迁移：缺失顶层块/轴/模态/字段按默认播种（仅补缺失，非法值仍
    由 validate 报错）。"""
    d = DEFAULT_PARAMS
    for k in ("floor", "contact_pulse", "thermal_softening",
              "scope_channels"):
        cfg.setdefault(k, copy.deepcopy(d[k]))
    axes = cfg.setdefault("axes", {})
    for name, dv in d["axes"].items():
        cur = axes.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, vv)
    modes = cfg.setdefault("modes", {})
    for name, dv in d["modes"].items():
        cur = modes.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, copy.deepcopy(vv))
    return cfg


def _deep_merge(dst, src):
    """递归合并：dict 逐键合并，其余（含数组）整体替换。"""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, path)


class LoadError(ValueError):
    """模态参数文件加载/校验失败（启动拒绝拉起）。"""


class ModalSystem(object):
    """模态仿真运行时：参数容器 + 1kHz 解析传播 + 状态暴露。

    线程模型：tick() 由物理线程在 engine.lock 内调用；actual/compute/
    API 读取侧只取基本类型快照（GIL 原子），enabled 开关照
    ErrorSystem.set_enabled 先例加锁；参数更新（PUT）与 tick 经
    self._lock 互斥（持锁极短，拷贝引用即释放）。"""

    def __init__(self, path=MODAL_JSON, module_names=None, dt=0.001):
        self._path = path
        self._module_names = tuple(module_names or ())
        self._dt = float(dt)
        self._lock = threading.RLock()
        self.enabled = False           # 总开关：运行时内存态，默认 OFF
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                cfg = json.load(f)
            cfg = _fill_defaults(cfg)
        else:
            cfg = copy.deepcopy(DEFAULT_PARAMS)
            _atomic_write(path, cfg)
        errors = validate_params(cfg, self._module_names or None)
        if errors:
            raise LoadError("modal.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._cfg = cfg
        self._rng = random.Random()    # 地板激励专用（不污染全局随机序列）
        self._trig = {}                # (fn 量化, zeta) -> (w, wd, e, c, s)
        self._reset_state()

    # ---------------- 状态 ----------------
    def _reset_state(self):
        """运行时状态清零（OFF 切换/参数替换时调用）。"""
        self._ax_state = {n: {"q": 0.0, "qd": 0.0, "var": 0.0}
                          for n in ac.AXIS_NAMES}
        self._mode_state = {n: {"q": 0.0, "qd": 0.0, "var": 0.0}
                            for n in self._cfg["modes"]}
        self._last_vel = {n: 0.0 for n in ac.AXIS_NAMES}
        n_scope = int(SCOPE_S / self._dt)
        self._scope = [deque(maxlen=n_scope)
                       for _ in self._cfg["scope_channels"]]

    def set_enabled(self, on):
        with self._lock:
            self.enabled = bool(on)
            if not self.enabled:
                self._reset_state()    # OFF 时状态清零（tick 零开销早退）
            return self.enabled

    # ---------------- 参数（GET/PUT） ----------------
    def floor_rms_um(self):
        fl = self._cfg["floor"]
        if fl["level"] == "custom":
            return float(fl["custom_rms_um"])
        return FLOOR_LEVELS[fl["level"]]

    def public(self):
        with self._lock:
            return {"ok": True, "enabled": self.enabled,
                    "floor_rms_um": self.floor_rms_um(),
                    "params": copy.deepcopy(self._cfg)}

    def update(self, patch):
        """部分合并 → 全量校验 → 原子落盘 + 热替换 + 状态清零。
        patch 可带 "enabled"（运行时开关，不落盘）。校验失败抛
        ValueError（消息含逐条 errors），原参数不污染。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            if "enabled" in patch:      # 运行时开关（同 POST /api/modal）
                self.set_enabled(patch["enabled"])
            merged = copy.deepcopy(self._cfg)
            _deep_merge(merged, {k: v for k, v in patch.items()
                                 if k != "enabled"})
            errors = validate_params(merged, self._module_names or None)
            if errors:
                raise ValueError("模态参数校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            _atomic_write(self._path, merged)
            self._cfg = merged
            self._trig.clear()
            self._reset_state()          # 参数变了，模态状态从头起

    # ---------------- 解析传播 ----------------
    def _trig_factors(self, fn_hz, zeta):
        """(w, wd, zw=ζω, e, c, s) 按 (fn 量化 0.01Hz, zeta) 缓存——稳态
        下省掉每 tick 的 exp/sin/cos（热软化时 fn_eff 缓慢漂移，量化避免
        缓存无限增长）。"""
        key = (round(fn_hz, 2), round(zeta, 4))
        t = self._trig.get(key)
        if t is None:
            w = 2.0 * math.pi * max(0.1, fn_hz)
            z = min(0.99, max(1e-4, zeta))
            wd = w * math.sqrt(max(1e-9, 1.0 - z * z))
            e = math.exp(-z * w * self._dt)
            t = (w, wd, z * w, e,
                 math.cos(wd * self._dt), math.sin(wd * self._dt))
            if len(self._trig) > 128:        # 防参数热改累积
                self._trig.clear()
            self._trig[key] = t
        return t

    def _fn_eff(self, entry, engine):
        """热软化有效频率：fn·(1−coeff·ΔT)，clamp [0.55,1.15]·fn；
        ΔT=对应加热器当前温度−22℃（无 thermal 标记 → 原值）。
        ΔT 源（2026-09-09 迭代 23）：组件级热仿真 ON 时改读热节点
        （head→bond_head、wafer→wafer_chuck 节点温度−22℃），OFF/未挂载
        回退 heater.cur（原语义不变）。"""
        fn = float(entry["fn_hz"])
        grp = entry.get("thermal")
        ts = self._cfg["thermal_softening"]
        if not grp or not ts.get("enable"):
            return fn
        dT = None
        th = getattr(engine, "thermal", None)
        if th is not None and th.enabled:
            dT = th.group_delta_t(grp)      # 热节点口径（迭代 23）
        if dT is None:
            h = engine.heaters.get(_HEATER_OF.get(grp, ""))
            if h is None:
                return fn
            dT = h.cur - REF_TEMP_C
        eff = fn * (1.0 - float(ts["coeff_per_c"]) * dT)
        return max(fn * 0.55, min(fn * 1.15, eff))

    def tick(self, dt, engine):
        """推进一个仿真 tick（engine.lock 内调用，enabled 时才有开销）。"""
        cfg = self._cfg
        k_rms = dt / RMS_TAU_S
        # ---- 轴加速度（vel 差分，限幅 ±2×amax，同参考项目执行器限幅）----
        acc = {}
        any_motion = False
        for n, ax in engine.axes.items():
            a = (ax.vel - self._last_vel[n]) / dt
            self._last_vel[n] = ax.vel
            lim = 2.0 * ax.amax
            if a > lim:
                a = lim
            elif a < -lim:
                a = -lim
            acc[n] = a
            if a != 0.0 or ax.moving:
                any_motion = True
        # ---- 地板激励强度（输出 RMS 口径：各使能模态 q 的 RSS≈等级值）----
        floor_um = self.floor_rms_um()
        n_modes_on = sum(1 for m in cfg["modes"].values() if m.get("enable"))
        mode_var_t = ((floor_um * 1e-3) ** 2 / max(1, n_modes_on)) \
            if floor_um > 0.0 else 0.0
        gauss = self._rng.gauss
        # ---- 每轴 SISO 模态 ----
        for n, entry in cfg["axes"].items():
            st = self._ax_state[n]
            if not entry.get("enable"):
                st["q"] = st["qd"] = st["var"] = 0.0
                continue
            f = float(entry.get("input_gain", 1.0)) * acc[n]
            if f != 0.0 or abs(st["q"]) > 1e-12 or abs(st["qd"]) > 1e-10:
                trig = self._trig_factors(self._fn_eff(entry, engine),
                                          float(entry["zeta"]))
                self._step(st, f, trig)
                if f == 0.0 and not any_motion \
                        and abs(st["q"]) < 1e-9 and abs(st["qd"]) < 1e-7:
                    st["q"] = st["qd"] = 0.0      # 空闲归零
            st["var"] += (st["q"] * st["q"] - st["var"]) * k_rms
        # ---- 全机 MIMO 模态 ----
        for name, m in cfg["modes"].items():
            st = self._mode_state[name]
            if not m.get("enable"):
                st["q"] = st["qd"] = st["var"] = 0.0
                continue
            f = 0.0
            for ax_name, g in m["inputs"].items():
                f += float(g) * acc.get(ax_name, 0.0)
            if mode_var_t > 0.0:
                # 白噪声随机力：Var[q]≈σ_F²·dt/(4ζω³) 反解 σ_F（输出
                # RMS 口径标定，工程近似）
                fn_eff = self._fn_eff(m, engine)
                w = 2.0 * math.pi * max(0.1, fn_eff)
                sig_f = math.sqrt(4.0 * float(m["zeta"]) * w ** 3
                                  * mode_var_t / dt)
                f += gauss(0.0, sig_f)
            if f != 0.0 or abs(st["q"]) > 1e-12 or abs(st["qd"]) > 1e-10:
                trig = self._trig_factors(self._fn_eff(m, engine),
                                          float(m["zeta"]))
                self._step(st, f, trig)
                if f == 0.0 and not any_motion \
                        and abs(st["q"]) < 1e-9 and abs(st["qd"]) < 1e-7:
                    st["q"] = st["qd"] = 0.0      # 空闲归零
            st["var"] += (st["q"] * st["q"] - st["var"]) * k_rms
        # ---- 示波器环形缓冲（恒写，空闲时补零保持时间轴连续）----
        with self._lock:         # 与 scope_public 读取互斥（deque 迭代保护）
            for i, chn in enumerate(cfg["scope_channels"]):
                buf = self._scope[i]
                if chn.startswith("axis:"):
                    buf.append(self._ax_state[chn[5:]]["q"])
                else:
                    buf.append(self._mode_state[chn[5:]]["q"])

    @staticmethod
    def _step(st, f, trig):
        """单模态精确离散二阶推进（ẍ+2ζωẋ+ω²x=−f 常值保持 dt）。"""
        w, wd, zw, e, c, s = trig
        xss = -f / (w * w)
        x0 = st["q"] - xss
        v0 = st["qd"]
        q1 = e * (x0 * c + (v0 + zw * x0) / wd * s)
        qd1 = e * (v0 * c - (zw * v0 + w * w * x0) / wd * s)
        st["q"] = xss + q1
        st["qd"] = qd1

    # ---------------- 效应读取（actual / compute / 视觉曝光路径） ----------------
    def axis_vib(self, axis):
        """该轴 SISO 模态当前位移（原生单位 mm/rad；actual() 加性分量）。"""
        return self._ax_state[axis]["q"]

    def module_offset(self, module):
        """模块 6DOF 模态叠加量 -> (t[3] mm, r[3] rad)（compute() 读取时
        合成，零 tick 开销）。"""
        t = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, 0.0]
        for name, st in self._mode_state.items():
            q = st["q"]
            if q == 0.0:
                continue
            out = self._cfg["modes"][name]["outputs"].get(module)
            if not out:
                continue
            for k in range(3):
                t[k] += q * out[k]
                r[k] += q * out[3 + k]
        return t, r

    def exposure_vib(self, mark_mod, cam_mod):
        """曝光窗振动等效：标记模块相对相机模块的横向（X/Y）模态位移
        RMS → (sigma_mm, 主导方向 rad)。同一模态对标记/相机两模块的投影
        是相干项（相对位移=q×(out_mark−out_cam)），模态间按 RSS 合成。
        返回 (0.0, 0.0) 表示无影响（调用方按原链路）。"""
        sx2 = sy2 = 0.0
        dom = (0.0, 0.0, 0.0)      # (权重, dx, dy)
        for name, st in self._mode_state.items():
            var = st["var"]
            if var <= 0.0:
                continue
            outs = self._cfg["modes"][name]["outputs"]
            om = outs.get(mark_mod) or _ZERO6
            oc = outs.get(cam_mod) or _ZERO6
            dx = om[0] - oc[0]
            dy = om[1] - oc[1]
            w2 = dx * dx + dy * dy
            if w2 > dom[0]:
                dom = (w2, dx, dy)
            sx2 += var * dx * dx
            sy2 += var * dy * dy
        sig = math.sqrt(sx2 + sy2)
        ang = math.atan2(dom[2], dom[1]) if dom[0] > 0.0 else 0.0
        return sig, ang

    def contact_pulse(self):
        """键合接触脉冲：接触沿（engine 力模型）给带 contact:true 的
        模态注入速度冲量（原生单位/s，幅度 contact_pulse.impulse_mm_s）。"""
        cp = self._cfg["contact_pulse"]
        if not cp.get("enable"):
            return
        dv = float(cp["impulse_mm_s"])
        if dv <= 0.0:
            return
        for name, m in self._cfg["modes"].items():
            if m.get("contact") and m.get("enable"):
                self._mode_state[name]["qd"] += dv
        for n, e in self._cfg["axes"].items():
            if e.get("contact") and e.get("enable"):
                self._ax_state[n]["qd"] += dv

    # ---------------- 状态/示波器 API ----------------
    @staticmethod
    def _amp(st, fn_hz):
        w = 2.0 * math.pi * max(0.1, fn_hz)
        return math.hypot(st["q"], st["qd"] / w)

    def state_public(self, engine=None):
        """GET /api/modal/state：per-mode/per-axis 实时幅值。
        直线轴 mm（附 nm 显示字段）、旋转轴 rad（附 µrad）。"""
        axes = {}
        for n, entry in self._cfg["axes"].items():
            st = self._ax_state[n]
            fn = self._fn_eff(entry, engine) if engine is not None \
                else float(entry["fn_hz"])
            amp = self._amp(st, fn)
            rms = math.sqrt(max(0.0, st["var"]))
            rotary = ac.axis_is_rotary(n)
            axes[n] = {"unit": "rad" if rotary else "mm",
                       "enable": bool(entry.get("enable")),
                       "fn_hz": round(fn, 3),
                       "q": round(st["q"], 9), "amp": round(amp, 9),
                       "rms": round(rms, 9),
                       "amp_nm" if not rotary else "amp_urad":
                           round(amp * 1e6, 3),
                       "rms_nm" if not rotary else "rms_urad":
                           round(rms * 1e6, 3)}
        modes = {}
        for name, m in self._cfg["modes"].items():
            st = self._mode_state[name]
            fn = self._fn_eff(m, engine) if engine is not None \
                else float(m["fn_hz"])
            amp = self._amp(st, fn)
            rms = math.sqrt(max(0.0, st["var"]))
            modes[name] = {"enable": bool(m.get("enable")),
                           "fn_hz": round(fn, 3),
                           "q_mm": round(st["q"], 9),
                           "amp_mm": round(amp, 9),
                           "amp_um": round(amp * 1e3, 6),
                           "rms_mm": round(rms, 9),
                           "rms_um": round(rms * 1e3, 6)}
        return {"ok": True, "enabled": self.enabled,
                "floor_level": self._cfg["floor"]["level"],
                "floor_rms_um": self.floor_rms_um(),
                "axes": axes, "modes": modes}

    def scope_public(self, ch, n):
        """GET /api/modal/scope?ch=&n=：环形缓冲抽取（时间升序）。"""
        chans = self._cfg["scope_channels"]
        if not (0 <= ch < len(self._scope)):
            raise ValueError("ch 须在 [0, %d]" % (len(self._scope) - 1))
        with self._lock:             # 与 tick 写侧互斥（deque 迭代保护）
            buf = self._scope[ch]
            data = list(buf)[-n:] if n < len(buf) else list(buf)
        label = chans[ch]
        unit = "mm"
        if label.startswith("axis:") and ac.axis_is_rotary(label[5:]):
            unit = "rad"
        return {"ok": True, "enabled": self.enabled, "ch": ch,
                "label": label, "unit": unit, "dt": self._dt,
                "n": len(data), "data": [round(v, 9) for v in data]}
