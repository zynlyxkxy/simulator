# -*- coding: utf-8 -*-
"""metrology/handoff_height_calib.py — 翻转交接接触高度标定（迭代 49，
routine flip_handoff_height，cam 恒 "up"——本项不用相机做测量，相机参数
仅为注册契约占位）。

测"键合头压到 flip 吸盘上 die 背面的实际接触位"——机器常数
flip_handoff.head_z_touch_mm（正逆向交接共用接触位；apply 组路由
head_z_touch_mm→flip_handoff 组写回）。

流程（慢速寻力范式，同 flow_engine._do_force_search）：
- 读 flip_handoff 常数 → BS_Arm_Z→head_z_safe_mm → AS_X/Y→
  as_safe_xy_mm → FA_X→fa_x_handoff_mm（均 _move_and_wait）；
- BS_Arm_Z 从 (head_z_touch_mm + 2)（名义接触位上方 2mm——若按任务书
  "−2" 则起点已在接触面下方 2mm、触力立即击穿阈值，物理不成立，故取
  +2 安全侧）以 search_speed_mm_s 下降，每 20ms 读
  SystemMachine.SS_Bonding.M_BS.oActualForce（与面板同源的含噪读数）
  与 ChuckVac/ChipVac 压力（PressureSwitch.oValue）；力≥阈值立即停轴
  （重规划到当前位，同 force_search 停法）→ Z*=BS_Arm_Z 实际位；
- 全程采样 (t_s, z_mm, force_n, chuck_vac_kpa, chip_vac_kpa) 落 series
  （抽稀到 ≤400 点）+ CSV。**不触发真实交接**（不动真空）。

参数（calibration._validate_extra 规范化）：chip_thickness_mm（默认
0.5）+ force_threshold_n（默认 2.0）+ search_speed_mm_s（默认 0.2）+
flip 有 die 守卫（start 400"flip 上无芯片"）+ 流程忙守卫。
cancel 检查点每 20ms 采样循环。收尾 BS_Arm_Z→head_z_safe_mm。

结果契约：measured={head_z_touch_mm: Z*, flip_surface_mm:
Z*−chip_thickness_mm, contact_force_n, chip_thickness_mm}；
current={head_z_touch_mm 现值}；series={t_s,z_mm,force_n,
chuck_vac_kpa,chip_vac_kpa}；csv_name/detail。"""
import math
import os
import sys
import time

# 同 as_head_center.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _log, _progress)
    from extra_routines import _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _log, _progress)
    from extra_routines import _csv_write

# 力读数点位（含噪声，与面板/flow_engine._do_force_search 同源）
FORCE_READ_PATH = "SystemMachine.SS_Bonding.M_BS.oActualForce"
_VAC_CHUCK = "SystemMachine.SS_ME.M_AIR.ChuckVac.PressureSwitch.oValue"
_VAC_CHIP = "SystemMachine.SS_ME.M_AIR.ChipVac.PressureSwitch.oValue"
_SERIES_MAX = 400      # series 抽稀上限


def _const(svc, group, key, dfl):
    try:
        v = (svc.engine.constants.get() or {}).get(group, {}).get(key)
        return v if v is not None else dfl
    except Exception:
        return dfl


def run_flip_handoff_height(svc, chip_thickness_mm, force_threshold_n,
                            search_speed_mm_s, job=None):
    """翻转交接接触高度标定主流程。参数校验（阈值/速度/片厚域 + flip 有
    die 守卫 + 流程忙）已在 calibration.start 做完。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    head_z_safe = float(_const(svc, "flip_handoff", "head_z_safe_mm", 0.0))
    as_safe = _const(svc, "flip_handoff", "as_safe_xy_mm", [0.0, 120.0])
    fa_x_handoff = float(_const(svc, "flip_handoff", "fa_x_handoff_mm",
                                -1242.35))
    head_z_touch0 = float(_const(svc, "flip_handoff", "head_z_touch_mm",
                                 -50.93))
    z_start = head_z_touch0 + 2.0       # 名义接触位上方 2mm 起降（安全侧）
    z_floor = head_z_touch0 - 2.0       # 超程下限（未触力则报错）
    with eng.lock:
        fa0 = eng.axes["FA_X"].pos
        asx0 = eng.axes["AS_X"].pos
        asy0 = eng.axes["AS_Y"].pos
    _log(svc, job, "flip handoff height: chip_t=%.3f threshold=%.2fN "
         "speed=%.3fmm/s z_start=%.4f→floor=%.4f"
         % (chip_thickness_mm, force_threshold_n, search_speed_mm_s,
            z_start, z_floor))

    series = {"t_s": [], "z_mm": [], "force_n": [], "chuck_vac_kpa": [],
              "chip_vac_kpa": []}
    rows = []
    contact_z = None
    contact_force = None
    i = 0
    try:
        # 前置走位（防撞顺序：先抬头，再退对准台，再 FA_X 到交接位）
        svc._move_and_wait("BS_Arm_Z", head_z_safe)
        svc._move_and_wait("AS_X", float(as_safe[0]))
        svc._move_and_wait("AS_Y", float(as_safe[1]))
        svc._move_and_wait("FA_X", fa_x_handoff)
        svc._move_and_wait("BS_Arm_Z", z_start)
        _log(svc, job, "descend: BS_Arm_Z %.4f → %.4f @ %.3fmm/s"
             % (z_start, z_floor, search_speed_mm_s))
        eng.move("BS_Arm_Z", z_floor, search_speed_mm_s)
        t_start = time.time()
        while True:
            _check_cancel(job, i + 1)
            with eng.lock:
                ax = eng.axes["BS_Arm_Z"]
                z = ax.pos
                moving = ax.moving
            rd = float(svc.points.get(FORCE_READ_PATH, 0.0) or 0.0)
            cv = float(svc.points.get(_VAC_CHUCK, 0.0) or 0.0)
            pv = float(svc.points.get(_VAC_CHIP, 0.0) or 0.0)
            t_rel = time.time() - t_start
            series["t_s"].append(round(t_rel, 3))
            series["z_mm"].append(round(z, 6))
            series["force_n"].append(round(rd, 4))
            series["chuck_vac_kpa"].append(round(cv, 3))
            series["chip_vac_kpa"].append(round(pv, 3))
            rows.append(["%.3f" % t_rel, "%.6f" % z, "%.4f" % rd,
                         "%.3f" % cv, "%.3f" % pv])
            i += 1
            _progress(job, i, 0, "寻力下降中 z=%.4f F=%.2fN" % (z, rd))
            if rd >= force_threshold_n:
                # 触力：原地停轴（重规划到当前位，同 force_search 停法）
                eng.move("BS_Arm_Z", z, search_speed_mm_s)
                contact_z = z
                contact_force = rd
                _log(svc, job, "触力停轴：z=%.6f F=%.2fN（阈值 %.2fN）"
                     % (z, rd, force_threshold_n))
                break
            if not moving and z <= z_floor + 1e-6:
                raise CalibError(
                    "寻力未接触：BS_Arm_Z 已到超程下限 %.4f 仍无触力（读数 "
                    "%.2fN < 阈值 %.2fN）——检查 flip 上 die/接触面高度常数"
                    % (z_floor, rd, force_threshold_n))
            time.sleep(0.02)
        # 停稳确认（10s 超时）
        t_wait = time.time()
        while time.time() - t_wait < 10.0:
            with eng.lock:
                ax = eng.axes["BS_Arm_Z"]
                if ax.in_position and not ax.moving:
                    break
            time.sleep(0.02)
        with eng.lock:
            contact_z = eng.axes["BS_Arm_Z"].pos
    finally:
        try:
            svc._move_and_wait("BS_Arm_Z", head_z_safe)
        except Exception:
            pass
        try:
            svc._move_and_wait("FA_X", fa0)
            svc._move_and_wait("AS_X", asx0)
            svc._move_and_wait("AS_Y", asy0)
        except Exception:
            pass
    fname = _csv_write(
        "flip_handoff_height",
        ["t_s", "z_mm", "force_n", "chuck_vac_kpa", "chip_vac_kpa"], rows)
    _log(svc, job, "data saved: %s" % fname)
    # series 抽稀到 ≤400 点
    n = len(series["t_s"])
    if n > _SERIES_MAX:
        step = int(math.ceil(n / float(_SERIES_MAX)))
        for k in series:
            series[k] = series[k][::step]
    return {
        "routine": "flip_handoff_height", "camera": "up",
        "measured": {"head_z_touch_mm": round(contact_z, 6),
                     "flip_surface_mm": round(contact_z
                                              - float(chip_thickness_mm), 6),
                     "contact_force_n": round(contact_force, 4),
                     "chip_thickness_mm": float(chip_thickness_mm)},
        "current": {"head_z_touch_mm": round(head_z_touch0, 6)},
        "series": series,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"force_threshold_n": float(force_threshold_n),
                   "search_speed_mm_s": float(search_speed_mm_s),
                   "z_start_mm": round(z_start, 6),
                   "z_floor_mm": round(z_floor, 6),
                   "n_samples": n,
                   "note": "慢速寻力（同 force_search）：从名义接触位上方 "
                           "2mm 以 %.3fmm/s 下降，每 20ms 读 oActualForce"
                           "≥%.2fN 立即原地停轴；全程采样带 ChuckVac/"
                           "ChipVac 压力列；不触发真实交接（不动真空）"
                           % (search_speed_mm_s, force_threshold_n)},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
