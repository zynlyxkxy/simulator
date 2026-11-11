# -*- coding: utf-8 -*-
"""metrology/bond_height_calib.py — 键合接触高度标定（迭代 49，routine
bond_height_calib，cam 恒 "up"——本项不用相机做测量，相机参数仅为注册
契约占位）。

测"键合头压到晶圆上 die 的实际接触位"——机器常数 bond.
head_z_contact_mm（**裸接触位**：无晶圆无芯片、吸盘表面贴陶瓷面的
BS_Arm_Z；有效接触位=裸值+晶圆厚+芯片厚，engine.effective_bond_
contact_mm；apply 组路由 head_z_contact_mm→bond 组写回）。

流程（慢速寻力范式，同 flow_engine._do_force_search）：
- BS_Arm_Z 从 bond.head_z_search_mm（慢速寻力段起点，裸值=裸接触位
  +2.0mm 安全余量）以 search_speed_mm_s 慢降，每 20ms 读
  SystemMachine.SS_Bonding.M_BS.oActualForce，力≥阈值立即停轴（重规划
  到当前位）→ Z*=实际接触位（含晶圆+芯片厚度的有效接触位实测）；
- measured.head_z_contact_mm = Z*−t_chip−t_wafer（裸值，候选写回）；
  measured.z_contact_mm=Z*（实测有效接触位）；
- 全程采样 (t_s, z_mm, force_n) 落 series（抽稀到 ≤400 点）+ CSV。
  收尾 BS_Arm_Z 抬回 0。

参数（calibration._validate_extra 规范化）：chip_thickness_mm（默认
0.5）+ wafer_thickness_mm（默认 1.0）+ force_threshold_n（默认 2.0）+
search_speed_mm_s（默认 0.2）+ 晶圆在位 + head 有 die 守卫（start
400"请先放入芯片和晶圆"）+ 流程忙守卫。cancel 检查点每 20ms 采样循环。

结果契约：measured={head_z_contact_mm, z_contact_mm, contact_force_n,
chip_thickness_mm, wafer_thickness_mm}；current={head_z_contact_mm 现值}；
series={t_s,z_mm,force_n}；csv_name/detail。"""
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

FORCE_READ_PATH = "SystemMachine.SS_Bonding.M_BS.oActualForce"
_SERIES_MAX = 400      # series 抽稀上限


def _const(svc, group, key, dfl):
    try:
        v = (svc.engine.constants.get() or {}).get(group, {}).get(key)
        return v if v is not None else dfl
    except Exception:
        return dfl


def run_bond_height_calib(svc, chip_thickness_mm, wafer_thickness_mm,
                          force_threshold_n, search_speed_mm_s, job=None):
    """键合接触高度标定主流程。参数校验（阈值/速度/厚度域 + 晶圆+head
    die 守卫 + 流程忙）已在 calibration.start 做完。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    z_search = float(_const(svc, "bond", "head_z_search_mm", -166.9))
    contact0 = float(_const(svc, "bond", "head_z_contact_mm", -168.93))
    # 寻力下降起点/超程下限（照 force_search 的 search_overtravel 语义：
    # 下限=期望有效接触位−1.0mm——再深会撞吸盘面沙盒对（RxRy调整球座×
    # 多孔陶瓷面），实测碰撞阈 ≈−171.6；期望有效接触位=裸接触位+晶圆厚+
    # 芯片厚（engine.effective_bond_contact_mm 同式）
    z_start = z_search
    z_floor = contact0 + float(wafer_thickness_mm) \
        + float(chip_thickness_mm) - 1.0
    with eng.lock:
        bs0 = eng.axes["BS_Arm_Z"].pos
    _log(svc, job, "bond height calib: chip_t=%.3f wafer_t=%.3f "
         "threshold=%.2fN speed=%.3fmm/s z_start=%.4f→floor=%.4f"
         % (chip_thickness_mm, wafer_thickness_mm, force_threshold_n,
            search_speed_mm_s, z_start, z_floor))

    series = {"t_s": [], "z_mm": [], "force_n": []}
    rows = []
    contact_z = None
    contact_force = None
    i = 0
    try:
        svc._move_and_wait("BS_Arm_Z", z_start)
        eng.move("BS_Arm_Z", z_floor, search_speed_mm_s)
        t_start = time.time()
        while True:
            _check_cancel(job, i + 1)
            with eng.lock:
                ax = eng.axes["BS_Arm_Z"]
                z = ax.pos
                moving = ax.moving
            rd = float(svc.points.get(FORCE_READ_PATH, 0.0) or 0.0)
            t_rel = time.time() - t_start
            series["t_s"].append(round(t_rel, 3))
            series["z_mm"].append(round(z, 6))
            series["force_n"].append(round(rd, 4))
            rows.append(["%.3f" % t_rel, "%.6f" % z, "%.4f" % rd])
            i += 1
            _progress(job, i, 0, "寻力下降中 z=%.4f F=%.2fN" % (z, rd))
            if rd >= force_threshold_n:
                eng.move("BS_Arm_Z", z, search_speed_mm_s)
                contact_force = rd
                _log(svc, job, "触力停轴：z=%.6f F=%.2fN（阈值 %.2fN）"
                     % (z, rd, force_threshold_n))
                break
            if not moving and z <= z_floor + 1e-6:
                raise CalibError(
                    "寻力未接触：BS_Arm_Z 已到超程下限 %.4f 仍无触力（读数 "
                    "%.2fN < 阈值 %.2fN）——检查晶圆/芯片厚度与接触位常数"
                    % (z_floor, rd, force_threshold_n))
            time.sleep(0.02)
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
            svc._move_and_wait("BS_Arm_Z", 0.0)
        except Exception:
            pass
    fname = _csv_write(
        "bond_height_calib", ["t_s", "z_mm", "force_n"], rows)
    _log(svc, job, "data saved: %s" % fname)
    n = len(series["t_s"])
    if n > _SERIES_MAX:
        step = int(math.ceil(n / float(_SERIES_MAX)))
        for k in series:
            series[k] = series[k][::step]
    bare = contact_z - float(chip_thickness_mm) - float(wafer_thickness_mm)
    return {
        "routine": "bond_height_calib", "camera": "up",
        "measured": {"head_z_contact_mm": round(bare, 6),
                     "z_contact_mm": round(contact_z, 6),
                     "contact_force_n": round(contact_force, 4),
                     "chip_thickness_mm": float(chip_thickness_mm),
                     "wafer_thickness_mm": float(wafer_thickness_mm)},
        "current": {"head_z_contact_mm": round(contact0, 6)},
        "series": series,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"force_threshold_n": float(force_threshold_n),
                   "search_speed_mm_s": float(search_speed_mm_s),
                   "z_start_mm": round(z_start, 6),
                   "z_floor_mm": round(z_floor, 6),
                   "n_samples": n,
                   "note": "慢速寻力（同 force_search）：从 bond.head_z_"
                           "search_mm 以 %.3fmm/s 慢降，oActualForce≥%.2fN "
                           "停轴；head_z_contact_mm=Z*−芯片厚−晶圆厚（裸"
                           "值候选写回，写回后 eff_contact=裸值+两厚度）"
                           % (search_speed_mm_s, force_threshold_n)},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
