# -*- coding: utf-8 -*-
"""metrology/temp_force_calib.py — 温度/力传感链标定（迭代 49，routines
bond_temp_calib / bond_force_calib，cam 恒 "up"——不用相机做测量，相机
参数仅为注册契约占位）。

测**传感读数链**的增益/零偏（机器常数 sensor_calib 组，apply 组路由
写回：head/wafer_temp_gain/offset_c、force_gain/offset_n → sensor_calib
组）。读数链（engine._tick_heaters_locked/_tick_force_locked）：raw=
真值×(1+gain_err)+offset_err+noise（recipe sim_sensors 注入真值旋钮）
→ 发布读数=(raw−offset_cal)/gain_cal（sensor_calib 标定补偿）。
**控制回路（PID）全程用真值 PV**，只有发布读数带注入/补偿——故标定
对"显示/量测"链生效，控制行为不变。

bond_temp_calib（heater="up"→PulseHeat_Up / "wafer"→Heat_Dw）：
每 setpoint：engine.heater_command(SetTargetTempWithPlan, Temperature=sp)
→ 等 |发布读数−SP|<settle_tol_c 稳 2s（超时 settle_timeout_s 报错含当前
温度）→ 采 oCurTemp×samples（每 sample_interval_ms 一点）→ mean/3σ；
全部点后最小二乘拟合 mean vs setpoint：r = a·T + b（纯 stdlib 闭式），
合成新标定常数 gain_new=a·gain_cur、offset_new=b·gain_cur+offset_cur
（apply 直写后发布读数精确回真值）。结束恢复加热器原 target/active
（wafer 常保温不被打断；up 随 bond 流程头冷态）。

bond_force_calib：先慢降寻力接触（同 bond_height_calib 前段，
bond.head_z_search_mm 起 search_speed 下行，≥force_threshold_n 停轴）
→ 每 setpoint：engine.bonding_force(Force=sp) → 等力控稳定（|发布读数
−SP|<0.5N 稳 1s，超时 60s）→ 采 oActualForce×samples → mean/3σ →
同样式合成 gain/offset_n。结束 Force=0 受控卸力 + BS_Arm_Z 抬回 0。

结果契约（两 routine 同构）：measured={gain, offset_c|offset_n,
points:[{setpoint,mean,s3,n}]} + **常数名同值键**（head_temp_gain/
head_temp_offset_c 或 wafer_temp_gain/wafer_temp_offset_c 或
force_gain/force_offset_n——apply 写回走常数名，fields 须显式给常数
名键）；current=对应 sensor_calib 现值；series={setpoint,samples}；
csv_name/detail。"""
import math
import os
import sys
import time

# 同 as_head_center.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _log, _mean_std, _progress)
    from extra_routines import _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _log, _mean_std, _progress)
    from extra_routines import _csv_write

FORCE_READ_PATH = "SystemMachine.SS_Bonding.M_BS.oActualForce"
_TEMP_POINT = "SystemMachine.M_Heater.%s.oCurTemp"
_SERIES_MAX = 400
# 加热器 ↔ sensor_calib 常数名映射（PulseHeat_Up↔head、Heat_Dw↔wafer）
_HEATER_CAL_KEYS = {"PulseHeat_Up": ("head_temp_gain", "head_temp_offset_c"),
                    "Heat_Dw": ("wafer_temp_gain", "wafer_temp_offset_c")}


def _const(svc, group, key, dfl):
    try:
        v = (svc.engine.constants.get() or {}).get(group, {}).get(key)
        return v if v is not None else dfl
    except Exception:
        return dfl


def _ls_fit(xs, ys):
    """最小二乘 y=a·x+b（纯 stdlib 闭式）；点数 <2 或 x 退化抛 ValueError。"""
    n = len(xs)
    if n < 2:
        raise ValueError("拟合至少 2 个点")
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-18:
        raise ValueError("setpoint 退化（全同值），无法拟合增益")
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    return a, my - a * mx


def run_bond_temp_calib(svc, heater, setpoints_c, samples,
                        sample_interval_ms, settle_tol_c, settle_timeout_s,
                        job=None):
    """温度传感链标定主流程。参数校验（heater 取值/setpoints 域/samples/
    interval/tol/timeout + 流程忙）已在 calibration.start 做完。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    hname = "PulseHeat_Up" if heater == "up" else "Heat_Dw"
    g_key, o_key = _HEATER_CAL_KEYS[hname]
    point = _TEMP_POINT % hname
    calib = {}
    try:
        calib = dict((eng.constants.get() or {}).get("sensor_calib") or {})
    except Exception:
        calib = {}
    g_cur = float(calib.get(g_key, 1.0))
    o_cur = float(calib.get(o_key, 0.0))
    # 加热器原态（finally 恢复：wafer 常保温/up 冷态不被打断）
    with eng.lock:
        h = eng.heaters[hname]
        saved_target, saved_active = h.target, h.active
    _log(svc, job, "bond temp calib: heater=%s(%s) setpoints=%s samples=%d "
         "interval=%dms tol=%.2f timeout=%.0fs"
         % (heater, hname, setpoints_c, samples, sample_interval_ms,
            settle_tol_c, settle_timeout_s))

    points = []
    series = {"setpoint_c": [], "samples": []}
    rows = []
    try:
        for si, sp in enumerate(setpoints_c):
            _check_cancel(job, si + 1)
            _progress(job, si + 1, len(setpoints_c),
                      "升温到 %.1f℃（%d/%d）" % (sp, si + 1, len(setpoints_c)))
            eng.heater_command(hname, "SetTargetTempWithPlan",
                               {"Temperature": float(sp)})
            # 等 |发布读数−SP|<tol 稳 2s
            t_sp = time.time()
            stable_since = None
            while True:
                _check_cancel(job, si + 1)
                rd = float(svc.points.get(point, 0.0) or 0.0)
                if abs(rd - sp) <= settle_tol_c:
                    if stable_since is None:
                        stable_since = time.time()
                    elif time.time() - stable_since >= 2.0:
                        break
                else:
                    stable_since = None
                if time.time() - t_sp > settle_timeout_s:
                    raise CalibError(
                        "等温超时（%.0fs）：%s 当前 %.2f℃ 未到 %.1f±%.2f℃ "
                        "稳定带" % (settle_timeout_s, hname, rd, sp,
                                    settle_tol_c))
                time.sleep(0.1)
            _log(svc, job, "setpoint %.1f℃ 到温稳定，采样 %d 点"
                 % (sp, samples))
            vals = []
            for k in range(samples):
                _check_cancel(job, si + 1)
                vals.append(float(svc.points.get(point, 0.0) or 0.0))
                rows.append(["%.2f" % sp, k, "%.3f" % vals[-1]])
                if k < samples - 1:
                    time.sleep(sample_interval_ms / 1000.0)
            m_v, s_v = _mean_std(vals)
            points.append({"setpoint_c": float(sp),
                           "mean_c": round(m_v, 4),
                           "s3_c": round(3.0 * s_v, 4),
                           "n": len(vals)})
            series["setpoint_c"].append(float(sp))
            series["samples"].append([round(v, 3) for v in vals])
            _log(svc, job, "setpoint %.1f℃: mean=%.3f℃ s3=%.3f℃"
                 % (sp, m_v, 3.0 * s_v))
    finally:
        # 恢复加热器原 target/active（不等待降温）
        try:
            if saved_active:
                eng.heater_command(hname, "SetTargetTempWithPlan",
                                   {"Temperature": saved_target})
            else:
                eng.heater_command(hname, "StopTargetTempWithPlan")
        except Exception:
            pass
    fname = _csv_write(
        "bond_temp_calib_%s" % heater,
        ["setpoint_c", "idx", "reading_c"], rows)
    _log(svc, job, "data saved: %s" % fname)
    a, b = _ls_fit([p["setpoint_c"] for p in points],
                   [p["mean_c"] for p in points])
    gain_new = a * g_cur
    offset_new = b * g_cur + o_cur
    _log(svc, job, "拟合：r=%.6f·T%+.4f → 新标定 %s=%.6f %s=%+.4f"
         % (a, b, g_key, gain_new, o_key, offset_new))
    return {
        "routine": "bond_temp_calib", "camera": "up",
        "measured": {"gain": round(gain_new, 6),
                     "offset_c": round(offset_new, 4),
                     g_key: round(gain_new, 6),
                     o_key: round(offset_new, 4),
                     "points": points},
        "current": {g_key: round(g_cur, 6), o_key: round(o_cur, 4)},
        "series": series,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"heater": heater, "heater_name": hname,
                   "samples": samples,
                   "sample_interval_ms": sample_interval_ms,
                   "settle_tol_c": settle_tol_c,
                   "settle_timeout_s": settle_timeout_s,
                   "fit_forward": {"a": round(a, 8), "b": round(b, 6)},
                   "note": "读数链标定：raw=真值×(1+gain_err)+offset_err+"
                           "noise → 发布=(raw−offset_cal)/gain_cal；拟合"
                           " mean=a·SP+b（发布读数对设定点），合成新常数 "
                           "gain_new=a·gain_cur、offset_new=b·gain_cur+"
                           "offset_cur（apply 直写后发布读数精确回真值）；"
                           "PID 控制全程用真值 PV，不受影响"},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }


def run_bond_force_calib(svc, setpoints_n, samples, sample_interval_ms,
                         force_threshold_n, job=None):
    """力传感链标定主流程。参数校验（setpoints 域/samples/interval/阈值
    + 晶圆+head die 守卫 + 流程忙）已在 calibration.start 做完。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    calib = {}
    try:
        calib = dict((eng.constants.get() or {}).get("sensor_calib") or {})
    except Exception:
        calib = {}
    g_cur = float(calib.get("force_gain", 1.0))
    o_cur = float(calib.get("force_offset_n", 0.0))
    z_search = float(_const(svc, "bond", "head_z_search_mm", -166.9))
    with eng.lock:
        bs0 = eng.axes["BS_Arm_Z"].pos
    _log(svc, job, "bond force calib: setpoints=%s samples=%d "
         "interval=%dms contact_threshold=%.2fN"
         % (setpoints_n, samples, sample_interval_ms, force_threshold_n))

    points = []
    series = {"setpoint_n": [], "samples": []}
    rows = []
    try:
        # ---- 前段：慢降寻力接触（同 bond_height_calib 口径；下限=期望
        # 有效接触位−1mm——再深撞吸盘面沙盒对，实测碰撞阈 ≈−171.6）----
        svc._move_and_wait("BS_Arm_Z", z_search)
        contact_est = float(_const(svc, "bond", "head_z_contact_mm",
                                   -168.93))
        try:
            rec = eng.recipe.get() if eng.recipe is not None else {}
            contact_est += float((rec.get("wafer") or {})
                                 .get("thickness_mm", 0.0)) \
                + float(((rec.get("film") or {}).get("dies") or {})
                        .get("thickness_mm", 0.0))
        except Exception:
            pass
        z_floor = contact_est - 1.0
        eng.move("BS_Arm_Z", z_floor, 0.2)
        _log(svc, job, "寻力下降：BS_Arm_Z %.4f → %.4f @ 0.2mm/s"
             % (z_search, z_floor))
        t_fs = time.time()
        while True:
            _check_cancel(job, 0)
            with eng.lock:
                ax = eng.axes["BS_Arm_Z"]
                z = ax.pos
                moving = ax.moving
            rd = float(svc.points.get(FORCE_READ_PATH, 0.0) or 0.0)
            if rd >= force_threshold_n:
                eng.move("BS_Arm_Z", z, 0.2)
                _log(svc, job, "触力停轴：z=%.6f F=%.2fN" % (z, rd))
                break
            if not moving and z <= z_floor + 1e-6:
                raise CalibError(
                    "寻力未接触：BS_Arm_Z 已到 %.4f 仍无触力（读数 %.2fN）"
                    "——检查晶圆/芯片/接触位常数" % (z_floor, rd))
            if time.time() - t_fs > 120.0:
                raise CalibError("寻力超时（120s）")
            time.sleep(0.02)
        t_wait = time.time()
        while time.time() - t_wait < 10.0:
            with eng.lock:
                ax = eng.axes["BS_Arm_Z"]
                if ax.in_position and not ax.moving:
                    break
            time.sleep(0.02)
        # ---- 逐 setpoint 力控稳定 + 采样 ----
        for si, sp in enumerate(setpoints_n):
            _check_cancel(job, si + 1)
            _progress(job, si + 1, len(setpoints_n),
                      "力控 %.1fN（%d/%d）" % (sp, si + 1, len(setpoints_n)))
            eng.bonding_force(float(sp), 0.0)
            t_sp = time.time()
            # 力控稳定判定：±1N 默认读数噪声下单点 |读数−SP|<0.5 只有
            # ~38% 命中（永远攒不够连续 1s）——取滑窗 1s 均值在带
            # （均值噪声 σ≈0.065N，工程惯例口径）
            win = []
            while True:
                _check_cancel(job, si + 1)
                rd = float(svc.points.get(FORCE_READ_PATH, 0.0) or 0.0)
                now = time.time()
                win.append((now, rd))
                while win and now - win[0][0] > 1.0:
                    win.pop(0)
                if len(win) >= 2 and now - win[0][0] >= 0.95:
                    mv = sum(v for _, v in win) / len(win)
                    if abs(mv - sp) <= 0.5:
                        break
                if now - t_sp > 60.0:
                    raise CalibError(
                        "力控稳定超时（60s）：目标 %.1fN 当前 %.2fN（滑窗 "
                        "均值 %.2fN）" % (sp, rd, mv if win else rd))
                time.sleep(0.05)
            vals = []
            for k in range(samples):
                _check_cancel(job, si + 1)
                vals.append(float(svc.points.get(FORCE_READ_PATH, 0.0)
                                 or 0.0))
                rows.append(["%.2f" % sp, k, "%.4f" % vals[-1]])
                if k < samples - 1:
                    time.sleep(sample_interval_ms / 1000.0)
            m_v, s_v = _mean_std(vals)
            points.append({"setpoint_n": float(sp),
                           "mean_n": round(m_v, 4),
                           "s3_n": round(3.0 * s_v, 4),
                           "n": len(vals)})
            series["setpoint_n"].append(float(sp))
            series["samples"].append([round(v, 4) for v in vals])
            _log(svc, job, "setpoint %.1fN: mean=%.3fN s3=%.3fN"
                 % (sp, m_v, 3.0 * s_v))
    finally:
        # 受控卸力 + 抬回 0（Force=0 是力控卸力命令，PID 卸到 ±tol 后
        # 自动退出力控，见 engine._tick_pid_locked）
        try:
            eng.bonding_force(0.0, 0.0)
            t_un = time.time()
            while time.time() - t_un < 30.0:
                with eng.lock:
                    if not eng.force_mode:
                        break
                time.sleep(0.05)
        except Exception:
            pass
        try:
            svc._move_and_wait("BS_Arm_Z", 0.0)
        except Exception:
            pass
    fname = _csv_write(
        "bond_force_calib", ["setpoint_n", "idx", "reading_n"], rows)
    _log(svc, job, "data saved: %s" % fname)
    a, b = _ls_fit([p["setpoint_n"] for p in points],
                   [p["mean_n"] for p in points])
    gain_new = a * g_cur
    offset_new = b * g_cur + o_cur
    _log(svc, job, "拟合：r=%.6f·F%+.4f → 新标定 force_gain=%.6f "
         "force_offset_n=%+.4f" % (a, b, gain_new, offset_new))
    return {
        "routine": "bond_force_calib", "camera": "up",
        "measured": {"gain": round(gain_new, 6),
                     "offset_n": round(offset_new, 4),
                     "force_gain": round(gain_new, 6),
                     "force_offset_n": round(offset_new, 4),
                     "points": points},
        "current": {"force_gain": round(g_cur, 6),
                    "force_offset_n": round(o_cur, 4)},
        "series": series,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"samples": samples,
                   "sample_interval_ms": sample_interval_ms,
                   "force_threshold_n": force_threshold_n,
                   "fit_forward": {"a": round(a, 8), "b": round(b, 6)},
                   "note": "力读数链标定：先慢降寻力接触（≥%.2fN 停轴），"
                           "逐 setpoint BondingByForceManual 力控稳定（|读数"
                           "−SP|<0.5N 稳 1s）后采样 oActualForce；拟合 "
                           "mean=a·SP+b，合成 gain_new=a·gain_cur、"
                           "offset_new=b·gain_cur+offset_cur；力控 PID "
                           "全程用真值 force_value，不受影响"
                           % force_threshold_n},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
