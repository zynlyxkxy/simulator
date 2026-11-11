# -*- coding: utf-8 -*-
"""metrology/rotate_gain.py — 硅片台旋转角度增益标定（迭代 32A，routine
ws_rotate_gain，cam=down；迭代 35 双示教位改造）。

测**实际旋转/指令旋转的增益系数**（写回 machine_constants fine_align.
rz_gain——flow_engine 精对准补偿按 物理角=轴位增量×rz_gain×rz_sign 换算）。
数据复用迭代 32 直接测角法（extra_routines._run_ws_rotate_taught 同款；
迭代 35 双示教位）：示教点（≥2，calibration 校验）基线逐点走 pose 测世
界坐标 P_k（_taught_snap 链）；每 repeat 循环逐角度档 θ_j（±step_rad×
1..n_angles 对称，WS_Rz 走 rz0+θ_j）逐点走 **pose_after**（用户示教旋后
可见位；预测走位 Δ 段已删，下发前 _walk_limit_guard 软限位守卫）实测
Q_kj → 每 (循环,档) extra_routines._fit_rot 闭式实测旋转角 θ_ij
（u=P−P̄、v=Q−Q̄ → θ=atan2(Σu×v,Σu·v)，世界系向量集——示教点各自
拍摄，像素集退化）→ 回 rz0。

拟合：θ_actual = gain×θ_cmd（+offset）。**过原点 LS 为主输出**
gain=Σθa·θc/Σθc²（per-repeat，θc=指令档 θ_cmd_j）；带截距 2 参数 LS
诊断（截距=固定角偏差 offset_urad，带截距斜率 gain_intercept——对称档
下与过原点恒等）。与旋转中心标定的差异：_fit_rot 平移自由，旋转台跑
出/杠杆误差不进增益（真值增益=物理角/轴位增量；sim 真值注入通道=
error_maps/low_order.json 的 WS_Rz scale_ppm——真值链 six_dof 须 ON，
i32 真值注入 1.001（scale_ppm=1000）测回口径）。

输出契约：measured.rz_gain（无量纲，~1.0）+ current.rz_gain；
stats.gain{mean,std,s3}、stats.offset_urad{mean,std,s3}（带截距诊断）、
stats.gain_intercept{mean,std,s3}（带截距斜率，对称档=gain）；series.
gain/gain_intercept/offset_urad（逐 repeat）+ series.theta_actual_urad
（逐 repeat×角度档嵌套）；CSV ws_rotate_gain_MMDD_HH_MM_SS.csv；apply
fields=["rz_gain"] 经 calibration.apply 组路由写回 fine_align 组
（machine_constants FINE_ALIGN_FIELDS "num" [1e-6,1e6] 原生支持）。
cancel 检查点每循环首+每角度档首；finally 恢复 AS/WS/WS_Rz 作业前轴位、
光源、视觉模式（与 rotate_center 同款）。异常抛给 calibration.
_run_simple 收 error 态。"""
import math
import os
import sys
import time

# 同 extra_routines.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _log, _mean_std, _progress,
                               _sleep_ms)
    from extra_routines import (_csv_write, _fit_rot, _rz_pivot_belief,
                                _taught_go_pose, _taught_snap,
                                _walk_limit_guard)
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _log, _mean_std, _progress,
                               _sleep_ms)
    from extra_routines import (_csv_write, _fit_rot, _rz_pivot_belief,
                                _taught_go_pose, _taught_snap,
                                _walk_limit_guard)


def run_ws_rotate_gain(svc, repeat, wait_ms, step_rad, n_angles, taught,
                       job=None):
    """硅片台旋转角度增益标定主流程（迭代 35 双示教位）。参数校验
    （taught_points[pose/pose_after]/step_rad/n_angles/repeat/wait_ms/
    WS_Rz 行程/逐轴软限位）已在 calibration.start 做完
    （_validate_rotate_sweep）。"""
    t0 = time.time()
    eng = svc.engine
    if eng.workpieces is None or not eng.workpieces.wafer["present"]:
        raise RuntimeError("晶圆不在位（先上片）")
    points = taught["points"]
    n_pt = len(points)
    cam = svc.vision.cams["down"].model
    rt = svc.vision.cams["down"]
    old_mode = svc.vision.mode
    old_light = bool(svc.points.get(cam.light_point, False))
    # 作业前轴位（finally 恢复）：AS/WS/WS_Rz
    restore_axes = ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y", "WS_Rz")
    saved = {}
    with eng.lock:
        for a in restore_axes:
            saved[a] = eng.axes[a].pos
    rz0 = saved["WS_Rz"]
    ws0 = (saved["WS_LS_X"], saved["WS_LS_Y"])   # 世界补偿基准（作业起点）
    pivot = _rz_pivot_belief(eng)     # 认知枢轴（ws0 帧；detail 参照）
    # 角度档：±step×1..n 对称（Σθc=0——过原点与带截距斜率恒等，固定角偏
    # 差只进截距）
    angles = [step_rad * j for j in range(1, n_angles + 1)]
    angles += [-v for v in angles]
    _log(svc, job, "ws rotate gain: n_points=%d step=%.6frad n_angles=%d "
         "repeat=%d angles=%s pivot=(%.3f,%.3f)"
         % (n_pt, step_rad, n_angles, repeat,
            ["%.4f" % a for a in angles], pivot[0], pivot[1]))

    gains, gains_ic, offsets = [], [], []
    theta_series = []                # 逐 repeat：[θ_actual ×角度档]（rad）
    rows = []
    base = None
    try:
        if old_mode != "real":
            svc.vision.set_mode("real")
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        base = []                    # 基线（rz0）逐点世界坐标 P_k
        for k in range(n_pt):
            _taught_go_pose(svc, points[k]["pose"])
            _sleep_ms(wait_ms)
            base.append(_taught_snap(svc, points, k, ws0, cam, "基线首测"))
        _log(svc, job, "baseline: %s"
             % " ".join("p%d=(%.2f,%.2f)" % (k + 1, base[k][0], base[k][1])
                        for k in range(n_pt)))
        p_base = [b[2] for b in base]
        for i in range(repeat):
            _check_cancel(job, i + 1)
            _progress(job, i + 1, repeat,
                      "第 %d/%d 次旋转增益循环" % (i + 1, repeat))
            theta_i = []
            for th in angles:
                _check_cancel(job, i + 1)
                svc._move_and_wait("WS_Rz", rz0 + th)
                _sleep_ms(wait_ms)
                qj = []
                for k in range(n_pt):
                    # 角度档实测：走 pose_after（双示教位改造，预测走位
                    # Δ 段已删；下发前软限位守卫）
                    pa = points[k]["pose_after"]
                    _walk_limit_guard(svc, k, points[k]["tpl"],
                                      pa["WS_LS_X"], pa["WS_LS_Y"], th)
                    _taught_go_pose(svc, pa)
                    _sleep_ms(wait_ms)
                    qj.append(_taught_snap(svc, points, k, ws0, cam,
                                           "角度档 %+0.4f 实测" % th)[2])
                # 每 (循环,档) _fit_rot 闭式实测旋转角（平移自由，旋转台
                # 跑出不进增益）
                th_a, _c = _fit_rot(p_base, qj)
                theta_i.append(th_a)
                rows.append([i, "%.6f" % th, "%.6f" % (th_a * 1e6),
                             "%.6f" % ((th_a - th) * 1e6)])
                _log(svc, job, "cycle %d θ=%+.4f: actual=%.3furad "
                     "err=%.3furad" % (i + 1, th, th_a * 1e6,
                                       (th_a - th) * 1e6))
            svc._move_and_wait("WS_Rz", rz0)         # 回零进下一循环
            _sleep_ms(wait_ms)
            theta_series.append(theta_i)
            # 过原点 LS：gain=Σθaθc/Σθc²（主输出）；带截距 2 参数 LS 诊断
            num = sum(a * c for a, c in zip(theta_i, angles))
            den = sum(c * c for c in angles)
            g0 = num / den if den > 1e-18 else 1.0
            xb = sum(angles) / len(angles)
            yb = sum(theta_i) / len(theta_i)
            sxx = sum((c - xb) ** 2 for c in angles)
            g1 = (sum((c - xb) * (a - yb) for c, a in zip(angles, theta_i))
                  / sxx if sxx > 1e-18 else g0)
            off = yb - g1 * xb
            gains.append(g0)
            gains_ic.append(g1)
            offsets.append(off)
            _log(svc, job, "cycle %d: gain=%.6f offset=%.3furad"
                 % (i + 1, g0, off * 1e6))
            _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    finally:
        for a in restore_axes:
            try:
                svc._move_and_wait(a, saved[a])
            except Exception:
                pass
        try:
            svc.vision._set_light(rt, old_light)
        except Exception:
            pass
        if old_mode != "real":
            try:
                svc.vision.set_mode(old_mode)
            except Exception:
                pass
    fname = _csv_write(
        "ws_rotate_gain",
        ["index", "theta_cmd_rad", "theta_actual_urad", "err_urad"], rows)
    _log(svc, job, "data saved: %s" % fname)
    m_g, s_g = _mean_std(gains)
    m_gi, s_gi = _mean_std(gains_ic)
    m_o, s_o = _mean_std(offsets)
    try:
        cur = float(((eng.constants.get() or {}).get("fine_align") or {})
                    .get("rz_gain", 1.0))
    except Exception:
        cur = 1.0
    return {
        "routine": "ws_rotate_gain", "camera": "down",
        "repeat": repeat, "wait_ms": wait_ms,
        "measured": {"rz_gain": round(m_g, 6)},
        "current": {"rz_gain": round(cur, 6)},
        "series": {"gain": [round(v, 6) for v in gains],
                   "gain_intercept": [round(v, 6) for v in gains_ic],
                   "offset_urad": [round(v * 1e6, 3) for v in offsets],
                   "theta_actual_urad": [
                       [round(a * 1e6, 3) for a in ti]
                       for ti in theta_series]},
        "stats": {"gain": {"mean": round(m_g, 6),
                           "std": round(s_g, 6),
                           "s3": round(3 * s_g, 6)},
                  "gain_intercept": {"mean": round(m_gi, 6),
                                     "std": round(s_gi, 6),
                                     "s3": round(3 * s_gi, 6)},
                  "offset_urad": {"mean": round(m_o * 1e6, 3),
                                  "std": round(s_o * 1e6, 3),
                                  "s3": round(3 * s_o * 1e6, 3)}},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"step_rad": step_rad, "n_angles": n_angles,
                   "angles_rad": [round(v, 6) for v in angles],
                   "n_points": n_pt,
                   "rz_pivot_belief_mm": [round(pivot[0], 6),
                                          round(pivot[1], 6)],
                   "note": "迭代 35 双示教位：每 (循环,档) _fit_rot 闭式实测"
                           " θ_ij 对指令 θ_cmd_j 过原点 LS gain=Σθaθc/Σθc²"
                           "（截距=固定角偏差诊断）；_fit_rot 平移自由，旋转"
                           "台跑出不进增益",
                   "taught_points": [
                       {"tpl": pt["tpl"],
                        "pose": {a: round(pt["pose"][a], 6)
                                 for a in pt["pose"]},
                        "pose_after": {a: round(pt["pose_after"][a], 6)
                                       for a in pt["pose_after"]}}
                       for pt in points]},
        "context": None, "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
