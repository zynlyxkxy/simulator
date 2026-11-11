# -*- coding: utf-8 -*-
"""metrology/rotate_center.py — 硅片台旋转中心标定（迭代 32，routine
ws_rotate_center，cam=down；迭代 35 双示教位改造）。

算法（迭代 35：**双示教位 + _fit_rot 闭式**）：示教点（≥2，calibration.
_validate_rotate_taught 校验：模板已注册/pose/pose_after 四轴分量/逐轴
软限位，pose_after 缺省=pose 同位复测）基线逐点走 pose 测世界坐标 P_k
（_taught_snap solve_pixel+WS 走位补偿口径，与 extra_routines 示教旋转
同款）；每 repeat 循环逐角度档 θ_j（±step_rad×1..n_angles 对称档，WS_Rz
走 rz0+θ_j）逐点走 **pose_after**（用户示教旋后可见位；预测走位 Δ 段已
删，下发前 _walk_limit_guard 软限位守卫）实测世界坐标 Q_kj → 全部档走
完回 rz0。每 (循环,档) 对 (P_k, Q_kj) 过 extra_routines._fit_rot 闭式
（find_rotation_center 口径）：u=P−P̄、v=Q−Q̄ → θ_ij=atan2(Σu×v,Σu·v)；
C_ij=(I−R(θ_ij))⁻¹(Q̄−R(θ_ij)P̄)（2×2 直接解，det=2(1−cosθ)——小 θ
(I−R) 病态=既有 1/θ 放大，沿用 _fit_rot 防护/文案）。中心=全部 (i,j)
拟合中心的**分量均值**、3σ=hypot(分量散布)（µm）。

输出契约：measured.rz_center_offset_mm=[cx,cy]（中心均值−**WS_Rz 机械
旋转轴心（运动链名义不动点，extra_routines._rz_axis_center）**——真值
注入 sim_align.rz_center_true_offset_um 即叠加在本点上（vision_model
collect_marks 的 pivot 语义），故 measured 与注入键同口径可比；注意与
machine_constants fine_align._note 的"相对吸盘中心"字面口径相差运动链
名义几何差（链 Rz center 占位值 vs 工具点，XY ~0.106mm，见
docs/逐键合位精对准_V1.md 枢轴口径与 error_kinematics.json
wafer_stage._note——精对准确认环对该名义差不敏感，现状一致））+
**measured.rz_center_ref_xy_mm=[作业起点 WS_LS_X, WS_LS_Y]**（标定时
机台位；迭代 35 起 apply 双写回 rz_center_offset_mm 与
rz_center_ref_xy_mm 两键，组路由 fine_align）；current 同两键现值；
拟合残差 stats.residual rms_um/s3_um（残差=(Q−R(θ)P)−(I−R(θ))C，逐
点逐 (循环,档)；nm 别名同出）；中心 3σ stats.center.s3_um（=hypot(分
量 3σ)，分量明细在 center.x/y）；series.center_x/y_mm、offset_x/y_um
与 theta_fit_urad（逐 (循环,档)，长度=repeat×2·n_angles）；CSV
ws_rotate_center_MMDD_HH_MM_SS.csv。

模板策略（实测驱动，同迭代 32）：**全程示教模板**（基线 rz0 与角度档
共用——示教位居中建制，小标记 site quad 150µm 的 REAL NCC 质心 θ 相
关偏差实测 ~10µm/rad 级）。反面教材（实测排除）：①晶圆全局 2mm 大
cross——渐晕下跨旋转态 NCC 质心系统差 ~µm 级/0.014rad（REAL−FAST 实
测），经 1/θ 放大别名进中心拟合 ~50µm/rad；②角度档就地建默认 ROI 临
时模板——偏中心建模板+背景主导，θ 相关偏差更大（实测拟合偏差
~450µm）。示教点宜选小标记（site quad——小模板走连续几何角
GEOM_ANGLE_MAX_TPL_PX=1.5Mpx² 以下，角度估计连续不量化）。
**双示教位可用性**（迭代 35）：pose_after 为每点单一示教位——角度档
扫 ±n×step 时各档的标记可见性由示教点杠杆×角度摆幅 < FOV 半宽保
证（宜近旋转中心小杠杆示教，或按最大档示教 pose_after）；缺省
pose_after=pose 时须杠杆×n_angles×step_rad ≪ FOV 半宽。
真值注入验证口径：recipe sim_align.rz_center_true_offset_um=[300,-200]
µm + 误差链 OFF（/api/errormaps six_dof_enabled=False——旋转台跑出
dx/dy 误差地图按 (ΣMᵀM)⁻¹ΣMᵀρ 结构别名进拟合 ~20µm，属设备真值固有
系统差，真机同此口径；验收对算法本身，隔离误差链）时拟合中心偏差
<10µm。

cancel 检查点每循环首+每角度档首；finally 恢复 AS/WS/WS_Rz 作业前轴位、
光源、视觉模式（与 extra_routines 旋转重复性同款）。异常抛给
calibration._run_simple 收 error 态。"""
import math
import os
import sys
import time

# 同 extra_routines.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _log, _mean_std,
                               _progress, _sleep_ms)
    from extra_routines import (_csv_write, _chuck_center, _fit_rot,
                                _rz_axis_center, _rz_pivot_belief,
                                _taught_go_pose, _taught_snap,
                                _walk_limit_guard)
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _log, _mean_std,
                               _progress, _sleep_ms)
    from extra_routines import (_csv_write, _chuck_center, _fit_rot,
                                _rz_axis_center, _rz_pivot_belief,
                                _taught_go_pose, _taught_snap,
                                _walk_limit_guard)


def run_ws_rotate_center(svc, repeat, wait_ms, step_rad, n_angles, taught,
                         job=None):
    """硅片台旋转中心标定主流程（迭代 35 双示教位）。参数校验
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
    chuck = _chuck_center(eng)        # 吸盘心工具点（ws0 帧；detail 参照）
    axc = _rz_axis_center(eng)        # Rz 机械轴心（ws0 帧；measured 基准）
    # 角度档：±step×1..n 对称（正档升序后负档升序——同侧连续走，行程
    # 利用与方向对称兼顾）
    angles = [step_rad * j for j in range(1, n_angles + 1)]
    angles += [-v for v in angles]
    _log(svc, job, "ws rotate center: n_points=%d step=%.6frad n_angles=%d "
         "repeat=%d angles=%s pivot=(%.3f,%.3f) axis_center=(%.3f,%.3f)"
         % (n_pt, step_rad, n_angles, repeat,
            ["%.4f" % a for a in angles], pivot[0], pivot[1],
            axc[0], axc[1]))

    centers = []                     # 每 (循环,档) 一个拟合中心 (cx, cy)
    thetas = []                      # 每 (循环,档) 一个拟合角 θ_ij（rad）
    resids_nm = []                   # 拟合残差模长（nm，逐 (i,j,k)）
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
             % " ".join("p%d=(%.2f,%.2f)w=(%.4f,%.4f)"
                        % (k + 1, base[k][0], base[k][1],
                           base[k][2][0], base[k][2][1])
                        for k in range(n_pt)))
        p_base = [b[2] for b in base]
        for i in range(repeat):
            _check_cancel(job, i + 1)
            _progress(job, i + 1, repeat,
                      "第 %d/%d 次旋转中心循环" % (i + 1, repeat))
            snaps = []               # 每档 [Q_world × n_pt]
            for j, th in enumerate(angles):
                _check_cancel(job, i + 1)
                svc._move_and_wait("WS_Rz", rz0 + th)
                _sleep_ms(wait_ms)
                qj = []
                for k in range(n_pt):
                    # 角度档实测：走 pose_after（用户示教旋后可见位——双
                    # 示教位改造，预测走位 Δ 段已删；各档可见性由示教点
                    # 杠杆×摆幅 <FOV 半宽保证，宜近旋转中心小杠杆示教）
                    pa = points[k]["pose_after"]
                    _walk_limit_guard(svc, k, points[k]["tpl"],
                                      pa["WS_LS_X"], pa["WS_LS_Y"], th)
                    _taught_go_pose(svc, pa)
                    _sleep_ms(wait_ms)
                    q = _taught_snap(svc, points, k, ws0, cam,
                                     "角度档 %+0.4f 点#%d 实测" % (th, k + 1))
                    qj.append(q[2])
                    rows.append([i, "%.6f" % th, k + 1,
                                 "%.4f" % q[0], "%.4f" % q[1],
                                 "%.6f" % q[2][0], "%.6f" % q[2][1]])
                snaps.append(qj)
            svc._move_and_wait("WS_Rz", rz0)         # 回零进下一循环
            _sleep_ms(wait_ms)
            # 每 (循环,档) _fit_rot 闭式：(θ_ij, C_ij)——u=P−P̄、v=Q−Q̄ →
            # θ=atan2(Σu×v,Σu·v)；C=(I−R(θ))⁻¹(Q̄−R(θ)P̄)（2×2 直接解，
            # 小 θ 病态防护/文案在 _fit_rot 内，沿用既有 1/θ 放大口径）
            for j, th in enumerate(angles):
                th_f, cij = _fit_rot(p_base, snaps[j])
                centers.append(cij)
                thetas.append(th_f)
                c_, s_ = math.cos(th_f), math.sin(th_f)
                m11 = m22 = 1.0 - c_
                m12, m21 = s_, -s_
                for k in range(n_pt):
                    p = p_base[k]
                    rp = (c_ * p[0] - s_ * p[1], s_ * p[0] + c_ * p[1])
                    rx = snaps[j][k][0] - rp[0]
                    ry = snaps[j][k][1] - rp[1]
                    ex = rx - (m11 * cij[0] + m12 * cij[1])
                    ey = ry - (m21 * cij[0] + m22 * cij[1])
                    resids_nm.append(math.hypot(ex, ey) * 1e6)   # mm→nm
                _log(
                    svc, job,
                    "cycle %d θ=%+.4f: fit theta=%.3furad center=(%.6f,"
                    "%.6f)mm offset=(%.1f,%.1f)um"
                    % (i + 1, th, th_f * 1e6, cij[0], cij[1],
                       (cij[0] - axc[0]) * 1000.0,
                       (cij[1] - axc[1]) * 1000.0))
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
        "ws_rotate_center",
        ["index", "theta_rad", "point", "col", "row",
         "world_x_mm", "world_y_mm"], rows)
    _log(svc, job, "data saved: %s" % fname)
    cxs = [c[0] for c in centers]
    cys = [c[1] for c in centers]
    m_cx, s_cx = _mean_std(cxs)
    m_cy, s_cy = _mean_std(cys)
    off = (m_cx - axc[0], m_cy - axc[1])
    rms_nm = math.sqrt(sum(r * r for r in resids_nm) / len(resids_nm)) \
        if resids_nm else 0.0
    _mr, s_r = _mean_std(resids_nm) if resids_nm else (0.0, 0.0)
    s3x_um = 3 * s_cx * 1000.0
    s3y_um = 3 * s_cy * 1000.0
    try:
        fa_mc = (eng.constants.get() or {}).get("fine_align") or {}
        cur = [float(v) for v in
               fa_mc.get("rz_center_offset_mm") or [0.0, 0.0]]
        cur_ref = [float(v) for v in
                   fa_mc.get("rz_center_ref_xy_mm") or [0.0, 0.0]]
    except Exception:
        cur, cur_ref = [0.0, 0.0], [0.0, 0.0]
    return {
        "routine": "ws_rotate_center", "camera": "down",
        "repeat": repeat, "wait_ms": wait_ms,
        # 迭代 35：measured 双键——offset（中心均值−Rz 机械轴心）+ ref_xy
        # （标定时机台位 [WS_LS_X, WS_LS_Y]@作业起点）；apply 双写回
        "measured": {"rz_center_offset_mm": [round(off[0], 6),
                                             round(off[1], 6)],
                     "rz_center_ref_xy_mm": [round(ws0[0], 6),
                                             round(ws0[1], 6)]},
        "current": {"rz_center_offset_mm": [round(v, 6) for v in cur],
                    "rz_center_ref_xy_mm": [round(v, 6) for v in cur_ref]},
        "series": {"center_x_mm": [round(v, 6) for v in cxs],
                   "center_y_mm": [round(v, 6) for v in cys],
                   "offset_x_um": [round((v - axc[0]) * 1000.0, 3)
                                   for v in cxs],
                   "offset_y_um": [round((v - axc[1]) * 1000.0, 3)
                                   for v in cys],
                   "theta_fit_urad": [round(v * 1e6, 3) for v in thetas]},
        "stats": {"center": {
                      # 客户端口径（首选）：整体 3σ=hypot(分量 3σ)——中心=
                      # 全 (循环,档) 拟合中心分量均值，散布=逐 (循环,档)
                      # 拟合散布；分量明细保留在 x/y 子键
                      "s3_um": round(math.hypot(s3x_um, s3y_um), 3),
                      "x": {"mean_mm": round(m_cx, 6),
                            "s3_um": round(s3x_um, 3)},
                      "y": {"mean_mm": round(m_cy, 6),
                            "s3_um": round(s3y_um, 3)}},
                  # 客户端口径（首选 stats.residual.rms_um/s3_um；
                  # nm 别名保留）
                  "residual": {"rms_um": round(rms_nm / 1000.0, 4),
                               "s3_um": round(3 * s_r / 1000.0, 4),
                               "rms_nm": round(rms_nm, 2),
                               "s3_nm": round(3 * s_r, 2)}},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"step_rad": step_rad, "n_angles": n_angles,
                   "angles_rad": [round(v, 6) for v in angles],
                   "n_points": n_pt,
                   "rz_axis_center_mm": [round(axc[0], 6), round(axc[1], 6)],
                   "chuck_center_mm": [round(chuck[0], 6),
                                       round(chuck[1], 6)],
                   "rz_pivot_belief_mm": [round(pivot[0], 6),
                                          round(pivot[1], 6)],
                   "note": "迭代 35 双示教位：基线走 pose 测 P_k、每角度档"
                           "走 pose_after 实测 Q_kj → 每 (循环,档) _fit_rot "
                           "闭式 (θ_ij,C_ij)（θ=atan2(Σu×v,Σu·v)、C=(I−R)⁻¹"
                           "(Q̄−RP̄) 2×2 直接解，小 θ (I−R) 病态=1/θ 放大，"
                           "沿用现有防护）；measured=拟合中心分量均值−Rz 机"
                           "械轴心（运动链名义不动点，与真值注入 sim_align."
                           "rz_center_true_offset_um 同口径）+ rz_center_ref"
                           "_xy_mm=标定时机台位（apply 双写回）；全程示教模"
                           "板（宜小标记：大 cross/就地默认 ROI 模板的旋转"
                           "态 NCC 偏差会经 1/θ 放大别名进拟合，实测不可用）",
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
