# -*- coding: utf-8 -*-
"""metrology/bs_rxry_axis.py — 键合头调平轴 Rx/Ry 旋转轴线标定（迭代 49，
routine bs_rxry_axis，cam 恒 "up"）。

测 BS_CAL_Rx/BS_CAL_Ry 两调平轴的**旋转轴线空间表示**（轴上一点 + 方向
向量，世界坐标）——机器常数 bs_rxry_axis 组（rx/ry_axis_point_mm +
rx/ry_axis_dir，apply 组路由写回；默认值与 error_kinematics.json
bond_head 链 R 链元真值对齐）。

算法（参考 测校/一些测校仿真/RxRy旋转轴标定/rxry_axis_sim.py，SVD 逆解
忠实移植为纯 numpy lazy-import，不用 scipy）：
- 前置位姿照 extra_routines._pose_for_head_shot：AS_Y=6、AS_Z=0、
  BS_Arm_Z=−118.33（die 底面进上相机焦面）、BS_CAL_Rx/Ry→0；
- die 4 角标（recipe film.marks ratio ±0.76 × die 半宽；默认 10mm die
  → ±3.8mm 四角）逐标记测空间坐标 (X,Y,Z)：
  · XY：AS_X/Y 把上相机送到该角标（相机零位世界=align_stage 工具点
    (−42.5,23.5)+mount(0,−22.5)=(−42.5,1.0)，AS 目标=标记世界−零位），
    identify→_centroid→_context（solve_pixel 含 dir/swap/rot_mrad 认知）
    得世界 XY，再按 δ=标记世界−相机世界迭代精找一次取值（相机随台动，
    新位=当前 AS 位+δ，同 as_head_center 符号约定）；
  · Z：AS 走位共焦偏移（focus_level.confocal_cam_dx/dy_mm，探头相对
    相机 (0,−25)）把 up 共焦探头落到该点，同步读 confocal up（read
    返回 reading_um/z_focus），Z = z_focus − (reading_um −
    confocal_mid_up_um)/1000（中点共面约定，与 confocal.py 读数模型
    同式）；每点 repeat 次取均值；
- 对每轴（Rx、Ry）三相 phase=−a/0/+a（a=angle_mrad/1000 rad，行程
  ±0.00174533 rad 内由 start 校验）各测 4 点；
- SVD：phase=−a 与 phase=+a 两组 4 点刚体拟合（estimate_rotation_
  axis_svd：质心对齐→H=(P−cP)ᵀ(Q−cQ)→SVD→R=VUᵀ（det<0 翻列）→
  t=cQ−R·cP）→ 轴方向 dir（rotvec_from_R：Rodrigues 逆）+ 轴上点
  point（lstsq(I−R,t) 在小转角下病态，弃用——改 ⊥dir 平面内 2D 闭式
  旋转中心，沿轴分量取测量重心投影）；方向符号归一化：Rx 的 dir x 分量强制
  为正、Ry 的 y 分量强制为正（为负则整体取反）。

参数（calibration._validate_extra 规范化）：template_name（缺省=机器常数
vision_marks.tpl_die_head；给了须已注册）+ angle_mrad（默认 1.5，
[0.2,1.7]——BS_CAL 行程 ±0.00174533 rad）+ repeat（默认 3，[1,10]）+
wait_ms（默认 10）+ head 有 die 守卫（start 400）+ 流程忙守卫。
cancel 检查点每相首；轴位/光源/视觉模式 finally 恢复（先抬 BS_Arm_Z
再回 AS，防撞）。

结果契约（标定程序形态，GUI 按此解析）：
measured={rx_axis_point_mm:[x,y,z], rx_axis_dir:[x,y,z],
ry_axis_point_mm:[...], ry_axis_dir:[...], rx_rmse_mm, ry_rmse_mm,
angle_mrad}；current=机器常数 bs_rxry_axis 组现值（同 4 键）；
points={"RX":{"neg":[[x,y,z]×4],"zero":[...],"pos":[...]},"RY":{...}}；
csv_name/detail。CSV 两份：nodes（行基准 axis,phase,mark_id,rep,col,row,
AS_X/Y/Z,BS_Arm_Z,BS_CAL_Rx/Ry,mark_world_x/y/z,confocal_reading_um,
cam_world_x/y）+ summary（轴点/方向/rmse/angle/各标记残差）。"""
import math
import os
import sys
import time

# 同 as_head_center.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _check_marks, _context,
                               _log, _mean_std, _progress, _sleep_ms)
    from extra_routines import _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _check_marks, _context,
                               _log, _mean_std, _progress, _sleep_ms)
    from extra_routines import _csv_write

_NP = None


def _np():
    """numpy 惰性导入（runtime 纯标准库约束的例外通道，同
    vision_model.np() 先例）。"""
    global _NP
    if _NP is None:
        import numpy
        _NP = numpy
    return _NP


# ---------------- SVD 逆解（移植自 rxry_axis_sim.py，纯 numpy） ----------------
def _rotvec_from_R(R):
    """Rodrigues 逆：旋转矩阵 → (单位轴, 角度 rad)。角度过小抛错。"""
    np = _np()
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    sin_t = np.linalg.norm(w) / 2.0
    cos_t = (np.trace(R) - 1.0) / 2.0
    angle = float(np.arctan2(sin_t, cos_t))
    if angle < 1e-9:
        raise ValueError("旋转角度过小，无法准确计算旋转轴")
    return w / (2.0 * sin_t), angle


def _estimate_rotation_axis_svd(points_before, points_after):
    """SVD 求刚体变换并提取旋转轴（与参考脚本逐步对应）。
    返回 (axis_point, axis_direction, angle_rad, rmse)。"""
    np = _np()
    P = np.asarray(points_before, float)
    Q = np.asarray(points_after, float)
    if len(P) != len(Q):
        raise ValueError("前后点对数量不一致")
    cP, cQ = P.mean(axis=0), Q.mean(axis=0)
    H = (P - cP).T @ (Q - cQ)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    R = V @ U.T
    if np.linalg.det(R) < 0:
        V[:, 2] *= -1
        R = V @ U.T
    t = cQ - R @ cP
    rmse = float(np.sqrt(np.mean(np.sum((P @ R.T + t - Q) ** 2, axis=1))))
    axis_dir, angle = _rotvec_from_R(R)
    axis_point, *_ = np.linalg.lstsq(np.eye(3) - R, t, rcond=None)
    return axis_point, axis_dir, angle, rmse


def _const(svc, group, key, dfl):
    """读机器常数（异常回退 dfl）。"""
    try:
        v = (svc.engine.constants.get() or {}).get(group, {}).get(key)
        return v if v is not None else dfl
    except Exception:
        return dfl


# 上相机零位世界 XY（AS_X/Y=0 时）：align_stage 工具点 (-42.5,23.5,910)
# + up 相机 mount (0,−22.5) = (−42.5, 1.0)（as_head_center.py 模块头同口径）
_CAM_ZERO_XY = (-42.5, 1.0)
# 键合头吸盘心名义世界 XY（error_kinematics bond_head 工具点）
_HEAD_CENTER_XY = (-42.5, 7.0)


def _mark_nominals(svc):
    """die 4 角标的世界名义位列表 [(mark_id, x, y)]：头心 + die site +
    ratio×die 半宽（含 die rz——head_die_marks 不随 die rz 转，见
    vision_model._die_marks 调用口径；此处按实体模型同式直接给）。"""
    from calibration import CalibError
    eng = svc.engine
    rec = eng.recipe.get() if eng.recipe is not None else None
    if rec is None:
        raise CalibError("recipe not attached（无法读 die 标记布局）")
    fm = (rec.get("film") or {}).get("marks") or {}
    ratio = fm.get("ratio") or []
    per = min(max(0, int(fm.get("per_die", len(ratio)))), len(ratio))
    if per < 3:
        raise CalibError("film.marks per_die=%d < 3，SVD 拟合至少 3 点" % per)
    dsz = (rec.get("film") or {}).get("dies") or {}
    hx = 0.5 * float(dsz.get("size_x_mm", 4.0))
    hy = 0.5 * float(dsz.get("size_y_mm", 4.0))
    site = [0.0, 0.0]
    try:
        for d in eng.workpieces.dies:
            if d.get("carrier") == "head":
                site = list(d.get("site") or [0.0, 0.0])
                break
    except Exception:
        pass
    out = []
    for i in range(per):
        rx, ry = float(ratio[i][0]), float(ratio[i][1])
        out.append((i, _HEAD_CENTER_XY[0] + site[0] + rx * hx,
                    _HEAD_CENTER_XY[1] + site[1] + ry * hy))
    return out


def run_bs_rxry_axis(svc, template_name, angle_mrad, repeat, wait_ms,
                     job=None):
    """BS_CAL_Rx/Ry 旋转轴线标定主流程。参数校验（模板/角度/repeat/
    wait_ms/head die 守卫/流程忙）已在 calibration.start 做完。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    cam = svc.vision.cams["up"].model
    rt = svc.vision.cams["up"]
    a = float(angle_mrad) / 1000.0            # rad
    conf_dx = float(_const(svc, "focus_level", "confocal_cam_dx_mm", 0.0))
    conf_dy = float(_const(svc, "focus_level", "confocal_cam_dy_mm", -25.0))
    conf_mid_up = float(_const(svc, "focus_level", "confocal_mid_up_um",
                               350.0))
    if eng.confocal is None:
        raise CalibError("confocal not attached（Z 向测量不可用）")
    marks_nom = _mark_nominals(svc)
    n_mark = len(marks_nom)
    old_light = bool(svc.points.get(cam.light_point, False))
    restore_axes = ("BS_CAL_Rx", "BS_CAL_Ry", "BS_Arm_Z", "AS_Z",
                    "AS_X", "AS_Y")
    saved = {}
    with eng.lock:
        for ax in restore_axes:
            saved[ax] = eng.axes[ax].pos
    _log(svc, job, "bs rxry axis: template=%s angle=%.3fmrad repeat=%d "
         "wait_ms=%s marks=%d conf_offset=(%.3f,%.3f)"
         % (template_name, angle_mrad, repeat, wait_ms, n_mark,
            conf_dx, conf_dy))
    # (axis, phase, mark_id, rep) -> 测量行
    pts = {"RX": {"neg": [], "zero": [], "pos": []},
           "RY": {"neg": [], "zero": [], "pos": []}}
    rows = []                               # nodes CSV 行
    total = 2 * 3 * n_mark

    def _measure_point(mark_id, mx, my, phase_tag, axis_tag, rep):
        """单点测量：相机 XY（迭代精找一次）+ 共焦 Z（repeat 已在调用侧
        逐次调用）。返回 (wx, wy, wz, col, row, reading_um, cwx, cwy,
        asx, asy)。"""
        # AS 目标 = 标记世界 − 相机零位（相机零位世界=(-42.5,1.0)）
        svc._move_and_wait("AS_X", mx - _CAM_ZERO_XY[0])
        svc._move_and_wait("AS_Y", my - _CAM_ZERO_XY[1])
        _sleep_ms(wait_ms)
        wx = wy = col = row = None
        for _it in range(2):                # 迭代精找一次再取值
            res = svc._identify("up", template_name)["result"]
            col, row = svc._centroid(res, cam.score_threshold)
            ctx = _context(svc, "up", cam, col, row)
            mw, cw = ctx.get("mark_world_mm"), ctx.get("cam_world_mm")
            if not mw or not cw:
                raise CalibError("世界坐标反算失败（_context 子项为空）")
            wx, wy = mw[0], mw[1]
            dx, dy = mw[0] - cw[0], mw[1] - cw[1]
            if _it == 0 and math.hypot(dx, dy) > 0.02:
                # 相机随对准台动：新位=当前 AS 位+δ
                svc._move_and_wait("AS_X", svc._axis_pos("AS_X") + dx)
                svc._move_and_wait("AS_Y", svc._axis_pos("AS_Y") + dy)
                _sleep_ms(wait_ms)
                continue
            break
        # 共焦 Z：AS 走位共焦偏移（探头=相机+(dx,dy)，随台动）把探头
        # 落到标记点：AS_new = AS_now − (conf_dx, conf_dy)（探头世界=
        # 相机+偏移，偏移随台动，符号推演见模块头）
        svc._move_and_wait("AS_X", svc._axis_pos("AS_X") - conf_dx)
        svc._move_and_wait("AS_Y", svc._axis_pos("AS_Y") - conf_dy)
        _sleep_ms(wait_ms)
        r = eng.confocal.read("up")
        if not r.get("ok") or r.get("reading_um") is None \
                or r.get("valid") is False:
            raise CalibError(
                "共焦读数无效（probe=up 量程外——标记 %s 处 die 面不在 "
                "700µm 量程内，检查 BS_Arm_Z 焦面位/片厚）" % mark_id)
        reading = float(r["reading_um"])
        wz = float(r["z_focus"]) - (reading - conf_mid_up) / 1000.0
        with eng.lock:
            asx = eng.axes["AS_X"].pos
            asy = eng.axes["AS_Y"].pos
            asz = eng.axes["AS_Z"].pos
            bsz = eng.axes["BS_Arm_Z"].pos
            brx = eng.axes["BS_CAL_Rx"].pos
            bry = eng.axes["BS_CAL_Ry"].pos
        rows.append([axis_tag, phase_tag, mark_id, rep,
                     "%.4f" % col, "%.4f" % row,
                     "%.6f" % asx, "%.6f" % asy, "%.6f" % asz,
                     "%.6f" % bsz, "%.8f" % brx, "%.8f" % bry,
                     "%.6f" % wx, "%.6f" % wy, "%.6f" % wz,
                     "%.4f" % reading,
                     "%.6f" % cw[0], "%.6f" % cw[1]])
        _log(svc, job, "  %s/%s/mark%d rep%d: world=(%.5f,%.5f,%.5f) "
             "conf=%.2fum"
             % (axis_tag, phase_tag, mark_id, rep, wx, wy, wz, reading))
        return (wx, wy, wz)

    try:
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        # 前置位姿（照 _pose_for_head_shot 口径；BS_CAL_Rx/Ry→0）
        svc._move_and_wait("AS_Y", 6.0)
        svc._move_and_wait("AS_Z", 0.0)
        svc._move_and_wait("BS_Arm_Z", -118.33)
        svc._move_and_wait("BS_CAL_Rx", 0.0)
        svc._move_and_wait("BS_CAL_Ry", 0.0)
        _sleep_ms(wait_ms)
        # 视野预检在首个角标位做（头心位 (−42.5,7.0) 距四角 ±3.8mm，超出
        # up 相机 1.8mm FOV——在头心预检必误拒，先走到角标 0 名义位再查）
        m0 = marks_nom[0]
        svc._move_and_wait("AS_X", m0[1] - _CAM_ZERO_XY[0])
        svc._move_and_wait("AS_Y", m0[2] - _CAM_ZERO_XY[1])
        _sleep_ms(wait_ms)
        _check_marks(svc, "up", cam)          # 走位到位后视野预检
        done = 0
        # 测量位 BS_Arm_Z（轴点 z 参考系修正用：链条枢轴随 BS_Arm_Z 联动）
        bs_z_meas = float(svc._axis_pos("BS_Arm_Z"))
        for axis_name, axis_tag in (("BS_CAL_Rx", "RX"), ("BS_CAL_Ry", "RY")):
            for phase_val, phase_tag in ((-a, "neg"), (0.0, "zero"),
                                         (a, "pos")):
                _check_cancel(job, done + 1)
                svc._move_and_wait(axis_name, phase_val)
                _sleep_ms(wait_ms)
                acc = []
                for mi, mx, my in marks_nom:
                    _check_cancel(job, done + 1)
                    # 每点 repeat 次取均值（相机 XY + 共焦 Z 各 repeat 次）
                    xs, ys, zs = [], [], []
                    for rep in range(repeat):
                        _progress(job, done + 1, total,
                                  "%s %s 标记 %d（%d/%d）"
                                  % (axis_tag, phase_tag, mi, rep + 1,
                                     repeat))
                        x, y, z = _measure_point(mi, mx, my, phase_tag,
                                                 axis_tag, rep)
                        xs.append(x)
                        ys.append(y)
                        zs.append(z)
                    acc.append((_mean_std(xs)[0], _mean_std(ys)[0],
                                _mean_std(zs)[0]))
                    done += 1
                pts[axis_tag][phase_tag] = acc
            svc._move_and_wait(axis_name, 0.0)   # 每轴测完回零再测下一轴
        # ---- SVD 拟合（phase=−a ↔ phase=+a 两组 4 点）----
        fits = {}
        for axis_tag, sign_key in (("RX", 0), ("RY", 1)):
            P = pts[axis_tag]["neg"]
            Q = pts[axis_tag]["pos"]
            _pt_unused, direction, angle, rmse = _estimate_rotation_axis_svd(
                P, Q)
            direction = [float(v) for v in direction]
            # 方向符号归一化：Rx 的 x 分量强制为正、Ry 的 y 分量强制为正
            if direction[sign_key] < 0:
                direction = [-v for v in direction]
            # 轴上点：lstsq(I−R,t) 在小转角下病态（det(I−R)~θ²，测量噪声
            # 按 ~1/θ² 放大成天文数字垃圾）——改走**垂直于拟合方向的平面
            # 内 2D 闭式解**（同 extra_routines._fit_rot 口径）：点集投影
            # 到 ⊥k 平面（基 u,v），c=(I−R2)⁻¹(q̄−R2·p̄)（det=2(1−cosθ)，
            # 中心噪声 ~σ/θ，良性）；沿轴分量不可观（秩亏），取测量重心
            # 在 k 上的投影作锚（保持与默认常数（物理球座心 x=−42.5）
            # 同量级语义）
            np = _np()
            kn = np.asarray(direction, float)
            Pn = np.asarray(P, float)
            Qn = np.asarray(Q, float)
            cP = Pn.mean(axis=0)
            e = np.eye(3)[int(np.argmin(np.abs(kn)))]
            u = np.cross(kn, e)
            u = u / np.linalg.norm(u)
            v = np.cross(kn, u)
            p2 = np.stack([Pn @ u, Pn @ v], axis=1)
            q2 = np.stack([Qn @ u, Qn @ v], axis=1)
            pb = p2.mean(axis=0)
            qb = q2.mean(axis=0)
            cross = dot = 0.0
            for pi, qi in zip(p2, q2):
                ux, uy = pi[0] - pb[0], pi[1] - pb[1]
                vx, vy = qi[0] - qb[0], qi[1] - qb[1]
                cross += ux * vy - uy * vx
                dot += ux * vx + uy * vy
            th2 = math.atan2(cross, dot) if abs(dot) > 1e-12 else 0.0
            c_, s_ = math.cos(th2), math.sin(th2)
            m11 = m22 = 1.0 - c_
            m12, m21 = s_, -s_
            det = m11 * m22 - m12 * m21
            if det < 1e-18:
                raise CalibError(
                    "角度档退化（angle_mrad 过小？）：旋转中心解算 (I−R(θ)) "
                    "矩阵奇异（det=%.3e）" % det)
            rx_ = qb[0] - (c_ * pb[0] - s_ * pb[1])
            ry_ = qb[1] - (s_ * pb[0] + c_ * pb[1])
            cu = (rx_ * m22 - ry_ * m12) / det
            cv = (m11 * ry_ - m21 * rx_) / det
            point = kn * float(np.dot(cP, kn)) + cu * u + cv * v
            point = [float(v_) for v_ in point]
            # 参考系修正：链条 R 枢轴随 BS_Arm_Z 联动（error_kinematics
            # rodrigues 的 center 按 T 链元平移）——实测的是"当前测量位
            # 的有效轴线"，世界 z = 常数语义 z + BS_Arm_Z；常数定义在
            # BS_Arm_Z=0 系，报告值 z 减回测量位 bs_z_meas
            point[2] -= bs_z_meas
            # 逐标记残差（R·P+t vs Q 的范数，mm；R/t 复用 SVD 结果重算）
            cQ = Qn.mean(axis=0)
            H = (Pn - cP).T @ (Qn - cQ)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt.T[:, 2] *= -1
                R = Vt.T @ U.T
            t = cQ - R @ cP
            resid = [float(np.linalg.norm(R @ Pn[i] + t - Qn[i]))
                     for i in range(len(P))]
            fits[axis_tag] = {"point": [float(v) for v in point],
                              "dir": direction, "angle_rad": float(angle),
                              "rmse_mm": rmse, "resid_mm": resid}
            _log(svc, job, "%s: dir=(%.6f,%.6f,%.6f) point=(%.4f,%.4f,%.4f) "
                 "rmse=%.2fum angle=%.6frad"
                 % (axis_tag, direction[0], direction[1], direction[2],
                    fits[axis_tag]["point"][0], fits[axis_tag]["point"][1],
                    fits[axis_tag]["point"][2], rmse * 1000.0, angle))
    finally:
        for ax in restore_axes:
            try:
                svc._move_and_wait(ax, saved[ax])
            except Exception:
                pass
        try:
            svc.vision._set_light(rt, old_light)
        except Exception:
            pass
    fname = _csv_write(
        "bs_rxry_axis_nodes",
        ["axis", "phase", "mark_id", "rep", "col", "row",
         "AS_X_mm", "AS_Y_mm", "AS_Z_mm", "BS_Arm_Z_mm",
         "BS_CAL_Rx_rad", "BS_CAL_Ry_rad",
         "mark_world_x_mm", "mark_world_y_mm", "mark_world_z_mm",
         "confocal_reading_um", "cam_world_x_mm", "cam_world_y_mm"], rows)
    _log(svc, job, "data saved: %s" % fname)
    # summary CSV：轴点/方向/rmse/angle/各标记残差
    srows = []
    for axis_tag in ("RX", "RY"):
        f = fits[axis_tag]
        srows.append([axis_tag, "axis_point_mm",
                      "%.6f" % f["point"][0], "%.6f" % f["point"][1],
                      "%.6f" % f["point"][2], "", ""])
        srows.append([axis_tag, "axis_dir",
                      "%.8f" % f["dir"][0], "%.8f" % f["dir"][1],
                      "%.8f" % f["dir"][2], "", ""])
        srows.append([axis_tag, "angle_rad", "%.8f" % f["angle_rad"],
                      "", "", "", ""])
        srows.append([axis_tag, "rmse_mm", "%.6f" % f["rmse_mm"],
                      "", "", "", ""])
        for i, rv in enumerate(f["resid_mm"]):
            srows.append([axis_tag, "mark%d_resid_mm" % i,
                          "%.6f" % rv, "", "", "", ""])
    srows.append(["meta", "angle_mrad", "%.4f" % float(angle_mrad),
                  "", "", "", ""])
    srows.append(["meta", "repeat", "%d" % repeat, "", "", "", ""])
    fname2 = _csv_write(
        "bs_rxry_axis_summary",
        ["axis", "item", "v0", "v1", "v2", "_r0", "_r1"], srows)
    _log(svc, job, "data saved: %s" % fname2)
    cur = {}
    try:
        cur = dict((eng.constants.get() or {}).get("bs_rxry_axis") or {})
    except Exception:
        cur = {}
    cur.pop("_note", None)
    return {
        "routine": "bs_rxry_axis", "camera": "up",
        "template_name": template_name,
        "repeat": repeat, "wait_ms": wait_ms,
        "measured": {
            "rx_axis_point_mm": [round(v, 6) for v in fits["RX"]["point"]],
            "rx_axis_dir": [round(v, 8) for v in fits["RX"]["dir"]],
            "ry_axis_point_mm": [round(v, 6) for v in fits["RY"]["point"]],
            "ry_axis_dir": [round(v, 8) for v in fits["RY"]["dir"]],
            "rx_rmse_mm": round(fits["RX"]["rmse_mm"], 6),
            "ry_rmse_mm": round(fits["RY"]["rmse_mm"], 6),
            "angle_mrad": float(angle_mrad)},
        "current": {k: (list(v) if isinstance(v, (list, tuple)) else v)
                    for k, v in cur.items()},
        "points": {tag: {ph: [[round(c, 6) for c in p] for p in pts[tag][ph]]
                         for ph in ("neg", "zero", "pos")}
                   for tag in ("RX", "RY")},
        "series": None,
        "stats": {"rx_rmse_mm": round(fits["RX"]["rmse_mm"], 6),
                  "ry_rmse_mm": round(fits["RY"]["rmse_mm"], 6)},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "csv_summary": "runtime/metrology_data/" + fname2,
        "detail": {"angle_mrad": float(angle_mrad),
                   "n_marks": n_mark,
                   "confocal_offset_mm": [conf_dx, conf_dy],
                   "confocal_mid_up_um": conf_mid_up,
                   "note": "SVD 刚体拟合（phase=−a↔+a 各 4 角标）：dir="
                           "rotvec_from_R 轴方向（Rx 的 x 分量强制为正、"
                           "Ry 的 y 分量强制为正），point=⊥dir 平面内 2D "
                           "闭式旋转中心+沿轴分量取测量重心投影（lstsq"
                           "(I−R,t) 在小转角下病态弃用）；Z 由 up 共焦测"
                           "（z_focus−(reading−mid)/1000），XY 由上相机 "
                           "solve_pixel（含 dir/swap/rot_mrad 认知）；"
                           "参考 rxry_axis_sim.py"},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
