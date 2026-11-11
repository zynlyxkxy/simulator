# -*- coding: utf-8 -*-
"""metrology/as_head_center.py — 上吸盘位置标定（迭代 42，routine
as_head_center，cam 恒 up）。

测"上相机认键合头吸盘（collet）中心时的对准台轴位"——机器常数
focus_level.as_head_center_xy_mm（fl die 空间寻址基准，flow_engine
消费）；wafer_ega.as_anchor_xy_mm（对准台锚点：WS 处于 EGA 位时上/下
相机光轴对准吸盘心）与之为**同一物理量**（上/下相机同轴安装，offset
仅 z 不同——align_stage 工具点 (-42.5,23.5,910)，up offset [0,-22.5,
+15]、down offset [0,-22.5,-14.1]，XY 同为 (-42.5,1.0)@AS 零位），故
measured 双键同值联动写回（apply 组路由 as_anchor_xy_mm→wafer_ega /
as_head_center_xy_mm→focus_level）。

算法（动态再逼近直接测量法）：
- 前置走位：AS_X/AS_Y→anchor0（wafer_ega.as_anchor_xy_mm 现值）、
  AS_Z→focus_level.as_z_chuck_focus_mm（上相机焦面基准位）、
  BS_Arm_Z→focus_level.head_z_upfocus_mm（裸吸盘面进上相机焦面）——
  到位后视野预检 _check_marks（迭代 42：校验在走位后做，start 校验器只
  查模板/repeat——start 时机台未必在测量位，当前位预检会误拒）；
- 每 repeat：动态再逼近（AS_X/AS_Y 先 +1mm 偏移、回 anchor0——单侧
  逼近消反向间隙，与 stage_nonortho 升序跳测同哲学）→ identify(up,
  template) 取判真点云质心 (col,row) → _context 链 solve_pixel 反算
  （含 dir/swap/rot_mrad 控制器认知）得 mark_world 与 cam_world；
- 世界偏移 δ=mark_world−cam_world（画幅中心即相机光轴）；**δ 符号约
  定**：上相机挂对准台随台动（AS 走 +δ 视场中心也走 +δ），故把相机
  光轴送到标记上的新锚点 = 当前 AS 位 + δ（与 JOG_AXES up 组 rel=−1
  的"标记相对相机"口径互逆——那里动相机 − 等效标记 +，这里求"相机要
  走多远才盖住标记"，直接 +δ）；
- anchor_i=(AS_X,AS_Y)+δ；repeat 轮取均值。**写回自证**：apply 写回后
  复测时 AS 已在新锚点、δ≈0 → 复测 measured≈已写回值（残差≈0，符号
  错误的偏差会被放大而不是吸收，测试 i42.as_head_center 验证）。

真值口径：误差链 OFF 时 bond_head 吸盘心世界=(−42.5,7.0)、上相机零位
世界=(−42.5,1.0) ⇒ 真值锚点=(0,6) 与现常数一致；误差链 ON 时真值含
align_stage/bond_head 两模块 6DOF 偏差（~µm~10µm 级，实测 (~+0.006,
5.984)），测得的是"含装配真值的实际锚点"——真机测校同此口径。

模板：params.template_name 给了须已注册（不自建不删）；缺省=机器常数
vision_marks.tpl_head_chuck（键合头吸盘标记模板，本测校预留，默认
"tpl_head_chuck"）。

视觉模式：FAST/REAL 均可（作业不切模式——score 两模式单调可测且真机
可得；FAST 下百点级扫描/重复快两个量级）。光源自管（作业开灯、finally
恢复原灯态）；轴位（AS_X/AS_Y/AS_Z/BS_Arm_Z）作业前记录、finally 恢复
（先抬 BS_Arm_Z 再回 AS，防撞）。cancel 检查点每循环首。

结果契约（标定程序形态，同 rotate_center）：
measured={as_anchor_xy_mm:[x,y], as_head_center_xy_mm:[x,y]}（同值联动）
+ current 同两键现值；stats.anchor={x:{mean_mm,s3_nm},y:{…},s3_nm:
hypot(分量 3σ)}；series.anchor_x/y_mm（逐轮锚点）+ delta_x/y_um（逐轮
世界偏移）；points 逐轮明细；CSV as_head_center_MMDD_HH_MM_SS.csv（落
runtime/metrology_data/）。"""
import math
import os
import sys
import time

# 同 rotate_center.py 的导入定位方式（包内相对导入兜底）
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

# 动态再逼近偏移量（mm，XY 两轴同偏；+偏移后回 anchor=单侧逼近）
_JOG_MM = 1.0


def _const(svc, group, key, dfl):
    """读机器常数（异常回退 dfl）。"""
    try:
        v = (svc.engine.constants.get() or {}).get(group, {}).get(key)
        return v if v is not None else dfl
    except Exception:
        return dfl


def run_as_head_center(svc, template_name, repeat, wait_ms, job=None):
    """上吸盘位置标定主流程。参数校验（模板存在性/repeat/wait_ms）已在
    calibration.start 做完（_validate_as_head_center）；视野预检在本函
    数前置走位到位后做（start 时机台未必在测量位）。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    cam = svc.vision.cams["up"].model
    rt = svc.vision.cams["up"]
    anchor0 = [float(v) for v in
               _const(svc, "wafer_ega", "as_anchor_xy_mm", [0.0, 6.0])]
    as_z_focus = float(_const(svc, "focus_level", "as_z_chuck_focus_mm",
                              -1.0))
    head_z_focus = float(_const(svc, "focus_level", "head_z_upfocus_mm",
                                -119.83))
    old_light = bool(svc.points.get(cam.light_point, False))
    # 作业前轴位（finally 恢复）：先抬 BS_Arm_Z 再回 AS（防撞顺序）
    restore_axes = ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y")
    saved = {}
    with eng.lock:
        for a in restore_axes:
            saved[a] = eng.axes[a].pos
    _log(svc, job, "as head center: template=%s repeat=%d wait_ms=%s "
         "anchor0=(%.4f,%.4f) AS_Z→%.4f BS_Arm_Z→%.4f"
         % (template_name, repeat, wait_ms, anchor0[0], anchor0[1],
            as_z_focus, head_z_focus))

    anchors = []                     # 每轮 (ax, ay)
    deltas = []                      # 每轮 (dx, dy) 世界偏移
    rows = []
    pts = []
    try:
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        # 前置走位：AS_Z→焦面基准 → AS_X/Y→锚点现值 → BS_Arm_Z→上焦面
        # （先 AS 后 BS：头下降时对准台已在锚点，与 fl_ready 同序）
        svc._move_and_wait("AS_Z", as_z_focus)
        svc._move_and_wait("AS_X", anchor0[0])
        svc._move_and_wait("AS_Y", anchor0[1])
        svc._move_and_wait("BS_Arm_Z", head_z_focus)
        _sleep_ms(wait_ms)
        _check_marks(svc, "up", cam)      # 走位到位后视野预检
        for i in range(repeat):
            _check_cancel(job, i + 1)
            _progress(job, i + 1, repeat,
                      "第 %d/%d 次吸盘中心测量" % (i + 1, repeat))
            # 动态再逼近：XY 同偏 +1mm 后回 anchor0（单侧逼近消间隙）
            svc._move_and_wait("AS_X", anchor0[0] + _JOG_MM)
            svc._move_and_wait("AS_Y", anchor0[1] + _JOG_MM)
            svc._move_and_wait("AS_X", anchor0[0])
            svc._move_and_wait("AS_Y", anchor0[1])
            _sleep_ms(wait_ms)
            res = svc._identify("up", template_name)["result"]
            col, row = svc._centroid(res, cam.score_threshold)
            score = max(res.get("lstScore") or [0.0])
            ctx = _context(svc, "up", cam, col, row)
            mw, cw = ctx.get("mark_world_mm"), ctx.get("cam_world_mm")
            if not mw or not cw:
                raise CalibError("世界坐标反算失败（_context 子项为空）")
            dx, dy = mw[0] - cw[0], mw[1] - cw[1]
            with eng.lock:
                qx, qy = eng.axes["AS_X"].pos, eng.axes["AS_Y"].pos
            ax_i, ay_i = qx + dx, qy + dy
            anchors.append((ax_i, ay_i))
            deltas.append((dx, dy))
            pts.append({"i": i, "col": round(col, 3), "row": round(row, 3),
                        "score": round(score, 4),
                        "as_x": round(qx, 6), "as_y": round(qy, 6),
                        "delta_x_um": round(dx * 1000.0, 3),
                        "delta_y_um": round(dy * 1000.0, 3),
                        "anchor_x_mm": round(ax_i, 6),
                        "anchor_y_mm": round(ay_i, 6)})
            rows.append([i, "%.6f" % qx, "%.6f" % qy,
                         "%.4f" % col, "%.4f" % row, "%.4f" % score,
                         "%.6f" % cw[0], "%.6f" % cw[1],
                         "%.6f" % mw[0], "%.6f" % mw[1],
                         "%.6f" % dx, "%.6f" % dy,
                         "%.6f" % ax_i, "%.6f" % ay_i])
            _log(svc, job, "cycle %d: px=(%.2f,%.2f) δ=(%+.1f,%+.1f)um "
                 "anchor=(%.6f,%.6f)"
                 % (i + 1, col, row, dx * 1000.0, dy * 1000.0, ax_i, ay_i))
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
    fname = _csv_write(
        "as_head_center",
        ["index", "AS_X_mm", "AS_Y_mm", "mark_col_px", "mark_row_px",
         "score", "cam_world_x_mm", "cam_world_y_mm",
         "mark_world_x_mm", "mark_world_y_mm", "delta_x_mm", "delta_y_mm",
         "anchor_x_mm", "anchor_y_mm"], rows)
    _log(svc, job, "data saved: %s" % fname)
    m_x, s_x = _mean_std([a[0] for a in anchors])
    m_y, s_y = _mean_std([a[1] for a in anchors])
    s3x_nm, s3y_nm = 3 * s_x * 1e6, 3 * s_y * 1e6
    cur_anchor = _const(svc, "wafer_ega", "as_anchor_xy_mm", [0.0, 0.0])
    cur_head = _const(svc, "focus_level", "as_head_center_xy_mm", [0.0, 0.0])
    return {
        "routine": "as_head_center", "camera": "up",
        "repeat": repeat, "wait_ms": wait_ms,
        "template_name": template_name,
        # measured 双键同值联动（上/下相机同轴，同一物理量；apply 组路由
        # as_anchor_xy_mm→wafer_ega / as_head_center_xy_mm→focus_level）
        "measured": {"as_anchor_xy_mm": [round(m_x, 6), round(m_y, 6)],
                     "as_head_center_xy_mm": [round(m_x, 6),
                                              round(m_y, 6)]},
        "current": {"as_anchor_xy_mm": [round(float(v), 6)
                                        for v in cur_anchor],
                    "as_head_center_xy_mm": [round(float(v), 6)
                                             for v in cur_head]},
        "points": pts,
        "series": {"anchor_x_mm": [round(a[0], 6) for a in anchors],
                   "anchor_y_mm": [round(a[1], 6) for a in anchors],
                   "delta_x_um": [round(d[0] * 1000.0, 3) for d in deltas],
                   "delta_y_um": [round(d[1] * 1000.0, 3) for d in deltas]},
        "stats": {"anchor": {"s3_nm": round(math.hypot(s3x_nm, s3y_nm), 2),
                             "x": {"mean_mm": round(m_x, 6),
                                   "s3_nm": round(s3x_nm, 2)},
                             "y": {"mean_mm": round(m_y, 6),
                                   "s3_nm": round(s3y_nm, 2)}}},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"anchor0_mm": [round(v, 6) for v in anchor0],
                   "as_z_chuck_focus_mm": as_z_focus,
                   "head_z_upfocus_mm": head_z_focus,
                   "jog_mm": _JOG_MM,
                   "note": "δ=标记世界位−相机世界位（solve_pixel 含 dir/"
                           "swap/rot_mrad 认知）；相机随对准台动：新锚点="
                           "当前 AS 位+δ（AS 走 +δ 视场中心也走 +δ）；写回"
                           "后复测 δ≈0 作符号自证。measured 双键同值联动"
                           "（上/下相机同轴）"},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
