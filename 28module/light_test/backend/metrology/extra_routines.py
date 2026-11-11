# -*- coding: utf-8 -*-
"""metrology/extra_routines.py — 新增测校项核心（2026-09-08，用户需求③）。

设计依据《测量与校准策略草稿》（正主 测校/测校文档/设计草稿）：
1. confocal_repeat（下/上光谱共焦重复性）——D-07 测量系统重复性验证：
   固定机位连读共焦读数统计 µm 3σ（近拍不动，重复读数走 engine.confocal
   同步 read——量程/中点共面/噪声模型见 docs/调焦调平与高度链_V1.md）。
2. ws_move_repeat（硅片台移动重复性）——水平向重复性表"硅片台"行：
   下相机固定看晶圆标记，WS 来回移动到位拍照，统计标记波动（复用
   repeatability.run_dynamic jog_group=wafer，独立 routine 名）。
3. as_move_repeat（对准台移动重复性）——水平向重复性表"对准台移动
   重复性"行：键合臂固定，AS 来回移动，统计上相机对芯片 Mark 的波动
   （复用 run_dynamic，cam=up 默认即 AS 组）。
4. die_transfer_repeat（芯片交接重复性）——"芯片交接重复性"行（蓝膜台
   →顶针→Flip→键合臂 分段偏差叠加）：作业首 flip_pick 取一颗 die
   （粗对准拍=flip 首段；迭代 43 起 flip 有余片则沿用、跳过取片）后
   die 在 翻转↔键合头 间往复，每循环 flip_handoff（上相机拍=head
   段）→head_to_flip（粗对准拍=back 段，迭代 34 起用 back_tpl——
   缺省=coarse_tpl，不自建临时模板）；循环内不放回蓝膜，迭代 43 起
   作业结束 die 留在拾取头（flip）不再归位（下次作业沿用该片），
   分段统计世界坐标散布。
5. ws_rotate_repeat（硅片台旋转重复性）——**示教参数化直接测角法·双
   示教位**（迭代 35 改造；迭代 34 起移除附件 4.2 远点对基线角全自动路
   径）：taught_points 2~10 点（迭代 29b 旧四键映射为 2 点走同一路径，
   各点 tpl 必须已注册——不自建不删；calibration.start 无 taught_
   points 即 400）。每元素 {tpl, pose, pose_after 可选}——pose_after
   缺省=pose（同位复测）。迭代 35 用户口径测量流（每循环）：
   ① 基线（首循环）逐点走 pose → identify → 世界坐标 P_k（_taught_
   snap 链）；② WS_Rz 走 rz0+rot_rad；③ 逐点走 pose_after（用户直接给
   旋后可见位；下发前 _walk_limit_guard 软限位守卫）→ 实测 Q_k；
   ④ 回 rz0。θ_i 按 _fit_rot 闭式（u=P−P̄、v=Q−Q̄ → θ=atan2(Σu×v,
   Σu·v)，世界系向量集）→ e_i=wrap(θ_i−rot_rad)；stats.angle_err{
   mean_urad,std_urad,s3_urad}（mean=系统角偏差、3σ=**旋转角重复性**）
   + series.angle_err_urad；各点位置残余=拟合残余 3σ(nm)。result 兼容：
   stats.dtheta 填 e_i 统计、stats.residual/series.points[i].d_*/
   series.dtheta_urad 保留（防旧客户端渲染崩）。迭代 35 起**删除预测走
   位 Δ 段**（原绕认知枢轴 Δ=(I−R)(P_k−c) 预估+start 预估 400，由用户
   pose_after 取代；CSV 删 p{k}_pred_d* 列）。rot_rad（迭代 31 起 rad
   全线统一，旧 rot_deg 在 calibration.start 换算走同一校验）。
6. ws_rotate_center（硅片台旋转中心标定，迭代 32，metrology/
   rotate_center.py；迭代 35 改双示教位+_fit_rot）：示教点基线 P_k（走
   pose），每 repeat 逐角度档 θ_j（±step_rad×1..n_angles 对称，WS_Rz 走
   rz0+θ_j）逐点走 pose_after 实测 Q_kj → 每 (循环,档) _fit_rot 闭式
   (θ_ij,C_ij)（θ=atan2(Σu×v,Σu·v)、C=(I−R(θ))⁻¹(Q̄−R(θ)P̄) 2×2 直接
   解）→ measured.rz_center_offset_mm（中心=分量均值−WS_Rz 机械轴心（运
   动链名义不动点，与真值注入 sim_align.rz_center_true_offset_um 同口
   径），3σ=hypot(分量散布)）+ **rz_center_ref_xy_mm=[作业起点
   WS_LS_X,WS_LS_Y]**（标定时机台位）→ apply 双写回两键（组路由
   fine_align）。全程示教模板（宜小标记 site quad——大 cross/就地默认
   ROI 模板旋转态 NCC 偏差经 1/θ 放大别名进拟合，实测不可用）。
7. ws_rotate_gain（硅片台旋转角度增益标定，迭代 32A，metrology/
   rotate_gain.py；迭代 35 同改双示教位+_fit_rot）：每 (循环,档) 实测
   θ_ij（_fit_rot）对指令 θ_cmd_j **过原点 LS** gain=Σθaθc/Σθc²
   （per-repeat；截距=固定角偏差诊断 offset_urad，带截距斜率诊断保留）；
   measured.rz_gain（无量纲~1.0）+ gain 3σ，apply 写回 fine_align.rz_gain。
8. camera_microrot（迭代 32 起执行链自 metrology/microrot.py 迁入本
   模块——**走位轴改对准台**，轴映射事实源=calibration.MICROROT_AXES
   按 (cam, move_stage) 查表）：down+"align"（默认）→AS_X/AS_Y rel=−1
   （下相机与上相机同挂对准台，动相机=等效动标记，标记相对相机反向
   移动）、down+"wafer"→WS_LS_X/Y rel=+1（动标记载体）、up 恒 align
   →AS_X/Y、coarse 恒 FA_X（不参与 move_stage）；模板/拍照位必填
   （迭代 34，无自建/无缺省路径）；_best_peak 复用 microrot.py 懒导入。

与 repeatability.py 同约定：svc=calibration service（svc._identify/
_centroid/_move_and_wait/engine/flow/vision），CSV 落 runtime/
metrology_data/，异常抛给 calibration._run 收 error 态。
"""
import csv
import math
import os
import sys
import time

# 同 repeatability.py 的导入定位方式（metrology 包内相对导入兜底：
# calibration.py 以 `from metrology import extra_routines` 引入时包已
# 在 sys.path；本文件被直接运行/测试时补包目录）
try:
    from repeatability import (DATA_DIR, _angle_deg, _check_cancel,
                               _check_marks, _context, _log, _mean_std,
                               _progress, _sleep_ms, nm_per_px)
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (DATA_DIR, _angle_deg, _check_cancel,
                               _check_marks, _context, _log, _mean_std,
                               _progress, _sleep_ms, nm_per_px)

CONF_COLS = ["index", "reading_um", "valid", "z_surface_mm", "z_focus_mm"]


def _csv_write(prefix, header, rows):
    os.makedirs(DATA_DIR, exist_ok=True)
    fname = "%s_%s.csv" % (prefix, time.strftime("%m%d_%H_%M_%S"))
    base, ext = os.path.splitext(fname)
    k = 1
    while os.path.exists(os.path.join(DATA_DIR, fname)):
        k += 1
        fname = "%s_%d%s" % (base, k, ext)
    with open(os.path.join(DATA_DIR, fname), "w", newline="",
              encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return fname


def _s3(vals):
    _m, s = _mean_std(vals)
    return 3.0 * s


# =====================================================================
# 1/2. 光谱共焦重复性（probe = "down" / "up"）
# =====================================================================
def run_confocal_repeat(svc, probe, repeat, wait_ms, job=None):
    """固定机位连读共焦读数：reading_um 序列统计 3σ（µm）。
    读数无效（量程外/无被测目标）按测校失败处理（error 态）——
    真实测校中量程外意味着机位/被测面异常，静默剔除会美化指标。"""
    t0 = time.time()
    if svc.engine.confocal is None:
        raise RuntimeError("confocal not attached")
    _log(svc, job, "confocal repeatability: probe=%s repeat=%d wait_ms=%s"
         % (probe, repeat, wait_ms))
    reads, valids, zs, zfs = [], [], [], []
    for i in range(repeat):
        _check_cancel(job, i + 1)
        _progress(job, i + 1, repeat, "第 %d/%d 次读数" % (i + 1, repeat))
        _sleep_ms(wait_ms)
        r = svc.engine.confocal.read(probe)
        if not r.get("ok"):
            raise RuntimeError("共焦读数失败：%s" % (r.get("error") or r))
        if r.get("reading_um") is None or r.get("valid") is False:
            raise RuntimeError(
                "共焦读数无效（probe=%s 量程外——down 需晶圆在位；up 把"
                "BS_Arm_Z 调到约 -118.8mm（无 die 读吸盘面）或用芯片"
                "交接后再测）" % probe)
        reads.append(float(r["reading_um"]))
        valids.append(bool(r.get("valid")))
        zs.append(r.get("z_surface"))
        zfs.append(r.get("z_focus"))
        _log(svc, job, "read: %.3f um" % reads[-1])
        _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    rows = [[i, "%.4f" % reads[i], int(valids[i]),
             "" if zs[i] is None else "%.6f" % zs[i],
             "" if zfs[i] is None else "%.6f" % zfs[i]]
            for i in range(len(reads))]
    fname = _csv_write("confocal_repeat_%s" % probe, CONF_COLS, rows)
    _log(svc, job, "data saved: %s" % fname)
    m_v, s_v = _mean_std(reads)
    return {
        "routine": "confocal_repeat", "camera": probe, "probe": probe,
        "repeat": repeat, "wait_ms": wait_ms,
        "series": {"reading_um": [round(v, 4) for v in reads]},
        "stats": {"reading": {"mean_um": round(m_v, 4),
                              "std_um": round(s_v, 4),
                              "s3_um": round(3 * s_v, 4)}},
        "context": None,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {}, "elapsed_s": round(time.time() - t0, 3),
    }


# =====================================================================
# 3/4. 台移动重复性（复用 run_dynamic，独立 routine 名）
# =====================================================================
def run_ws_move_repeat(svc, tpl, repeat, wait_ms, start_xy, photo_xy,
                       axes_touched, job=None, cam="down",
                       jog_axes_override=None):
    """硅片台移动重复性：下相机固定看晶圆标记，WS_LS_X/Y 往复。
    cam 恒 "down"（ROUTINE_CAM 限制，形参仅为 _run 统一透传）；
    jog_axes_override 迭代 34 起由 calibration._run 透传（validation
    提取的 jog_axes，如 down+align 组）。"""
    from repeatability import run_dynamic
    return run_dynamic(svc, cam, tpl, repeat, wait_ms, start_xy,
                       photo_xy, axes_touched, job=job,
                       routine="ws_move_repeat",
                       jog_axes_override=jog_axes_override)


def run_as_move_repeat(svc, tpl, repeat, wait_ms, start_xy, photo_xy,
                       axes_touched, job=None, cam="up",
                       jog_axes_override=None):
    """对准台移动重复性：键合臂（键合头芯片）固定，AS_X/Y 往复，
    上相机统计芯片 Mark 波动。cam 默认 "up"；迭代 34 起 ROUTINE_CAM
    允许 "down"（下相机看对准台动，须 jog_group=align 使走位轴=AS_X/Y
    ——metrology_gui StageMoveRepeat 契约），cam/jog_axes_override 均
    由 calibration._run 透传 run_dynamic。"""
    from repeatability import run_dynamic
    return run_dynamic(svc, cam, tpl, repeat, wait_ms, start_xy,
                       photo_xy, axes_touched, job=job,
                       routine="as_move_repeat",
                       jog_axes_override=jog_axes_override)


# =====================================================================
# 5. 芯片交接重复性
# =====================================================================
def _run_flow(svc, builder, *args, timeout=300.0):
    """异步启动一段工艺 flow 并轮询到终态（run_steps 启动即返回
    {"flow","steps","paused"}，执行在 flow 线程）。失败/中止抛错。"""
    eng = svc.engine
    if eng.flow is None:
        raise RuntimeError("flow engine not attached")
    import process_flows
    name, steps = builder(eng, *args)
    eng.flow.run_steps(steps, name)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = eng.flow.state
        if st == "done":
            return name
        if st in ("aborted", "error"):
            reason = eng.flow.abort_reason or ""
            raise RuntimeError("工艺段 %s 失败（state=%s %s）"
                               % (name, st, reason))
        time.sleep(0.02)
    raise RuntimeError("工艺段 %s 超时（%.0fs）" % (name, timeout))


def _shot_world(svc, cam_id, tpl):
    """拍照 → (col,row) + 世界坐标（质心反算）。"""
    cam = svc.vision.cams[cam_id].model
    entry = svc._identify(cam_id, tpl)
    res = entry.get("result") or {}
    col, row = svc._centroid(res, cam.score_threshold)
    c = _context(svc, cam_id, cam, col, row)
    return col, row, c


def run_die_transfer_repeat(svc, repeat, wait_ms, job=None,
                            coarse_tpl=None, up_tpl=None, back_tpl=None,
                            measure_back=None):
    """芯片交接重复性（设计草稿"蓝膜台→顶针→Flip→键合臂 分段测量"）：
    首段 flip_pick（蓝膜→翻转，整个作业仅取一颗 die，粗对准拍 die 中心
    =flip 段）后，die 在 翻转↔键合头 之间往复，每循环：
      flip_handoff（翻转→键合头，上相机拍头下芯片 Mark=head 段）
      → head_to_flip（回翻转，粗对准拍=back 段）
    三段世界坐标分别统计 3σ（µm）——段间差异即交接链位置偏差叠加。

    模板三件套（迭代 34：start 已做非空/存在性 400 并规范化，此处保留
    运行期兜底）：coarse_tpl（粗对准相机 die 标记，缺省=机器常数
    vision_marks.tpl_coarse_die 或 cv_chipback）、up_tpl（上相机头下
    die 标记，缺省=tpl_die_head）、back_tpl（回翻段粗对准标记，可选，
    缺省=coarse_tpl 的值）——**不自建临时模板**（原 _dt_back_{ms} 就地
    自建/自删路径已移除）。

    物料口径（迭代 43，用户明确要求"结束留片在拾取头"）：die 全程持于
    交接链，作业结束/异常/取消均**不再归位蓝膜**——循环结束 die 本就
    在 flip，finally 只按实际 carrier 打一行留片日志（轴位/光源/视觉
    模式照旧恢复）。起始兼容（作业首守卫）：flip+head 双残留 →
    CalibError 请先在工艺流程页清片；head 单残留 → _reclaim_head
    回收 head→flip（不再回放蓝膜，回收后沿用该片）；flip 有余片
    （含回收所得）→ 沿用、跳过首次 flip_pick 取片（首段 flip 拍照
    照常）。measure_back 参数已废止（新语义天然含 back 段）：保留
    入参位仅兼容旧调用（calibration._run 仍透传），传入一律忽略。

    作业开始存 AS_X/AS_Y/AS_Z/BS_Arm_Z/FA_X/ES_X/ES_Y 轴位，finally
    恢复（迭代 29b）。现场（real 模式/两台相机光源）由本函数自管
    （finally 恢复）。"""
    t0 = time.time()
    eng = svc.engine
    from calibration import CalibError   # 懒导入避免模块环（同 _fit_rot）
    wp = eng.workpieces
    if wp is None or not wp.film["present"]:
        raise RuntimeError("蓝膜不在位（先上膜：POST /api/workpiece/film）")
    try:
        vmarks = (eng.constants.get() or {}).get("vision_marks") or {}
    except Exception:
        vmarks = {}
    coarse_tpl = coarse_tpl or vmarks.get("tpl_coarse_die") or "cv_chipback"
    up_tpl = up_tpl or vmarks.get("tpl_die_head") or "tpl_die_head"
    back_tpl = back_tpl or coarse_tpl
    for tpl, cam_id in ((coarse_tpl, "coarse"), (up_tpl, "up"),
                        (back_tpl, "coarse")):
        if svc.vision.templates.get(tpl) is None:
            raise RuntimeError("模板 %r 不存在（先在视觉页建立）" % tpl)
    old_mode = svc.vision.mode
    lights = {cid: bool(svc.points.get(
        svc.vision.cams[cid].model.light_point, False))
        for cid in ("coarse", "up")}
    # 作业前轴位（finally 恢复）：交接链走位涉及的所有轴（迭代 29b）
    AXES_SAVE = ("AS_X", "AS_Y", "AS_Z", "BS_Arm_Z", "FA_X", "ES_X", "ES_Y")
    saved_axes = {}
    with eng.lock:
        for _a in AXES_SAVE:
            saved_axes[_a] = eng.axes[_a].pos
    seg_tags = ["flip", "head", "back"]
    _log(svc, job, "die transfer repeatability: repeat=%d coarse_tpl=%s "
         "up_tpl=%s back_tpl=%s（循环内不放回蓝膜，结束留片拾取头）"
         % (repeat, coarse_tpl, up_tpl, back_tpl))
    segs = {tag: [] for tag in seg_tags}

    def _reclaim_head():
        """作业首/循环首幂等复位：head 有 die（上一作业中断残留）回收
        head→flip（迭代 43 起不再回放蓝膜——回收片留 flip 沿用，蓝膜
        die 数不变；正常路径循环首 die 在 flip、head 空=空操作）。
        flip 已有 die 时 head_to_flip 无法承接 → CalibError（与作业首
        双残留守卫同一文案）。"""
        if any(d.get("carrier") == "head" for d in _dies(eng)):
            if any(d.get("carrier") == "flip" for d in _dies(eng)):
                raise CalibError(
                    "翻转拾取头与键合头均有残留芯片，请先在工艺流程页清片")
            _log(svc, job, "reclaim: head 残留 die 回收"
                 "（head_to_flip，留拾取头沿用）")
            _run_flow(svc, lambda e:
                      __import__("process_flows").build_head_to_flip_flow(e))

    def _snap(tag, cam_id, tpl):
        col, row, c = _shot_world(svc, cam_id, tpl)
        mw = c.get("mark_world_mm") or (None, None, None)
        segs[tag].append((col, row, mw))
        _log(svc, job, "detect[%s]: col=%.2f row=%.2f world=(%.4f,%.4f)"
             % (tag, col, row, mw[0] if mw[0] is not None else float('nan'),
                mw[1] if mw[1] is not None else float('nan')))

    def _pose_for_head_shot():
        """handoff 后键合头 die 标记的观测走位（参考 test_metrology
        prep_up 的后段：AS_Y=6 上相机对键合头吸盘中心、BS_Arm_Z=-118.33
        把 die 底面送进上相机焦面、guide 引导居中最近标记）。"""
        import vision_model as vm
        svc._move_and_wait("AS_Y", 6.0)
        svc._move_and_wait("AS_Z", 0.0)
        svc._move_and_wait("BS_Arm_Z", -118.33)
        g = svc.vision.guide("up")
        ms = g.get("marks") or []
        if not ms:
            raise RuntimeError("上相机视野无 die 标记（head 无芯片？）")
        for s in (ms[0].get("suggested") or []):
            svc._move_and_wait(s["axis"],
                               svc._axis_pos(s["axis"]) + s["delta_mm"])

    def _pose_after_head_back():
        """head_to_flip 回放后粗对准观测走位：FA_Z 回粗对准调焦位
        （fa_z_view_mm——head_to_flip 结束在 Z 安全位 20mm，die 离焦糊成
        均匀亮雾，迭代 34 起 back 段用固定 back_tpl 识别，必须先回
        合焦位，与 coarse_tpl 示教时的景深口径一致）+ FA_X=−5 观测位 +
        guide 引导居中（die 回翻后位置与 pick 后略有差异，guide 精修）。"""
        try:
            fa_z_view = float((eng.constants.get() or {})
                              .get("flip_pick", {})
                              .get("fa_z_view_mm", 0.0))
        except Exception:
            fa_z_view = 0.0
        svc._move_and_wait("FA_Z", fa_z_view)
        svc._move_and_wait("FA_X", -5.0)
        g = svc.vision.guide("coarse")
        ms = g.get("marks") or []
        if ms:
            for s in (ms[0].get("suggested") or []):
                svc._move_and_wait(
                    s["axis"], svc._axis_pos(s["axis"]) + s["delta_mm"])

    try:
        if old_mode != "real":
            svc.vision.set_mode("real")
        for cid in ("coarse", "up"):
            svc.vision._set_light(svc.vision.cams[cid], True)
            svc.vision._await_light(svc.vision.cams[cid], True)
        # 起始兼容守卫（迭代 43）：flip+head 双残留无法自治 → 报错请先
        # 清片；head 单残留 → _reclaim_head 回收 head→flip（回收片沿用）；
        # flip 有余片（含回收所得/上一作业留片）→ 沿用该片跳过取片
        _ds0 = _dies(eng)
        if any(d.get("carrier") == "flip" for d in _ds0) \
                and any(d.get("carrier") == "head" for d in _ds0):
            raise CalibError(
                "翻转拾取头与键合头均有残留芯片，请先在工艺流程页清片")
        _reclaim_head()         # 作业首幂等复位（head 残留回收→flip）
        if any(d.get("carrier") == "flip" for d in _dies(eng)):
            _log(svc, job, "拾取头有余片，沿用该片测校（跳过 flip_pick 取片）")
            # 回收/遗留态 flip 未必在粗对准观测位（回收结束在 Z 安全位）
            # ——与循环内 back 段同口径先回合焦观测位再首段拍照
            _pose_after_head_back()
        else:
            _run_flow(svc, lambda e, d=None:
                      __import__("process_flows").build_flip_pick_flow(e, d))
        _sleep_ms(wait_ms)
        _snap("flip", "coarse", coarse_tpl)     # 首段：取片/沿用后仅此一拍
        for i in range(repeat):
            _check_cancel(job, i + 1)
            _progress(job, i + 1, repeat, "第 %d/%d 次交接循环" % (i + 1, repeat))
            _reclaim_head()     # 循环首幂等复位（保留；正常路径空操作）
            _run_flow(svc, lambda e:
                      __import__("process_flows").build_flip_handoff_flow(e))
            _sleep_ms(wait_ms)
            _pose_for_head_shot()
            _sleep_ms(wait_ms)
            _snap("head", "up", up_tpl)
            _run_flow(svc, lambda e:
                      __import__("process_flows").build_head_to_flip_flow(e))
            _sleep_ms(wait_ms)
            _pose_after_head_back()
            _sleep_ms(wait_ms)
            # 循环内不放回蓝膜：die 留在 flip 上进下一循环；back 段用
            # back_tpl（缺省=coarse_tpl，迭代 34 起不自建临时模板）
            _snap("back", "coarse", back_tpl)
            _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    finally:
        # 迭代 43：die 不归位——循环结束 die 本在 flip（中断场景可能在
        # head），按实际 carrier 打留片日志（不掩盖原始异常）
        try:
            _ds = _dies(eng)
            _on_head = any(d.get("carrier") == "head" for d in _ds)
            _on_flip = any(d.get("carrier") == "flip" for d in _ds)
            if _on_head and _on_flip:
                _log(svc, job, "die 留在拾取头与键合头"
                     "（carrier=flip+head），未归位蓝膜")
            elif _on_head:
                _log(svc, job, "die 留在键合头（carrier=head），"
                     "未归位蓝膜")
            elif _on_flip:
                _log(svc, job, "die 留在拾取头（carrier=flip），"
                     "未归位蓝膜")
        except Exception:
            pass
        # 恢复作业前轴位（迭代 29b）
        for _a, _p0 in saved_axes.items():
            try:
                svc._move_and_wait(_a, _p0)
            except Exception:
                pass
        for cid, was in lights.items():
            try:
                svc.vision._set_light(svc.vision.cams[cid], was)
            except Exception:
                pass
        if old_mode != "real":
            try:
                svc.vision.set_mode(old_mode)
            except Exception:
                pass
    header = ["index"]
    for tag in seg_tags:
        header += ["%s_col" % tag, "%s_row" % tag,
                   "%s_world_x_mm" % tag, "%s_world_y_mm" % tag]
    rows = []
    for i in range(repeat):
        r = [i]
        for tag in seg_tags:
            if i < len(segs[tag]):      # flip 首段仅 1 样本（第 0 行）
                col, row, mw = segs[tag][i]
                r += ["%.4f" % col, "%.4f" % row,
                      "" if mw[0] is None else "%.6f" % mw[0],
                      "" if mw[1] is None else "%.6f" % mw[1]]
            else:
                r += ["", "", "", ""]
        rows.append(r)
    fname = _csv_write("die_transfer_repeat", header, rows)
    _log(svc, job, "data saved: %s" % fname)
    stats = {}
    for tag in seg_tags:
        wx = [s[2][0] for s in segs[tag] if s[2][0] is not None]
        wy = [s[2][1] for s in segs[tag] if s[2][1] is not None]
        cols_ = [s[0] for s in segs[tag]]
        st = {"col_s3_px": round(_s3(cols_), 4)}
        if len(wx) == len(segs[tag]) and wx:
            st["world_x_s3_um"] = round(_s3(wx) * 1000.0, 4)
            st["world_y_s3_um"] = round(_s3(wy) * 1000.0, 4)
        stats[tag] = st
    return {
        "routine": "die_transfer_repeat", "camera": "coarse+up",
        "repeat": repeat, "wait_ms": wait_ms,
        "series": {tag: {"col": [round(s[0], 4) for s in segs[tag]],
                         "row": [round(s[1], 4) for s in segs[tag]]}
                   for tag in seg_tags},
        "stats": stats,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"coarse_tpl": coarse_tpl, "up_tpl": up_tpl,
                   "back_tpl": back_tpl},
        "context": None, "elapsed_s": round(time.time() - t0, 3),
    }


def _dies(eng):
    with eng.workpieces._lock:
        return [dict(d) for d in eng.workpieces.dies]


# =====================================================================
# 旋转类测校共享几何（迭代 32：示教旋转直接测角法 / 旋转中心标定共用）
# =====================================================================
def _rz_pivot_belief(eng, positions=None):
    """认知侧 WS_Rz 旋转枢轴（**ws0 帧**世界 XY，mm）：吸盘心（wafer_stage
    模块中心，与 flow_engine._fa_apply_comp 同口径）+ 机械常数
    fine_align.rz_center_offset_mm（标定值——认知侧补偿枢轴=吸盘心世界
    XY+本偏移，见 machine_constants.json fine_align._note）。
    迭代 35 双示教位改造新增 **rz_center_ref_xy_mm 平移项**：offset 标定
    于 ref_xy 机位（ws_rotate_center measured 双键、apply 双写回），枢轴
    信念按 pivot=吸盘心信念+offset+(当前WS_LS_XY−ref_xy) 平移（当前位取
    本快照的 WS_LS 轴位——各调用方均在作业起点快照，即"当前"=该作业
    机位）。ref=[0,0]（旧数据缺省/未标定参考位）→ 平移项关闭，严格等于
    原 kc+offset 行为（兼容）。仅作预测走位/枢轴参照用。"""
    import vision_model as vm
    if positions is None:
        with eng.lock:
            positions = {n: ax.pos for n, ax in eng.axes.items()}
    poses = eng.kinematics.compute(positions)
    kc, _rot = vm._module_pose(poses, "wafer_stage")
    try:
        fa = (eng.constants.get() or {}).get("fine_align") or {}
    except Exception:
        fa = {}
    off = fa.get("rz_center_offset_mm") or [0.0, 0.0]
    ref = fa.get("rz_center_ref_xy_mm") or [0.0, 0.0]
    px = kc[0] + float(off[0])
    py = kc[1] + float(off[1])
    if float(ref[0]) != 0.0 or float(ref[1]) != 0.0:
        px += float(positions.get("WS_LS_X", 0.0)) - float(ref[0])
        py += float(positions.get("WS_LS_Y", 0.0)) - float(ref[1])
    return (px, py)


def _chuck_center(eng, positions=None):
    """吸盘心模块中心（ws0 帧世界 XY，mm）——wafer_stage 工具点（多孔陶瓷
    面中心）。"""
    import vision_model as vm
    if positions is None:
        with eng.lock:
            positions = {n: ax.pos for n, ax in eng.axes.items()}
    poses = eng.kinematics.compute(positions)
    kc, _rot = vm._module_pose(poses, "wafer_stage")
    return (kc[0], kc[1])


def _rz_axis_center(eng, positions=None):
    """WS_Rz **机械旋转轴心**（ws0 帧世界 XY，mm）：运动链名义正解的不动
    点——工具点名义位置随 WS_Rz 绕本点旋转（error_kinematics R 链元
    rodrigues）。用两个 Rz 位置的名义工具点反解 (I−R(δ))c = o(δ)−R(δ)o(0)
    （仅用 nominal_mm——机械轴心是名义几何，与误差链无关）。
    注意：本点与吸盘心（工具点）存在名义几何差（链配置 Rz center 为
    sandboxes 挖出的占位值，见 error_kinematics.json wafer_stage._note；
    XY 向 ~0.106mm）——ws_rotate_center 的 measured 以本点为基准（真值
    注入 sim_align.rz_center_true_offset_um 也是叠加在本点上的，见
    vision_model.collect_marks 的 pivot 语义）。"""
    if positions is None:
        with eng.lock:
            positions = {n: ax.pos for n, ax in eng.axes.items()}
    p0 = dict(positions)
    p1 = dict(positions)
    d = 0.001
    p1["WS_Rz"] = float(p1.get("WS_Rz", 0.0)) + d
    mods0 = eng.kinematics.compute(p0)["modules"]["wafer_stage"]
    mods1 = eng.kinematics.compute(p1)["modules"]["wafer_stage"]
    o0, o1 = mods0["nominal_mm"], mods1["nominal_mm"]
    c_, s_ = math.cos(d), math.sin(d)
    rx = o1[0] - (c_ * o0[0] - s_ * o0[1])
    ry = o1[1] - (s_ * o0[0] + c_ * o0[1])
    m11, m12, m21, m22 = 1.0 - c_, s_, -s_, 1.0 - c_
    det = m11 * m22 - m12 * m21
    return ((rx * m22 - ry * m12) / det, (m11 * ry - m21 * rx) / det)


def _rot2d(vx, vy, theta):
    """R(θ)·(vx,vy)（世界系逆时针，与 vision_model._rotvec_apply 的 z 分
    量同式）。"""
    c, s = math.cos(theta), math.sin(theta)
    return (c * vx - s * vy, s * vx + c * vy)


def _fit_rot(P_list, Q_list):
    """旋转闭式拟合（迭代 35，find_rotation_center 口径，stdlib 2×2 直接
    解）：u=P−P̄、v=Q−Q̄ → **θ=atan2(Σu×v, Σu·v)**；平移重心差经
    **C=(I−R(θ))⁻¹(Q̄−R(θ)·P̄)** 解旋转中心（(I−R) 行列式=2(1−cosθ)）。
    ws_rotate_repeat（θ_i 与逐点拟合残余）/ ws_rotate_center（每 (循环,
    档) (θ_ij,C_ij)）/ ws_rotate_gain（每 (循环,档) θ_ij）三件套共用，
    替换各自原 Kabsch 段/已知指令角堆叠 LS 的角度来源（统计口径不变）。
    P_list/Q_list：等长（≥2）世界坐标 (x,y) 列表（示教 _taught_snap
    口径）。返回 (theta_rad, (cx, cy))。
    防护沿用现有口径：向量集退化（dot≤1e-12）→ θ=0；小 θ (I−R) 病态
    （det<1e-18，既有 1/θ 放大）→ CalibError（沿用旋转中心文案）。"""
    from calibration import CalibError
    n = len(P_list)
    pbx = sum(p[0] for p in P_list) / n
    pby = sum(p[1] for p in P_list) / n
    qbx = sum(q[0] for q in Q_list) / n
    qby = sum(q[1] for q in Q_list) / n
    cross = dot = 0.0
    for (px, py), (qx, qy) in zip(P_list, Q_list):
        ux, uy = px - pbx, py - pby
        vx, vy = qx - qbx, qy - qby
        cross += ux * vy - uy * vx
        dot += ux * vx + uy * vy
    theta = math.atan2(cross, dot) if dot > 1e-12 else 0.0
    c_, s_ = math.cos(theta), math.sin(theta)
    m11 = m22 = 1.0 - c_
    m12, m21 = s_, -s_
    det = m11 * m22 - m12 * m21
    if det < 1e-18:
        raise CalibError(
            "角度档退化（step_rad/rot_rad 过小？）：旋转中心解算 (I−R(θ)) "
            "矩阵奇异（det=%.3e）——小 θ 经 1/θ 放大病态，增大旋转角"
            % det)
    rx = qbx - (c_ * pbx - s_ * pby)
    ry = qby - (s_ * pbx + c_ * pby)
    cx = (rx * m22 - ry * m12) / det
    cy = (m11 * ry - m21 * rx) / det
    return theta, (cx, cy)


def _taught_go_pose(svc, pose):
    """走到示教位（先 AS 后 WS，与迭代 29b/31 示教路径同款）。迭代 35
    双示教位改造：旋转后走访直接走 pose_after（用户示教旋后可见位），
    不再叠加预测走位 Δ——本函数不再接受 dxy 补偿。"""
    for a in ("AS_X", "AS_Y"):
        svc._move_and_wait(a, pose[a])
    svc._move_and_wait("WS_LS_X", pose["WS_LS_X"])
    svc._move_and_wait("WS_LS_Y", pose["WS_LS_Y"])


def _taught_snap(svc, points, idx, ws0, cam, phase):
    """示教点 idx 拍照 → (col,row,world_ref)；找不到标记给清晰点位/阶段
    提示。world_ref = 质心反算世界坐标 − 相对作业起点 WS 走位量（反算
    坐标≈相机中心，跨点几何由 WS 走位量承担；质心亚像素残差自动保留）。"""
    from calibration import CalibError
    pt = points[idx]
    try:
        res = svc._identify("down", pt["tpl"])["result"]
        col, row = svc._centroid(res, cam.score_threshold)
    except Exception as e:
        raise CalibError(
            "示教点 #%d（tpl=%s）%s找不到标记：%s——该点示教位需保证旋转"
            "后标记仍可见（请按旋转后的可见位重新示教该点）"
            % (idx + 1, pt["tpl"], phase, e))
    mw = _context(svc, "down", cam, col, row).get("mark_world_mm")
    if not mw:
        raise CalibError("示教点 #%d（tpl=%s）%s世界坐标反算失败"
                         % (idx + 1, pt["tpl"], phase))
    with svc.engine.lock:
        dxw = svc.engine.axes["WS_LS_X"].pos - ws0[0]
        dyw = svc.engine.axes["WS_LS_Y"].pos - ws0[1]
    return col, row, (mw[0] - dxw, mw[1] - dyw)


def _walk_limit_guard(svc, idx, tpl, tx, ty, theta):
    """旋转后走位目标软限位守卫（运行期；start 时 pose_after 已逐轴软限位
    400 校验，本守卫为下发前双保险，迭代 35 双示教位改造沿用）。"""
    from calibration import CalibError
    for axis, tgt in (("WS_LS_X", tx), ("WS_LS_Y", ty)):
        with svc.engine.lock:
            ax = svc.engine.axes[axis]
            lo, hi = ax.lo, ax.hi
        if not (lo <= tgt <= hi):
            raise CalibError(
                "示教点 #%d（tpl=%s）旋转 %+.6f rad 的走位 %s=%.4fmm "
                "超出软限位 [%.3f, %.3f]——请检查该点 pose_after 旋后示教位"
                % (idx + 1, tpl, theta, axis, tgt, lo, hi))


# =====================================================================
# 6. 硅片台旋转重复性——示教参数化直接测角法（唯一模式，迭代 34 移除
#    附件 4.2 远点对基线角全自动路径）
# =====================================================================
def run_ws_rotate_repeat(svc, repeat, wait_ms, rot_rad, job=None,
                          tpl=None, taught=None):
    """硅片台旋转重复性——**示教参数化直接测角法为唯一模式**（迭代 34
    起移除全自动路径，calibration._validate_extra 已保证无 taught_
    points → start 400，此处再兜一层清晰错误）。

    taught：非空 dict（{"points":[{tpl,pose,pose_after},...]}，calibration
    ._validate_rotate_taught 已校验——各点 tpl 已注册不自建不删、pose/
    pose_after 四轴示教位、pose_after 缺省=pose），→ _run_ws_rotate_
    taught（结果 detail.mode="taught"，迭代 35 双示教位：基线走 pose/
    旋后走 pose_after）。tpl 形参保留兼容旧调用（示教路径不使用，自动
    路径的 vision_marks 默认模板/tpl_ref 语义随 auto 路径一并移除）。
    rot_rad（迭代 31 起 rad 全线统一，旧 rot_deg 在 calibration.start
    换算走同一校验）。"""
    if not taught:
        raise RuntimeError(
            "ws_rotate_repeat 须给 taught_points（2~10 点示教，每点 tpl "
            "已注册）——全自动模式已于迭代 34 移除")
    return _run_ws_rotate_taught(svc, repeat, wait_ms, rot_rad, job, taught)

# =====================================================================
# 6b. 硅片台旋转重复性——示教参数化模式（迭代 32 直接测角法）
# =====================================================================
def _run_ws_rotate_taught(svc, repeat, wait_ms, rot_rad, job, taught):
    """示教参数化模式——**直接测角法·双示教位**（迭代 35 改造；迭代 32
    用户口径重写；迭代 31 多点化 2~10 点、迭代 29b 旧四键映射为 2 点数组
    走本路径不变）：示教位/模板全部外部给定（各点 tpl 必须已注册——本函数
    不自建不删；pose/pose_after 为 {AS_X,AS_Y,WS_LS_X,WS_LS_Y} 四分量
    示教位，calibration.start 已做缺省补当前位+逐轴软限位 400——pose_after
    缺省=pose 同位复测）。各示教点须使标记在 rz0（作业启动时 WS_Rz 位）
    基线可见、且按 pose_after 示教的旋后位在 rz0+rot_rad 可见。时序
    （每循环）：
      ① 基线（首循环）逐点走 pose → identify → 世界坐标 P_k
        （_taught_snap solve_pixel+WS 走位补偿口径）
      ② WS_Rz 走 rz0+rot_rad
      ③ 逐点走 **pose_after**（用户直接给旋后可见位；下发前
        _walk_limit_guard 软限位守卫双保险）→ 实测世界坐标 Q_k
      ④ WS_Rz 回 rz0 进下一循环
    θ_i 按 _fit_rot 闭式（u=P−P̄、v=Q−Q̄ → θ=atan2(Σu×v,Σu·v)，世界系
    向量集——示教点各自拍摄，像素集退化无跨点几何）→ 角度误差
    e_i=wrap(θ_i−rot_rad)（mean=系统角偏差、3σ=**旋转角重复性**）。
    迭代 35 起**删除预测走位 Δ 段**（原绕认知枢轴 Δ=(I−R)(P_k−c) 预估，
    由用户 pose_after 取代）。各点位置残余=拟合残余（v−R(θ_i)u，世界系
    mm→nm）。结果兼容：stats.dtheta 填 e_i 统计、stats.residual/
    series.points[i].d_*/series.dtheta_urad 保留（防旧客户端渲染崩）；
    CSV 动态列沿用 ws_rotate_taught_* 命名（迭代 35 起删 p{k}_pred_d*
    预测走位列）；detail.taught_points 逐点带 pose_after。
    cancel 检查点每循环首；finally 恢复 AS/WS/WS_Rz 作业前轴位、光源、
    视觉模式（与全自动路径同款）。"""
    t0 = time.time()
    eng = svc.engine
    if eng.workpieces is None or not eng.workpieces.wafer["present"]:
        raise RuntimeError("晶圆不在位（先上片）")
    points = taught["points"]
    n_pt = len(points)
    cam = svc.vision.cams["down"].model
    rt = svc.vision.cams["down"]
    k_nm = nm_per_px(svc, "down")
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
    _log(svc, job, "ws rotate taught(direct, two-pose): n_points=%d "
         "tpls=%s rot=%.6frad repeat=%d"
         % (n_pt, [pt["tpl"] for pt in points], rot_rad, repeat))

    dcols = [[] for _ in range(n_pt)]      # 逐点拟合残余 x（px 换算兼容键）
    drows = [[] for _ in range(n_pt)]
    dcol_nm = [[] for _ in range(n_pt)]
    drow_nm = [[] for _ in range(n_pt)]
    errs = []                              # e_i = θ_i − rot_rad（rad）
    thetas = []                            # θ_i 拟合实际旋转角（rad）
    rows = []
    base = None
    try:
        if old_mode != "real":
            svc.vision.set_mode("real")
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        for i in range(repeat):
            _check_cancel(job, i + 1)
            _progress(job, i + 1, repeat,
                      "第 %d/%d 次示教旋转实测" % (i + 1, repeat))
            if base is None:
                base = []
                for k in range(n_pt):
                    _taught_go_pose(svc, points[k]["pose"])
                    _sleep_ms(wait_ms)
                    base.append(_taught_snap(svc, points, k, ws0, cam,
                                             "基线首测"))
                _log(svc, job, "baseline: %s"
                     % " ".join("p%d=(%.2f,%.2f)" % (k + 1, base[k][0],
                                                    base[k][1])
                                for k in range(n_pt)))
            # ② WS_Rz 走 rz0+rot_rad → ③ 逐点走 pose_after 实测 Q_k
            svc._move_and_wait("WS_Rz", rz0 + rot_rad)
            _sleep_ms(wait_ms)
            rem = []
            for k in range(n_pt):
                pa = points[k]["pose_after"]
                _walk_limit_guard(svc, k, points[k]["tpl"],
                                  pa["WS_LS_X"], pa["WS_LS_Y"], rot_rad)
                _taught_go_pose(svc, pa)
                _sleep_ms(wait_ms)
                rem.append(_taught_snap(svc, points, k, ws0, cam, "旋转实测"))
            svc._move_and_wait("WS_Rz", rz0)             # ④ 回零进下一循环
            _sleep_ms(wait_ms)
            # 跨点闭式拟合 P_k→Q_k 实际旋转角 θ_i（_fit_rot：u=P−P̄、v=Q−Q̄
            # → θ=atan2(Σu×v,Σu·v)——示教点各自拍摄，像素集退化，跨点旋转
            # 信息在世界系；向量集退化守卫在 _fit_rot 内）
            th, _c = _fit_rot([b[2] for b in base], [r[2] for r in rem])
            e_i = (th - rot_rad + math.pi) % (2.0 * math.pi) - math.pi
            thetas.append(th)
            errs.append(e_i)
            # 逐点拟合残余 r=v−R(θ_i)u（世界系 mm→nm；平移残差与噪声，
            # 旋转分量已被 θ_i 吸收）
            bx = sum(b[2][0] for b in base) / n_pt
            by = sum(b[2][1] for b in base) / n_pt
            mx = sum(r[2][0] for r in rem) / n_pt
            my = sum(r[2][1] for r in rem) / n_pt
            c_, s_ = math.cos(th), math.sin(th)
            row = [i]
            for k in range(n_pt):
                ux, uy = base[k][2][0] - bx, base[k][2][1] - by
                vx, vy = rem[k][2][0] - mx, rem[k][2][1] - my
                ex = vx - (c_ * ux - s_ * uy)
                ey = vy - (s_ * ux + c_ * uy)
                ex_nm, ey_nm = ex * 1e6, ey * 1e6
                dcol_nm[k].append(ex_nm)
                drow_nm[k].append(ey_nm)
                dcols[k].append(ex_nm / k_nm)
                drows[k].append(ey_nm / k_nm)
                row += ["%.4f" % rem[k][0], "%.4f" % rem[k][1],
                        "%.4f" % dcols[k][-1], "%.4f" % drows[k][-1],
                        "%.6f" % rem[k][2][0], "%.6f" % rem[k][2][1]]
            row.append("%.6f" % (e_i * 1e6))
            rows.append(row)
            _log(svc, job, "cycle %d: theta=%.3furad err=%.3furad %s"
                 % (i + 1, th * 1e6, e_i * 1e6,
                    " ".join("p%d(dx=%.0f,dy=%.0f)nm"
                             % (k + 1, dcol_nm[k][-1], drow_nm[k][-1])
                             for k in range(n_pt))))
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
    header = ["index"]
    for k in range(n_pt):
        header += ["p%d_col_re" % (k + 1), "p%d_row_re" % (k + 1),
                   "p%d_d_col_px" % (k + 1), "p%d_d_row_px" % (k + 1),
                   "p%d_world_x_mm" % (k + 1), "p%d_world_y_mm" % (k + 1)]
    header.append("dtheta_urad")
    fname = _csv_write("ws_rotate_taught", header, rows)
    _log(svc, job, "data saved: %s" % fname)
    pts_series = []
    pts_stats = []
    for k in range(n_pt):
        pts_series.append({"tpl": points[k]["tpl"],
                           "d_col_nm": [round(v, 3) for v in dcol_nm[k]],
                           "d_row_nm": [round(v, 3) for v in drow_nm[k]],
                           "d_col_px": [round(v, 4) for v in dcols[k]],
                           "d_row_px": [round(v, 4) for v in drows[k]]})
        pts_stats.append({"tpl": points[k]["tpl"],
                          "col_s3_px": round(_s3(dcols[k]), 4),
                          "row_s3_px": round(_s3(drows[k]), 4),
                          "col_s3_nm": round(_s3(dcol_nm[k]), 2),
                          "row_s3_nm": round(_s3(drow_nm[k]), 2)})
    m_e, s_e = _mean_std(errs)
    angle_err = {"mean_urad": round(m_e * 1e6, 3),
                 "std_urad": round(s_e * 1e6, 3),
                 "s3_urad": round(3 * s_e * 1e6, 3)}
    return {
        "routine": "ws_rotate_repeat", "camera": "down",
        "repeat": repeat, "wait_ms": wait_ms,
        "series": {"points": pts_series,
                   # 角度误差序列（直接测角法主结果）+ 旧键兼容（=同一序列）
                   "angle_err_urad": [round(v * 1e6, 3) for v in errs],
                   "theta_fit_urad": [round(v * 1e6, 3) for v in thetas],
                   # 点0 兼容键（迭代 29b 结果 schema，旧判读路径消费）
                   "d_col_px": [round(v, 4) for v in dcols[0]],
                   "d_row_px": [round(v, 4) for v in drows[0]],
                   "dtheta_urad": [round(v * 1e6, 3) for v in errs]},
        "stats": {"points": pts_stats,
                  "angle_err": dict(angle_err),
                  "residual": {            # 点0 兼容键（迭代 29b schema）
                      "col_s3_px": pts_stats[0]["col_s3_px"],
                      "row_s3_px": pts_stats[0]["row_s3_px"],
                      "col_s3_nm": pts_stats[0]["col_s3_nm"],
                      "row_s3_nm": pts_stats[0]["row_s3_nm"]},
                  "dtheta": dict(angle_err)},   # 兼容键：填 e_i 统计
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"mode": "taught", "method": "direct_angle",
                   "rot_rad": rot_rad,
                   "n_points": n_pt,
                   "note": "直接测角法·双示教位（迭代 35）：基线走 pose 测 "
                           "P_k → rz0+rot_rad 走 pose_after（用户示教旋后"
                           "可见位，预测走位 Δ 段已删）测 Q_k → _fit_rot 闭式 "
                           "θ_i，e_i=θ_i−rot_rad（mean=系统角偏差、3σ=旋转角"
                           "重复性）；位置残余=拟合残余",
                   "taught_points": [
                       {"tpl": pt["tpl"],
                        "pose": {a: round(pt["pose"][a], 6)
                                 for a in pt["pose"]},
                        "pose_after": {a: round(pt["pose_after"][a], 6)
                                       for a in pt["pose_after"]}}
                       for pt in points]},
        "context": None, "elapsed_s": round(time.time() - t0, 3),
    }


# =====================================================================
# 7. camera_microrot 相机微小旋转角标定（迭代 32 起执行链自
#    metrology/microrot.py 迁入本模块——走位轴按 (cam, move_stage)
#    查表，迭代 34 down 可选 wafer 晶圆台）
# =====================================================================
def run_camera_microrot(svc, cam_id, tpl, repeat, wait_ms,
                        photo_pos1, photo_pos2, axes_touched, job=None,
                        move_stage="align"):
    """相机微旋标定主流程（迭代 32 迁入；参考公式/映射/结果契约与
    metrology/microrot.py 原文一致——纯 X 平移两点法逐轮
    φ_raw=−atan2(Δrow,Δcol)，undo swap_xy→undo dir_x/dir_y→指令世界
    运动方向角减观测角得 rot_i）。

    **measured.rot_mrad 约定（迭代 39 真值/认知分离重钉）**：输出=相机
    物理安装旋转角**真值估计**（=建议写入机器常数 rot_mrad 的**认知**
    值）。新架构下投影链施加 R(−θ_t)（view_rot 真值，图像内容反向
    旋转），旧双角色公式（内容 +r → 输出 +r）会得 −θ_t——故映射末端
    翻一次符号使输出=θ_t：θ_b=θ_t 时 solve∘project=恒等（补偿到位）。
    符号锚定用例=i39.microrot_view_rot_loop（注入 view_rot=+0.4mrad
    等效 → measured≈+0.4）。

    **走位轴按 (cam, move_stage) 查表（迭代 34）**：轴映射事实源=
    calibration.MICROROT_AXES——down+"align"（默认）→AS_X/AS_Y（下相机
    与上相机同挂对准台，见 vision_cameras.json mount.module=align_
    stage：动相机=等效动标记，标记相对相机**反向**移动，rel=−1，与
    up 同款）、down+"wafer"→WS_LS_X/WS_LS_Y rel=+1（动标记载体口径，
    与 JOG_AXES down 组同——下相机固定看晶圆标记动）、up 恒 align→
    AS_X/Y、coarse 恒 FA_X（不参与 move_stage）。符号自洽性：ang_cmd=
    纯 X 指令运动方向角随 rel 符号翻 π，像素位移向量同步翻 π，atan2
    差值不变——各 (cam, move_stage) 同一公式仍收敛各自 rot_mrad
    （align rel=−1 与 coarse rel=+1 已由 i31/i32 实测验证）。
    参数校验（repeat/wait_ms/template_name 必填/photo_pos 必填/软限位/
    move_stage 取值）已在 calibration.start 做完。cancel 检查点每轮首；
    轴位/灯/模式恢复由 calibration._run 通用现场路径负责。"""
    from microrot import _best_peak   # 懒导入防循环（microrot 顶层 import 本模块 _csv_write）
    from calibration import MICROROT_AXES         # 懒导入避免模块环
    t0 = time.time()
    cam = svc.vision.cams[cam_id].model
    jog = MICROROT_AXES[cam_id]["align" if cam_id == "coarse"
                               else move_stage]   # ((axis, rel_sign), ...)
    axes = [a for a, _s in jog]
    _check_marks(svc, cam_id, cam)
    for axis in axes:                         # 首步运动前记录原始位
        axes_touched.append((axis, svc._axis_pos(axis)))
    _log(svc, job, "camera microrot: cam=%s move_stage=%s template=%s "
         "repeat=%d wait_ms=%s pos1=%s pos2=%s axes=%s"
         % (cam_id, move_stage, tpl, repeat, wait_ms, photo_pos1,
            photo_pos2, axes))

    def _go(pos):
        for k, axis in enumerate(axes):
            svc._move_and_wait(axis, pos[k])

    def _snap():
        res = svc._identify(cam_id, tpl)["result"]
        return _best_peak(res, cam.score_threshold)

    # 指令世界运动方向角（标记相对相机；纯 X 杠杆 → 0 或 π）
    dx_cmd = (photo_pos2[0] - photo_pos1[0]) * jog[0][1]
    ang_cmd = 0.0 if dx_cmd > 0 else math.pi
    phis, raws = [], []
    rows = []
    pts = []
    for i in range(repeat):
        _check_cancel(job, i + 1)
        _progress(job, i + 1, repeat, "第 %d/%d 轮两点拍照" % (i + 1, repeat))
        _go(photo_pos1)
        _sleep_ms(wait_ms)
        c1, r1 = _snap()
        _go(photo_pos2)
        _sleep_ms(wait_ms)
        c2, r2 = _snap()
        dc, dr = c2 - c1, r2 - r1
        phi_raw = -math.atan2(dr, dc)         # 参考公式原式
        # 映射到真值约定（迭代 39）：undo swap → undo dir → 指令运动方向角
        # 减观测角。投影链 R(−θ_t)（view_rot 真值）下旧式（观测角−指令角）
        # 得 −θ_t，末端翻号使输出=θ_t（物理角真值估计=建议写入 rot_mrad
        # 的认知值）
        u, v = (dr, dc) if cam.swap_xy else (dc, dr)
        rot_i = ang_cmd - math.atan2(v * cam.dir_y, u * cam.dir_x)
        rot_i = (rot_i + math.pi) % (2.0 * math.pi) - math.pi
        phis.append(rot_i)
        raws.append(phi_raw)
        pts.append({"i": i, "col1": round(c1, 3), "row1": round(r1, 3),
                    "col2": round(c2, 3), "row2": round(r2, 3),
                    "phi_urad": round(rot_i * 1e6, 3)})
        rows.append([i, "%.4f" % c1, "%.4f" % r1, "%.4f" % c2, "%.4f" % r2,
                     "%.4f" % dc, "%.4f" % dr,
                     "%.6f" % phi_raw, "%.6f" % (rot_i * 1e6)])
        _log(svc, job, "cycle %d: p1=(%.2f,%.2f) p2=(%.2f,%.2f) "
             "phi=%.3furad (raw=%.3furad)"
             % (i + 1, c1, r1, c2, r2, rot_i * 1e6, phi_raw * 1e6))
        _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    fname = _csv_write(
        "camera_microrot",
        ["index", "col1", "row1", "col2", "row2", "d_col_px", "d_row_px",
         "phi_raw_rad", "rot_urad"], rows)
    _log(svc, job, "data saved: %s" % fname)
    m_v, s_v = _mean_std(phis)
    k_nm = nm_per_px(svc, cam_id)
    return {
        "routine": "camera_microrot", "camera": cam_id,
        "measured": {"rot_mrad": round(m_v * 1000.0, 3)},
        "current": {"rot_mrad": round(cam.rot * 1000.0, 6)},
        "repeat": repeat, "wait_ms": wait_ms,
        "points": pts,
        "series": {"phi_rad": [round(v, 9) for v in phis],
                   "phi_urad": [round(v * 1e6, 3) for v in phis],
                   "phi_raw_urad": [round(v * 1e6, 3) for v in raws]},
        "stats": {"rot": {"mean_mrad": round(m_v * 1000.0, 6),
                          "mean_urad": round(m_v * 1e6, 3),
                          "std_urad": round(s_v * 1e6, 3),
                          "s3_urad": round(3.0 * s_v * 1e6, 3)}},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"axes": axes,
                   "move_stage": move_stage,
                   "rel_sign": jog[0][1],
                   "photo_pos1": [round(v, 6) for v in photo_pos1],
                   "photo_pos2": [round(v, 6) for v in photo_pos2],
                   "span_mm": round(abs(photo_pos2[0] - photo_pos1[0]), 6),
                   "nm_per_px": round(k_nm, 4),
                   "note": "逐轮 φ=−atan2(Δrow,Δcol) 经相机 dir/swap 与"
                           "运动方向映射到真值约定（迭代 39：输出=相机物理"
                           "角真值估计=建议写入 rot_mrad 的认知值，投影链 "
                           "R(−θ_t) 下映射末端翻号）；走位轴按 "
                           "(cam, move_stage) 查表 MICROROT_AXES——down "
                           "align=AS_X/Y（动相机等效动标记 rel=−1）/wafer="
                           "WS_LS_X/Y（动标记载体 rel=+1），up 恒 align，"
                           "coarse 恒 FA_X（rel=+1）；rel 翻 π 时被减数与"
                           "像素位移同步翻 π，差值不变"},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
