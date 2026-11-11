# -*- coding: utf-8 -*-
"""metrology/repeatability.py — 相机对准重复性测校核心（静态/动态）。

对照参考框架 HoriMetrologyScirptsFramework 的 SingleCameraAlignStability /
AlignStabilityDynamic 两个测校脚本，但：
- 拍照走本后端的 svc._identify（calibration.py 的 REAL 全链路识别），
  质心取 svc._centroid（得分过滤后的判真点云质心，px）；
- 动态轴映射沿用 calibration.JOG_AXES（down→WS_LS_X/Y 动晶圆台、
  up→AS_X/Y 动对准台相机、coarse→FA_X 动翻转模块，coarse 仅 X 向单轴）；
  迭代 25 起每循环语义对齐参考脚本（绝对坐标）：各轴先到 起始位置
  start_xy → wait → 到 拍照位置 photo_xy → wait → 拍照（多轴：X 分量给
  第 1 根轴、Y 分量给第 2 根；coarse 单轴只用 X）；start_xy/photo_xy 由
  calibration.start 校验+规范化（travel_mm 是兼容别名：photo_xy=各轴当前
  位、start_xy=photo−travel），并记录各动轴实际到位位置；
- nm 换算按**该相机**机器常数 k_nm_per_px = pixel_um/mag*1000
  （禁止沿用参考脚本硬编码的 137nm：本机 up/down=600nm/px、
  coarse≈9133nm/px）；
- CSV 落盘 runtime/metrology_data/，文件名沿用参考框架
  single_camera_stability_MMDD_HH_MM_SS.csv /
  align_stability_dynamic_MMDD_HH_MM_SS.csv；
- 操作日志（对齐参考脚本的 textEdit 流）：svc.log(job, line)——每循环
  "cycle time: i/n"、每拍 "detect result: row/col"、CSV 写完
  "data saved: <fname>"（job 非 None 且 svc 有 log 方法时）。

取消走协作式检查点（迭代 29b）：每循环开头 _check_cancel 查
job["cancel_req"]（POST /api/calibration/cancel 置位），置位即抛
CalibCanceled → calibration._run 收 canceled 终态；wait 仍用简单
time.sleep（取消在下一循环边界生效）。
"""
import csv
import math
import os
import time

# 落盘目录：backend/runtime/metrology_data/（相对本文件定位，不依赖
# 调用方 sys.path）
DATA_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    "runtime", "metrology_data"))


def nm_per_px(svc, cam_id):
    """该相机的机器常数像素当量（nm/px）= pixel_um/mag*1000。"""
    cam = svc.vision.cams[cam_id].model
    return cam.pixel_um / cam.mag * 1000.0


def jog_axes(cam_id):
    """该相机的载体点动轴列表（轴名顺序固定）。懒导入避免模块环
    （calibration 顶层 import 本模块；vision_service 同此先例）。"""
    from calibration import JOG_AXES
    return [a for a, _s in JOG_AXES[cam_id]]


def _check_marks(svc, cam_id, cam):
    """几何前置：视野内须有至少一个标记（无标记载体/没对准 → 清晰错误）。
    不能只靠识别侧拦：REAL 在无特征纹理（陶瓷面/吸盘）上 NCC 可能误判
    过阈值（2026-09 实测陶瓷面 score>0.6），会"测"出一组垃圾散布。
    与 calibration._focus_scan 的 collect_marks 前置同一惯例。"""
    import vision_model as vm
    from calibration import CalibError
    positions, poses = vm._engine_poses(svc.engine)
    cam_pos, _yaw = vm.camera_pose(svc.engine, cam, poses, positions)
    marks = vm.collect_marks(svc.engine, cam, positions, poses)
    if not marks:
        raise CalibError("相机 %s 无可用标记（先上片/取 die）" % cam_id)
    in_fov = 0
    for m in marks:
        col, row = cam.project(m.x - cam_pos[0], m.y - cam_pos[1])
        if cam.in_fov(col, row):
            in_fov += 1
    if not in_fov:
        near = min(marks, key=lambda m: abs(m.x - cam_pos[0])
                   + abs(m.y - cam_pos[1]))
        raise CalibError(
            "相机 %s 视野内无标记（最近的 %s 距视野中心 %.2fmm——先走位对准）"
            % (cam_id, near.name,
               math.hypot(near.x - cam_pos[0], near.y - cam_pos[1])))


def _sleep_ms(wait_ms):
    """拍照前等待。简单 time.sleep——协作式取消（_check_cancel）在下一
    循环边界生效，wait 期间不中断。"""
    if wait_ms > 0:
        time.sleep(wait_ms / 1000.0)


def _angle_deg(res, cam):
    """识别结果角度（deg）：取与质心同口径的判真点（score≥阈值）均值；
    无 lstAngle 或无判真点 → None。"""
    angs = [a for a, s in zip(res.get("lstAngle") or [],
                              res.get("lstScore") or [])
            if a is not None and s >= cam.score_threshold]
    if not angs:
        return None
    return sum(angs) / len(angs)


def _context(svc, cam_id, cam, col, row):
    """拍照上下文（2026-09-08 测校数据增强）：一次拍照时刻的辅助判读
    数据——相机世界位姿、质心反算的世界坐标、相关轴 pos/actual（含误差
    链 actual-value 分量）。全部从 engine/vision_model 直读（无二次 HTTP）。
    返回 dict；任何子项失败置 None 不阻塞拍照。"""
    out = {"cam_world_mm": None, "mark_world_mm": None,
           "axes_pos": {}, "axes_actual": {}}
    try:
        import vision_model as vm
        positions, poses = vm._engine_poses(svc.engine)
        cam_pos, _yaw = vm.camera_pose(svc.engine, cam, poses, positions)
        out["cam_world_mm"] = [round(v, 6) for v in cam_pos]
        if col is not None:
            wx, wy, wz = cam.solve_pixel(col, row, cam_pos)
            out["mark_world_mm"] = [round(wx, 6), round(wy, 6),
                                    round(wz, 6)]
    except Exception:
        pass
    try:
        with svc.engine.lock:
            for name, ax in svc.engine.axes.items():
                out["axes_pos"][name] = round(ax.pos, 7)
                if svc.engine.errors is not None:
                    try:
                        act, _d = svc.engine.errors.actual(
                            name, ax.pos, base=ax.x_actual)
                        out["axes_actual"][name] = round(act, 7)
                    except Exception:
                        out["axes_actual"][name] = None
                else:
                    out["axes_actual"][name] = round(ax.pos, 7)
    except Exception:
        pass
    return out


def _axis_actual(svc, axis):
    """轴实际位置（编码器口径，迭代 48）：误差链 actual（含到位 repeat
    重采样/误差地图，error_system.actual(name, pos, base=x_actual)）；
    errors 未挂/异常回退名义 pos。动态测校 axis_log/到位 3σ 用——名义
    pos 到位后恒=target（3σ 恒 0，迭代 24 起已知无意义口径），到位
    重复性物理量在 actual 上（每次到位 repeat_offset 重采样）。"""
    try:
        with svc.engine.lock:
            ax = svc.engine.axes[axis]
            if svc.engine.errors is None:
                return ax.pos
            act, _d = svc.engine.errors.actual(axis, ax.pos,
                                               base=ax.x_actual)
            return act
    except Exception:
        try:
            return svc._axis_pos(axis)
        except Exception:
            return float("nan")


def _shot(svc, cam_id, tpl, cam):
    """一次拍照 → (col_px, row_px, angle_deg|None)。无标记/识别失败由
    svc._identify/_centroid 抛清晰错误（作业进 error 态）。"""
    res = svc._identify(cam_id, tpl)["result"]
    col, row = svc._centroid(res, cam.score_threshold)
    return col, row, _angle_deg(res, cam)


def _progress(job, i, n, note):
    if job is not None:
        job["progress"] = {"i": i, "n": n, "note": note}


def _check_cancel(job, i=None):
    """协作式取消检查点（迭代 29b，每循环开头调用）：job["cancel_req"]
    置位（POST /api/calibration/cancel）即抛 CalibCanceled——由
    calibration._run/_run_simple 捕获置 canceled 终态（finally 现场
    恢复照跑）。懒导入避免模块环（同 jog_axes 先例）。"""
    if job is not None and job.get("cancel_req"):
        from calibration import CalibCanceled
        raise CalibCanceled("作业被用户取消（第 %s 次循环边界生效）"
                            % (i if i is not None else "?"))


def _log(svc, job, line):
    """作业操作日志（job 非 None 且 svc 实现了 log 时）。"""
    if job is not None and hasattr(svc, "log"):
        svc.log(job, line)


def _mean_std(vals):
    """总体口径（ddof=0，与参考脚本 np.std 默认一致）。"""
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return mean, math.sqrt(var)


def _write_csv(prefix, cols, rows, angs, axis_log, extra_cols=None,
               extra_rows=None):
    """列：index,col,row[,angle_deg][,每根动轴实际位置][,辅助判读列]。
    extra_cols/extra_rows（2026-09-08 数据增强）：mark_world_x/y/z_mm、
    cam_world_x/y/z_mm 等世界坐标辅助列。返回文件名。"""
    os.makedirs(DATA_DIR, exist_ok=True)
    fname = "%s_%s.csv" % (prefix, time.strftime("%m%d_%H_%M_%S"))
    # 同秒连跑防撞名（作业串行，但 n 小 wait=0 时两个作业可同秒结束）
    base, ext = os.path.splitext(fname)
    k = 1
    while os.path.exists(os.path.join(DATA_DIR, fname)):
        k += 1
        fname = "%s_%d%s" % (base, k, ext)
    axes = list(axis_log or [])
    has_ang = angs is not None and any(a is not None for a in angs)
    header = ["index", "col", "row"]
    if has_ang:
        header.append("angle_deg")
    header += axes
    header += list(extra_cols or [])
    with open(os.path.join(DATA_DIR, fname), "w", newline="",
              encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for i in range(len(cols)):
            r = [i, "%.4f" % cols[i], "%.4f" % rows[i]]
            if has_ang:
                r.append("" if angs[i] is None else "%.6f" % angs[i])
            r += ["%.7f" % axis_log[a][i] for a in axes]
            r += list((extra_rows[i] if extra_rows is not None
                       and i < len(extra_rows) else []))
            w.writerow(r)
    return fname


def _assemble(routine, cam_id, repeat, wait_ms, k_nm, cols, rows, angs,
              axis_log, fname, t0, tpl_name=None, start_xy=None,
              photo_xy=None, context=None):
    """按契约组装结果 dict（panel 按字段名渲染，勿动字段名）。
    angle 序列有值才带 angle_deg/stats.angle；axis_log/stats.axes 仅动态
    （静态为 None）。series_nm = px 序列 × k_nm（round 3 位，前端画 nm
    趋势图直接用）；动态带 detail.start_xy/photo_xy（若有）。
    context（2026-09-08 数据增强）：每拍上下文列表（相机世界位姿/
    质心世界坐标/全轴 pos+actual），带 result.context 供判读。"""
    m_c, s_c = _mean_std(cols)
    m_r, s_r = _mean_std(rows)
    stats = {
        "row": {"mean_px": round(m_r, 5), "std_px": round(s_r, 5),
                "s3_px": round(3 * s_r, 5), "s3_nm": round(3 * s_r * k_nm, 2)},
        "col": {"mean_px": round(m_c, 5), "std_px": round(s_c, 5),
                "s3_px": round(3 * s_c, 5), "s3_nm": round(3 * s_c * k_nm, 2)},
        "angle": None,
        "axes": None,
    }
    series = {"row": [round(v, 4) for v in rows],
              "col": [round(v, 4) for v in cols],
              "angle_deg": None}
    if angs is not None and any(a is not None for a in angs):
        vals = [a for a in angs if a is not None]
        m_a, s_a = _mean_std(vals)
        stats["angle"] = {"mean_deg": round(m_a, 7),
                          "std_deg": round(s_a, 7),
                          # deg → µrad：×π/180×1e6
                          "s3_urad": round(3 * s_a * math.pi / 180.0 * 1e6, 3)}
        series["angle_deg"] = [None if a is None else round(a, 6)
                               for a in angs]
    if axis_log is not None:
        ax_stats = {}
        for a, vals in axis_log.items():
            m_x, s_x = _mean_std(vals)
            ax_stats[a] = {"mean_mm": round(m_x, 7),
                           "std_mm": round(s_x, 9),
                           "s3_um": round(3 * s_x * 1000.0, 4)}
        stats["axes"] = ax_stats
    # 数据增强：世界坐标序列统计（mark_world/cam_world）
    if context:
        try:
            mw = [c["mark_world_mm"][:2] for c in context
                  if c.get("mark_world_mm")]
            if len(mw) == len(cols):
                for j, key in enumerate(("x", "y")):
                    m_w, s_w = _mean_std([p[j] for p in mw])
                    stats.setdefault("mark_world", {})[key] = {
                        "mean_mm": round(m_w, 6),
                        "s3_um": round(3 * s_w * 1000.0, 4)}
        except Exception:
            pass
    detail = {}
    if start_xy is not None:
        detail["start_xy"] = [round(v, 6) for v in start_xy]
    if photo_xy is not None:
        detail["photo_xy"] = [round(v, 6) for v in photo_xy]
    return {
        "routine": routine,
        "camera": cam_id,
        "repeat": repeat,
        "wait_ms": wait_ms,
        "nm_per_px": round(k_nm, 4),
        "template_name": tpl_name,
        "series": series,
        "series_nm": {"row": [round(v * k_nm, 3) for v in rows],
                      "col": [round(v * k_nm, 3) for v in cols]},
        "axis_log": ({a: [round(v, 7) for v in vals]
                      for a, vals in axis_log.items()}
                     if axis_log is not None else None),
        "context": context,
        "stats": stats,
        "csv": "runtime/metrology_data/" + fname,
        "csv_name": fname,
        "detail": detail,
        "elapsed_s": round(time.time() - t0, 3),
    }


def run_static(svc, cam_id, tpl, repeat, wait_ms, job=None):
    """静态对准重复性：设备保持静止，每拍前等 wait_ms，连拍 repeat 次。
    tpl 由 calibration 决定（自建临时模板或 params.template_name 指定的
    用户模板——用户模板不删）。

    返回契约 dict（见 _assemble）；异常（无标记/识别失败）抛给
    calibration._run 统一收 error 态。CSV=single_camera_stability_*。
    2026-09-08 数据增强：每拍附带上下文（相机世界位姿/质心世界坐标/
    全轴 pos+actual），入 CSV 辅助列与 result.context 序列。"""
    t0 = time.time()
    cam = svc.vision.cams[cam_id].model
    k_nm = nm_per_px(svc, cam_id)
    _check_marks(svc, cam_id, cam)
    _log(svc, job, "static repeatability: camera=%s template=%s repeat=%d "
         "wait_ms=%s" % (cam_id, tpl, repeat, wait_ms))
    cols, rows, angs = [], [], []
    ctx = []
    for i in range(repeat):
        _check_cancel(job, i + 1)
        _progress(job, i + 1, repeat, "第 %d/%d 次拍照" % (i + 1, repeat))
        _sleep_ms(wait_ms)
        col, row, ang = _shot(svc, cam_id, tpl, cam)
        cols.append(col)
        rows.append(row)
        angs.append(ang)
        ctx.append(_context(svc, cam_id, cam, col, row))
        _log(svc, job, "detect result: row: %.2f, col: %.2f" % (row, col))
        _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    EXTRA_COLS = ["mark_world_x_mm", "mark_world_y_mm", "mark_world_z_mm",
                  "cam_world_x_mm", "cam_world_y_mm", "cam_world_z_mm"]
    extra_rows = []
    for c in ctx:
        mw = c.get("mark_world_mm") or ["", "", ""]
        cw = c.get("cam_world_mm") or ["", "", ""]
        extra_rows.append(
            ["" if v is None else "%.6f" % v for v in list(mw) + list(cw)])
    fname = _write_csv("single_camera_stability", cols, rows, angs, None,
                       extra_cols=EXTRA_COLS, extra_rows=extra_rows)
    _log(svc, job, "data saved: %s" % fname)
    return _assemble("align_repeat_static", cam_id, repeat, wait_ms, k_nm,
                     cols, rows, angs, None, fname, t0, tpl_name=tpl,
                     context=ctx)


def run_dynamic(svc, cam_id, tpl, repeat, wait_ms, start_xy, photo_xy,
                axes_touched, job=None, jog_axes_override=None,
                routine=None):
    """动态对准重复性（迭代 25 起绝对坐标语义，对齐参考脚本
    AlignStabilityDynamic）：每循环 各轴先到 起始位置 start_xy → wait →
    到 拍照位置 photo_xy → wait → 拍照；记录各动轴**实际**到位位置
    （axis_log，迭代 48 起=误差链 actual 编码器口径——含到位 repeat
    重采样，到位 3σ 即配置 repeat_3σ；旧记名义 pos 恒=target、3σ 恒 0
    无意义，迭代 24 起已知口径）。start_xy/photo_xy=[x,y] mm 绝对坐标：
    多轴时 X 分量给第 1 根轴、Y 分量给第 2 根轴；coarse 单轴只用 X
    （Y 忽略）。

    travel_mm 兼容别名：calibration.start 在未给 start_xy/photo_xy 时按
    旧语义换算（photo_xy=各轴当前位、start_xy=photo−travel）后仍以
    start_xy/photo_xy 传入，本函数不再感知 travel_mm。

    jog_axes_override（2026-09-08 需求②）：指定运动组轴名列表（如
    ["AS_X","AS_Y"]——下相机选对准台动）；缺省用 jog_axes(cam_id) 默认组。
    routine（2026-09-08）：结果 routine 名覆盖（ws/as_move_repeat 别名）。

    各动轴 (axis, 首次运动前的当前位) append 进 axes_touched（恢复由
    calibration._run 的 finally 统一做）。参数校验（repeat/wait_ms/
    start_xy/photo_xy/软限位/流程忙）已在 calibration.start 做完。
    CSV=align_stability_dynamic_*（别名 routine 亦沿用，csv_name 由
    routine 名派生见 _assemble）。2026-09-08 数据增强：每拍附带上下文
    （相机世界位姿/质心世界坐标/全轴 pos+actual）。"""
    t0 = time.time()
    cam = svc.vision.cams[cam_id].model
    k_nm = nm_per_px(svc, cam_id)
    _check_marks(svc, cam_id, cam)
    axes = list(jog_axes_override) if jog_axes_override else jog_axes(cam_id)
    for axis in axes:               # 首步运动前记录原始位
        axes_touched.append((axis, svc._axis_pos(axis)))
    _log(svc, job, "dynamic repeatability: camera=%s template=%s repeat=%d "
         "wait_ms=%s start=%s photo=%s axes=%s"
         % (cam_id, tpl, repeat, wait_ms, start_xy, photo_xy, axes))
    cols, rows, angs = [], [], []
    ctx = []
    axis_log = {a: [] for a in axes}
    for i in range(repeat):
        _check_cancel(job, i + 1)
        _progress(job, i + 1, repeat,
                  "第 %d/%d 次往复+拍照" % (i + 1, repeat))
        for k, axis in enumerate(axes):      # 各轴 → 起始位置
            svc._move_and_wait(axis, start_xy[k])
        _sleep_ms(wait_ms)
        for k, axis in enumerate(axes):      # 各轴 → 拍照位置
            svc._move_and_wait(axis, photo_xy[k])
        _sleep_ms(wait_ms)
        col, row, ang = _shot(svc, cam_id, tpl, cam)
        cols.append(col)
        rows.append(row)
        angs.append(ang)
        ctx.append(_context(svc, cam_id, cam, col, row))
        for axis in axes:
            axis_log[axis].append(_axis_actual(svc, axis))
        _log(svc, job, "detect result: row: %.2f, col: %.2f" % (row, col))
        _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    EXTRA_COLS = ["mark_world_x_mm", "mark_world_y_mm", "mark_world_z_mm",
                  "cam_world_x_mm", "cam_world_y_mm", "cam_world_z_mm"]
    extra_rows = []
    for c in ctx:
        mw = c.get("mark_world_mm") or ["", "", ""]
        cw = c.get("cam_world_mm") or ["", "", ""]
        extra_rows.append(
            ["" if v is None else "%.6f" % v for v in list(mw) + list(cw)])
    fname = _write_csv("align_stability_dynamic", cols, rows, angs, axis_log,
                       extra_cols=EXTRA_COLS, extra_rows=extra_rows)
    _log(svc, job, "data saved: %s" % fname)
    return _assemble(routine or "align_repeat_dynamic", cam_id, repeat,
                     wait_ms, k_nm, cols, rows, angs, axis_log, fname, t0,
                     tpl_name=tpl, start_xy=start_xy, photo_xy=photo_xy,
                     context=ctx)
