# -*- coding: utf-8 -*-
"""metrology/best_focus.py — 相机最佳焦平面标定（迭代 42，routine
best_focus，cam=up/down）。

参考框架 测校/HoriMetrologyScirptsFramework/OptimalFocalCal.py 的**区间
收缩法**：z_start→z_end 按 step 扫 z-score 曲线，argmax 峰 → 区间缩到
峰点左右各一采样点、step 减半，下一轮；本实现两处适配：
- 清晰度评价改 **score 代理**（参考框架用 Sobel 原图清晰度，本链
  VSS_Com 不回图）：FAST 分数随 blur_ratio=|focus_err_um|/dof_um 线性
  下压（FAST_BASE_SCORE−0.15·min(blur,3)，blur>3 判废）、REAL NCC 天然
  退化——两模式单调可测且真机可得；每点 frames_per_point 帧取均值降噪；
- 末轮峰处三点**抛物线拟合**得亚步距 z*（z*=z_i+(s_{i−1}−s_{i+1})·
  step/(2(s_{i−1}−2s_i+s_{i+1}))，分母近 0 或峰在边界时取 argmax 点，
  与 calibration._focus_scan 同款防护）。

扫描轴与写回（机器常数 focus_level 组，apply 组路由两键→focus_level）：
- cam=down：扫 **AS_Z**（下相机挂对准台，动镜头；前置：当前位视野有标
  记——calibration._validate_best_focus 在 start 做 _check_marks），
  measured={as_z_chuck_focus_mm: z*}（下焦面=裸吸盘面 AS_Z；**注意**
  晶圆在位时测得的是晶圆面焦面，与裸吸盘面相差片厚——真机测校须在
  裸吸盘+吸盘标记工况做；仿真 FAST 裸吸盘无解析标记，见 i42 缺口注
  记）；z 默认窗口=as_z_chuck_focus_mm 现值 ±0.3mm；
- cam=up：先把 AS_Z→as_z_chuck_focus_mm、AS_X/Y→wafer_ega.
  as_anchor_xy_mm、BS_Arm_Z→head_z_upfocus_mm 附近（前置走位，视野
  预检在走位后做），扫 **BS_Arm_Z**（键合臂带吸盘动=标记动），
  measured={head_z_upfocus_mm: z*}（上焦面基准=裸吸盘面@AS_Z=
  as_z_chuck_focus 的 BS_Arm_Z）；z 默认窗口=head_z_upfocus_mm 现值
  ±0.3mm。

参数（calibration._validate_best_focus 规范化）：template_name（缺省
up=vision_marks.tpl_head_chuck / down=vision_marks.tpl_wafer_site；
给了须已注册）+ z_start/z_end（z_start<z_end，默认=焦面常数±0.3mm）
+ step_mm（默认 0.02，>0 且 ≤ 窗口）+ rounds（搜索轮数，默认 2，
[1,5]）+ frames_per_point（默认 3，[1,10]）+ wait_ms（默认 10）+
扫描行程在轴软限位内（start 校验）。

视觉模式：FAST/REAL 均可（作业不切模式）；离焦过深识别失败的点记
score=0 不阻塞（同 _focus_scan 惯例）。光源自管（作业开灯、finally
恢复原灯态）；轴位作业前记录、finally 恢复（up 先抬 BS_Arm_Z 再回
AS）；cancel 检查点每轮首+每点首。

结果契约：measured={<焦面常数键>: z*} + current 同键现值；
series.rounds=[{round, step_mm, z_mm[], score[], blur_ratio[],
peak_z_mm}]（GUI 画各轮 z-score 曲线）+ series.peaks_mm（各轮峰历史）
+ series.best_z_mm；detail 记 axis/const_key/窗口/step/rounds/fpp/
fit（parabola|argmax|peak_at_edge）/preset（up 前置走位）/note；CSV
best_focus_<cam>_MMDD_HH_MM_SS.csv（round,point,z_mm,score,
blur_ratio,focus_err_um；落 runtime/metrology_data/）。"""
import os
import sys
import time

# 同 rotate_center.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _check_marks, _log,
                               _progress, _sleep_ms)
    from extra_routines import _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _check_marks, _log,
                               _progress, _sleep_ms)
    from extra_routines import _csv_write

_EPS_DENOM = 1e-9          # 抛物线分母退化阈值（平台/平顶曲线取 argmax）


def _z_grid(z_start, z_end, step):
    """z_start→z_end 步进网格（含端点；末段不足一步时补 z_end 端点，
    与参考框架 OptimalFocalCal 同式）。"""
    zs = []
    z = z_start
    while z <= z_end + 1e-12:
        zs.append(round(z, 6))
        z += step
    if zs and abs(zs[-1] - z_end) > 1e-9:
        zs.append(round(z_end, 6))
    return zs


def _parabola_peak(zs, ss, i):
    """峰点 i 的三点抛物线拟合（须 0<i<len−1；分母退化回退 zs[i]）。"""
    z1, z2, z3 = zs[i - 1], zs[i], zs[i + 1]
    s1, s2, s3 = ss[i - 1], ss[i], ss[i + 1]
    denom = s1 - 2.0 * s2 + s3
    if abs(denom) < _EPS_DENOM:
        return z2
    return z2 + (s1 - s3) * (z3 - z1) / (4.0 * denom)


def run_best_focus(svc, cam_id, params, job=None):
    """最佳焦平面标定主流程。参数校验（模板/z 窗口/step/rounds/fpp/
    行程/down 视野预检）已在 calibration.start 做完
    （_validate_best_focus）。"""
    from calibration import CalibError         # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    cam = svc.vision.cams[cam_id].model
    rt = svc.vision.cams[cam_id]
    tpl = params["template_name"]
    axis = params["axis"]                    # down=AS_Z / up=BS_Arm_Z
    const_key = params["const_key"]          # 写回键（focus_level 组）
    z_start = params["z_start"]
    z_end = params["z_end"]
    step = params["step_mm"]
    rounds = params["rounds"]
    fpp = params["frames_per_point"]
    wait_ms = params["wait_ms"]
    preset = params.get("preset")            # up 前置走位 dict（axis→pos）
    old_light = bool(svc.points.get(cam.light_point, False))
    # 作业前轴位（finally 恢复；up 先抬 BS_Arm_Z 再回 AS——防撞顺序）
    restore_axes = ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y") if cam_id == "up" \
        else ("AS_Z",)
    saved = {}
    with eng.lock:
        for a in restore_axes:
            saved[a] = eng.axes[a].pos
    _log(svc, job, "best focus: cam=%s axis=%s const=%s template=%s "
         "window=[%.4f,%.4f] step=%.4f rounds=%d fpp=%d wait_ms=%s"
         % (cam_id, axis, const_key, tpl, z_start, z_end, step, rounds,
            fpp, wait_ms))

    scan_rounds = []                   # 每轮 {round,step,zs,scores,blurs,errs,peak}
    peaks = []
    rows = []
    z_best = None
    fit_note = "parabola"
    try:
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        if preset:                     # up 前置走位（AS→锚点/焦面基准）
            for a in ("AS_Z", "AS_X", "AS_Y", "BS_Arm_Z"):
                if a in preset:
                    svc._move_and_wait(a, preset[a])
            _sleep_ms(wait_ms)
            _check_marks(svc, cam_id, cam)   # 走位到位后视野预检
        lo, hi, h = z_start, z_end, step
        pts_done = 0
        for r in range(rounds):
            _check_cancel(job, r + 1)
            zs = _z_grid(lo, hi, h)
            scores, blurs, errs = [], [], []
            n_est = pts_done + len(zs) * (rounds - r)
            for k, z in enumerate(zs):
                _check_cancel(job, r + 1)
                _progress(job, pts_done + k + 1, n_est,
                          "第 %d/%d 轮 z=%.4f（%d/%d）"
                          % (r + 1, rounds, z, k + 1, len(zs)))
                svc._move_and_wait(axis, z)
                _sleep_ms(wait_ms)
                ss, bb, ee = [], [], []
                for _f in range(fpp):
                    try:
                        entry = svc._identify(cam_id, tpl)
                        res = entry["result"]
                        ss.append(max(res.get("lstScore") or [0.0]))
                        if entry.get("blur_ratio") is not None:
                            bb.append(float(entry["blur_ratio"]))
                        if entry.get("focus_err_um") is not None:
                            ee.append(float(entry["focus_err_um"]))
                    except Exception:
                        # 离焦过深/识别失败的点记 0 不阻塞（同
                        # _focus_scan 惯例）
                        ss.append(0.0)
                scores.append(sum(ss) / len(ss))
                blurs.append(sum(bb) / len(bb) if bb else None)
                errs.append(sum(ee) / len(ee) if ee else None)
                rows.append([r, k, "%.6f" % zs[k], "%.4f" % scores[k],
                             "" if blurs[k] is None else "%.4f" % blurs[k],
                             "" if errs[k] is None else "%.2f" % errs[k]])
                _log(svc, job, "round %d z=%.4f: score=%.4f%s"
                     % (r + 1, zs[k], scores[k],
                        "" if blurs[k] is None
                        else " blur=%.2f" % blurs[k]))
            pts_done += len(zs)
            if max(scores) <= 0.0:
                raise CalibError(
                    "第 %d 轮整个扫描范围都没识别到标记，检查物料/光照/"
                    "模板/焦面常数先决" % (r + 1))
            i_pk = max(range(len(zs)), key=lambda i: scores[i])
            peaks.append(zs[i_pk])
            scan_rounds.append({"round": r + 1, "step_mm": h,
                                "z_mm": zs, "score": scores,
                                "blur_ratio": blurs, "peak_z_mm": zs[i_pk]})
            _log(svc, job, "round %d peak: z=%.6f score=%.4f"
                 % (r + 1, zs[i_pk], scores[i_pk]))
            if r == rounds - 1:
                # 末轮：峰处三点抛物线拟合（亚步距）；峰在边界/分母退化
                # → argmax 点
                if 0 < i_pk < len(zs) - 1:
                    z_best = _parabola_peak(zs, scores, i_pk)
                else:
                    z_best = zs[i_pk]
                    fit_note = ("peak_at_edge（峰值在扫描边界，建议扩 "
                                "z_start/z_end）")
                if fit_note == "parabola" and abs(
                        scores[i_pk - 1] - 2.0 * scores[i_pk]
                        + scores[i_pk + 1]) < _EPS_DENOM:
                    fit_note = "argmax（平顶曲线抛物线分母退化）"
                break
            # 区间收缩：峰点左右各一采样点（边界侧保持），step 减半
            if i_pk > 0:
                lo = zs[i_pk - 1]
            if i_pk < len(zs) - 1:
                hi = zs[i_pk + 1]
            h = h / 2.0
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
        "best_focus_%s" % cam_id,
        ["round", "point", "z_mm", "score", "blur_ratio", "focus_err_um"],
        rows)
    _log(svc, job, "data saved: %s" % fname)
    cur = 0.0
    try:
        cur = float((eng.constants.get() or {}).get("focus_level", {}).get(
            const_key, 0.0))
    except Exception:
        pass
    return {
        "routine": "best_focus", "camera": cam_id,
        "template_name": tpl,
        "measured": {const_key: round(z_best, 6)},
        "current": {const_key: round(cur, 6)},
        "points": [{"round": rd["round"], "z_mm": z, "score": s}
                   for rd in scan_rounds
                   for z, s in zip(rd["z_mm"], rd["score"])],
        "series": {"rounds": [{"round": rd["round"],
                               "step_mm": rd["step_mm"],
                               "z_mm": [round(v, 6) for v in rd["z_mm"]],
                               "score": [round(v, 5) for v in rd["score"]],
                               "blur_ratio": rd["blur_ratio"],
                               "peak_z_mm": round(rd["peak_z_mm"], 6)}
                              for rd in scan_rounds],
                   "peaks_mm": [round(v, 6) for v in peaks],
                   "best_z_mm": round(z_best, 6)},
        "confidence": round(max(scan_rounds[-1]["score"]), 4)
        if scan_rounds else None,
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {"axis": axis, "const_key": const_key,
                   "z_start": z_start, "z_end": z_end, "step_mm": step,
                   "rounds": rounds, "frames_per_point": fpp,
                   "wait_ms": wait_ms, "fit": fit_note,
                   "peaks_mm": [round(v, 6) for v in peaks],
                   "preset": ({a: round(v, 6) for a, v in preset.items()}
                              if preset else None),
                   "note": "区间收缩法（参考 OptimalFocalCal）：每轮 argmax"
                           " → 区间缩到峰点左右各一采样点、step 减半；末轮"
                           "峰处三点抛物线拟合得亚步距 z*；清晰度=score "
                           "代理（FAST 分数随 blur 下压/REAL NCC 退化，"
                           "blur>3 判废点 score=0）"},
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
