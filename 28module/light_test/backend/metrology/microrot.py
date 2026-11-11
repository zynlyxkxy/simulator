# -*- coding: utf-8 -*-
"""metrology/microrot.py — 相机微小旋转角标定（迭代 31，routine
camera_microrot，三相机 up/down/coarse 全支持）。

参考做法（真机脚本 测校/HoriMetrologyScirptsFramework/
AlignCameraRzCalibration / CameraTiltAngleCalibration，SOP
测校/重构后SOP/相机倾斜测校_SOP_V2）：纯 X 平移两点法——运动台带标记
沿 X 走两个拍照位，各拍一次记标记像素 (col,row)，逐轮
φ_raw = −atan2(row2−row1, col2−col1)（rad），repeat 轮取均值。

写回语义（与 machine_constants cameras[cam].rot_mrad 同约定，apply 链
fields=["rot_mrad"] 写回热生效；迭代 39 真值/认知分离重钉）：φ_raw 是
图像系观测角，须映射到真值约定才是可写回的 rot_mrad——undo swap_xy →
undo dir_x/dir_y → 指令世界运动方向角减观测角（点动轴符号语义同
calibration.JOG_AXES：down/coarse 动标记载体 +、up 动相机 −；纯 X 指令
下方向角 = 0 或 π）。投影链 R(−θ_t)（view_rot=相机物理安装旋转角真值，
图像内容反向旋转）下旧双角色公式（内容 +r → 输出 +r）得 −θ_t，映射
末端翻一次符号使输出=θ_t——measured.rot_mrad=相机物理角**真值估计**
（=建议写入 rot_mrad 的**认知**值）：θ_b=θ_t 时 solve∘project=恒等
（补偿到位），measured≈current 时写回幂等，写错符号的偏差会被放大而
不是吸收——三相机同一公式全部收敛各自真值。

X 间距口径（参考 SOP"建议 ≥10mm 且两拍照位标记均在视野内"）：coarse
FOV 27.4mm 按 SOP ≥10mm；up/down FOV 仅 1.8mm——≥10mm 杠杆物理不可
行（标记必出视场），按视场可用跨度取 [0.3, 1.4]mm（间距校验在
calibration._validate_microrot 做，含逐轴软限位/纯 X/成对/模板存在性）。
模板：params.template_name 给了须已注册（不自建不删）；缺省=作业在
当前位自建特征 ROI 临时模板（601px——默认中央 3/4 ROI 匹配可达范围
仅 ±374px 装不下杠杆行程，见 calibration._microrot_tpl_roi），结束删。

结果契约（标定程序形态，同 calibration 的 pixel_equivalent 等：
measured/current/points/detail/ts + 本类附加 series/stats/csv）：
measured.rot_mrad=mean(rot_i)×1000（迭代 39：物理角真值估计=建议写回的
认知值）、current.rot_mrad（机器常数认知现值）、
stats.rot={mean/std/s3 µrad}、series.phi_urad=逐轮 rot_i（µrad，
常数约定）、series.phi_raw_urad=逐轮 φ_raw（参考公式原式）、CSV
camera_microrot_MMDD_HH_MM_SS.csv。cancel 检查点每轮首；轴位/灯/
模式/模板恢复由 calibration._run 通用现场路径负责（动过的轴 append
进 axes_touched）。"""
import math
import os
import sys
import time

# 同 extra_routines.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _check_marks, _log,
                               _mean_std, _progress, _sleep_ms, nm_per_px)
    from extra_routines import _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _check_marks, _log,
                               _mean_std, _progress, _sleep_ms, nm_per_px)
    from extra_routines import _csv_write


def _best_peak(res, min_score):
    """单特征模板匹配取最强峰（score ≥ 阈值）；无过阈峰抛清晰错误。
    不用 svc._centroid 多点平均：远距滑窗下纹理/边缘幽灵峰（score 贴阈
    0.6×）会把质心拖离真峰——实测 coarse die 角点模板 0.617 分幽灵峰与
    真峰 0.998 平均后质心偏移 ~870px（迭代 31 冒烟）。"""
    from calibration import CalibError
    best, bi = -1.0, None
    for i, s in enumerate(res.get("lstScore") or []):
        if s is not None and s >= min_score and s > best:
            best, bi = s, i
    if bi is None:
        raise CalibError("无判真匹配点（得分均低于 %.2f）" % min_score)
    return res["lstCol"][bi], res["lstRow"][bi]


def run_camera_microrot(svc, cam_id, tpl, repeat, wait_ms,
                        photo_pos1, photo_pos2, axes_touched, job=None):
    """相机微旋标定主流程。每轮：走 pos1 → wait → 拍照记 (col1,row1) →
    走 pos2 → wait → 拍照记 (col2,row2) → φ_raw=−atan2(Δrow,Δcol) →
    映射到真值约定得 rot_i（迭代 39：输出=物理角真值估计=建议写入
    rot_mrad 的认知值，映射末端翻号）。参数校验（repeat/wait_ms/
    photo_pos/软限位/模板存在性）已在 calibration.start 做完。
    精度口径（与参考离线仿真一致）：σ_φ≈√2·σ_pt/(L√R)——亚 mrad 真值
    在 up/down 短杠杆（≤1.4mm 视场限）下需更多轮次/更低质心噪声。"""
    from calibration import JOG_AXES         # 懒导入避免模块环
    t0 = time.time()
    cam = svc.vision.cams[cam_id].model
    jog = JOG_AXES[cam_id]                    # ((axis, rel_sign), ...)
    axes = [a for a, _s in jog]
    _check_marks(svc, cam_id, cam)
    for axis in axes:                         # 首步运动前记录原始位
        axes_touched.append((axis, svc._axis_pos(axis)))
    _log(svc, job, "camera microrot: cam=%s template=%s repeat=%d "
         "wait_ms=%s pos1=%s pos2=%s axes=%s"
         % (cam_id, tpl, repeat, wait_ms, photo_pos1, photo_pos2, axes))

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
        # 减观测角；投影链 R(−θ_t) 下旧式得 −θ_t，末端翻号使输出=θ_t
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
                   "photo_pos1": [round(v, 6) for v in photo_pos1],
                   "photo_pos2": [round(v, 6) for v in photo_pos2],
                   "span_mm": round(abs(photo_pos2[0] - photo_pos1[0]), 6),
                   "nm_per_px": round(k_nm, 4),
                   "note": "逐轮 φ=−atan2(Δrow,Δcol) 经相机 dir/swap 与"
                           "运动方向映射到真值约定（迭代 39：输出=物理角"
                           "真值估计=建议写入 rot_mrad 的认知值，映射末端"
                           "翻号）"},
        "context": None,
        "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
