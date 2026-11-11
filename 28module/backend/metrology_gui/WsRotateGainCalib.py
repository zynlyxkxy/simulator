# -*- coding: utf-8 -*-
"""
Filename: WsRotateGainCalib.py
Description:
    旋转角度校准（WS_Rz 增益标定；迭代 32 新测校项；界面三段式对照
    CameraMicrorotCalib/WsRotateCenterCalib）。核心链路在后端作业
    ws_rotate_gain（cam=down：示教点位上 WS_Rz 按 step_rad×n_angles
    ±对称档步进拍照，实测角/指令角直线拟合增益），本页走 job_base 的
    "参数下发 + 作业轮询 + 结果展示"。

    参数：示教点表格（taught_points_table 共用行布局组件，2~10 点，
    迭代 35 起每点两位姿 {tpl, pose{AS_X,AS_Y,WS_LS_X,WS_LS_Y},
    pose_after{同结构可选}}：旋前走遍各点 pose 求 P_k，旋转后走遍
    pose_after 求 Q_k，后端按 find_rotation_center 口径拟合 θ 与中心；
    pose_after 缺省=pose（小角度），旋后位用于防止大角度旋转后标记出
    视场；读旧存档（无 pose_after）时旋后四格自动填旋前值。模板名
    必填、须已在视觉页注册，各配"→旋前"/"→旋后"取当前位按钮）
    + step_rad（(0,0.3]，默认 0.01）+ n_angles（[2,8]，默认 3）+
    repeat（[3,100]，默认 10）+ wait_ms。

    结果：measured.rz_gain（≈1.0）+ current.rz_gain +
    stats.gain{mean,std,s3} + stats.offset_urad{mean,std,s3} +
    stats.gain_intercept（诊断）+ 逐 repeat series.gain。结果区：
    gain 大数字（9 位小数，附当前值/Δ）+ 逐 repeat gain 趋势（标题
    mean/3σ）+ 统计表（gain 三行 + offset_urad 三行 + intercept
    诊断行）+ "写回 rz_gain"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["rz_gain"]）。
"""
import math
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos, http_json
from qt_custom_ui import fill_stats_row
import WsRotateGainCalib_ui as ui

# 示教位控件 attr → 后端契约轴名；"*_aft" 后缀=旋后位姿（迭代 35，
# 同旋转重复性/中心标定）
_AXIS_OF_ATTR = {"as_x": "AS_X", "as_y": "AS_Y",
                 "ws_x": "WS_LS_X", "ws_y": "WS_LS_Y"}
_AXIS_OF_ATTR_AFT = {a + "_aft": ax for a, ax in _AXIS_OF_ATTR.items()}


class WsRotateGainCalib(JobRoutineBase):
    ROUTINE = "ws_rotate_gain"
    TITLE = "旋转角度校准"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None
        self._wire_point_groups()

        # 迭代 35：figure 收窄（默认 6.4in 过宽），给 8 列示教表让宽
        self.res_fig = Figure(figsize=(4.2, 3.2))
        self.res_ax = self.res_fig.add_subplot(111)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.n_points.valueChanged.connect(self._on_n_points_changed)
        self.ui.btn_apply.clicked.connect(self._on_apply_clicked)
        self.sig_apply_done.connect(self._on_apply_done)
        self.bind_run_controls()

    # ---------------- 示教点行动态增删（GUI 线程） ----------------
    def _wire_point_groups(self):
        for i, g in enumerate(self.ui.point_groups):
            g["btn_snap_before"].clicked.connect(
                lambda _=False, idx=i: self._snap_pose(idx, False))
            g["btn_snap_after"].clicked.connect(
                lambda _=False, idx=i: self._snap_pose(idx, True))

    def _on_n_points_changed(self, _v):
        n = int(self.ui.n_points.value())
        if n == len(self.ui.point_groups):
            return
        self.ui.rebuild_points(n)
        self._wire_point_groups()
        self.ui.textEdit.append("示教点数调整为 %d（已填值按下标保留）" % n)

    # ---------------- 取当前位（GUI 线程） ----------------
    def _snap_pose(self, idx, after=False):
        if idx >= len(self.ui.point_groups):
            return
        try:
            pos = fetch_axes_pos(timeout=5)
        except Exception as e:
            self.ui.textEdit.append("[warn] 取当前位失败：%s" % e)
            return
        spins = self.ui.point_groups[idx]["spins"]
        axis_map = _AXIS_OF_ATTR_AFT if after else _AXIS_OF_ATTR
        for attr, ax in axis_map.items():
            v = pos.get(ax)
            if v is not None:
                spins[attr].setValue(float(v))
        self.ui.textEdit.append("示教点 #%d 已回填%s当前位：%s"
                                % (idx + 1, "旋后" if after else "旋前",
                                   {a: pos.get(ax) for a, ax in
                                    axis_map.items()}))

    # ---------------- 参数 ----------------
    @staticmethod
    def _read_pose(spins, after=False):
        axis_map = _AXIS_OF_ATTR_AFT if after else _AXIS_OF_ATTR
        return {axis: float(spins[attr].value())
                for attr, axis in axis_map.items()}

    def get_saved_param(self):
        points = []
        for i, g in enumerate(self.ui.point_groups):
            tpl = g["tpl"].text().strip()
            if not tpl:
                raise ValueError("示教点 #%d 模板名必填（模板须已在视觉页"
                                 "注册）" % (i + 1))
            points.append({"tpl": tpl,
                           "pose": self._read_pose(g["spins"]),
                           "pose_after": self._read_pose(g["spins"],
                                                         after=True)})
        INPUT = {
            'n_points': len(points),
            'points': points,
            'step_rad': float(self.ui.step_rad.value()),
            'n_angles': int(self.ui.n_angles.value()),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        points = param_dict['points']
        n = int(param_dict.get('n_points') or len(points))
        self.ui.n_points.setValue(n)
        if len(self.ui.point_groups) != len(points):
            self.ui.rebuild_points(len(points))
            self._wire_point_groups()
        for g, pt in zip(self.ui.point_groups, points):
            g["tpl"].setText(pt.get("tpl", ""))
            pose = pt.get("pose") or {}
            # 旧档兼容（迭代 32~34）：无 pose_after → 旋后四格自动填旋前值
            pose_after = pt.get("pose_after") or pose
            for attr, axis in _AXIS_OF_ATTR.items():
                if pose.get(axis) is not None:
                    g["spins"][attr].setValue(float(pose[axis]))
                if pose_after.get(axis) is not None:
                    g["spins"][attr + "_aft"].setValue(
                        float(pose_after[axis]))
        self.ui.step_rad.setValue(float(param_dict.get('step_rad', 0.01)))
        self.ui.n_angles.setValue(int(param_dict.get('n_angles', 3)))
        self.ui.repeat_times.setValue(int(param_dict['repeat_time']))
        self.ui.wait_time.setValue(float(param_dict['wait_time']))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "down"

    def build_params(self, dict_input):
        return {"taught_points": [
                    {"tpl": pt["tpl"], "pose": dict(pt["pose"]),
                     # pose_after 缺省=pose（旧档经 set_saved_param 已回填，
                     # 此处再兜底，保证下发契约带双位姿）
                     "pose_after": dict(pt.get("pose_after") or pt["pose"])}
                    for pt in dict_input['points']],
                "step_rad": dict_input['step_rad'],
                "n_angles": int(dict_input['n_angles']),
                "repeat": int(dict_input['repeat_time']),
                "wait_ms": dict_input['wait_time']}

    # ---------------- 结果解读（多 schema 归一，GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (gain_meas, gain_cur, g_mean, g_std, g_s3,
        o_mean, o_std, o_s3, intercept, gain_ser)：
        gain_meas = measured.rz_gain（缺省回落 stats.gain.mean）；
        gain_cur = current.rz_gain；g_* = stats.gain{mean,std,s3}
        （或扁平 gain_mean/gain_std/gain_s3，缺省由 series.gain 推算）；
        o_* = stats.offset_urad{mean,std,s3}（或扁平 offset_*_urad，
        缺省由 series.offset_urad 推算）；intercept = stats.gain_intercept
        （诊断标量）。"""
        res = res or {}
        measured = res.get("measured") or {}
        current = res.get("current") or {}
        st = res.get("stats") or {}
        series = res.get("series") or {}
        gain_meas = measured.get("rz_gain")
        if gain_meas is None:
            gain_meas = measured.get("gain")
        gain_cur = current.get("rz_gain")
        if gain_cur is None:
            gain_cur = current.get("gain")
        g = st.get("gain") or {}
        g_mean = g.get("mean")
        if g_mean is None:
            g_mean = st.get("gain_mean")
        g_std = g.get("std")
        if g_std is None:
            g_std = st.get("gain_std")
        g_s3 = g.get("s3")
        if g_s3 is None:
            g_s3 = st.get("gain_s3")
        gain_ser = [float(v) for v in (series.get("gain") or [])]
        # 兜底：由序列推算（总体口径 ddof=0，与后端统计惯例一致）
        if gain_ser and (g_mean is None or g_std is None):
            n = len(gain_ser)
            m = sum(gain_ser) / n
            var = sum((v - m) ** 2 for v in gain_ser) / n
            if g_mean is None:
                g_mean = m
            if g_std is None:
                g_std = math.sqrt(var)
        if g_s3 is None and g_std is not None:
            g_s3 = 3.0 * g_std
        if gain_meas is None:
            gain_meas = g_mean
        o = st.get("offset_urad") or {}
        o_mean = o.get("mean")
        if o_mean is None:
            o_mean = st.get("offset_mean_urad")
        o_std = o.get("std")
        if o_std is None:
            o_std = st.get("offset_std_urad")
        o_s3 = o.get("s3")
        if o_s3 is None:
            o_s3 = st.get("offset_s3_urad")
        off_ser = [float(v) for v in (series.get("offset_urad") or [])]
        if off_ser and (o_mean is None or o_std is None):
            n = len(off_ser)
            m = sum(off_ser) / n
            var = sum((v - m) ** 2 for v in off_ser) / n
            if o_mean is None:
                o_mean = m
            if o_std is None:
                o_std = math.sqrt(var)
        if o_s3 is None and o_std is not None:
            o_s3 = 3.0 * o_std
        # gain_intercept：后端契约是 {mean,std,s3} 字典（诊断统计），
        # 展示取 mean 标量；旧扁平标量兼容
        icp = st.get("gain_intercept")
        if isinstance(icp, dict):
            icp = icp.get("mean")
        return (gain_meas, gain_cur, g_mean, g_std, g_s3,
                o_mean, o_std, o_s3, icp, gain_ser)

    def result_log_lines(self, res):
        (gm, _gc, g_mean, g_std, g_s3,
         o_mean, _osd, o_s3, icp, gain_ser) = self._normalize(res)
        return ["rz_gain = %s（mean = %s, std = %s, 3σ = %s；offset "
                "mean = %s µrad, 3σ = %s µrad, intercept = %s, N = %d）"
                % (_fmt(gm, 9), _fmt(g_mean, 9), _fmt(g_std, 9),
                   _fmt(g_s3, 9), _fmt(o_mean, 3), _fmt(o_s3, 3),
                   _fmt(icp, 9), len(gain_ser))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        (gm, gc, g_mean, g_std, g_s3,
         o_mean, o_std, o_s3, icp, gain_ser) = self._normalize(res)
        # gain 大数字（9 位小数，附当前值/Δ）
        if gm is not None:
            txt = "rz_gain = %.9f" % gm
            if gc is not None:
                txt += "（当前 %.9f，Δ = %+.9f）" % (gc, gm - gc)
            self.ui.lbl_gain.setText(txt)
        else:
            self.ui.lbl_gain.setText("rz_gain：—（结果缺 "
                                     "measured.rz_gain）")
        # 逐 repeat gain 趋势（标题 mean/3σ）
        self.res_ax.clear()
        if gain_ser:
            self.res_ax.plot(list(range(len(gain_ser))), gain_ser, ".-",
                             color="#4a90e2", markersize=4)
        self.res_ax.set_title("per-repeat gain: mean %s / 3σ %s"
                              % (_fmt(g_mean, 9), _fmt(g_s3, 9)),
                              fontsize=10)
        self.res_ax.set_xlabel("repeat")
        self.res_ax.set_ylabel("gain")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表：gain mean/std/3σ + offset_urad 统计行 + intercept 诊断行
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["rz_gain (mean)", _fmt(g_mean, 9)])
        fill_stats_row(table, 1, ["gain std", _fmt(g_std, 9)])
        fill_stats_row(table, 2, ["gain 3σ", _fmt(g_s3, 9)])
        fill_stats_row(table, 3, ["offset mean (µrad)", _fmt(o_mean, 3)])
        fill_stats_row(table, 4, ["offset std (µrad)", _fmt(o_std, 3)])
        fill_stats_row(table, 5, ["offset 3σ (µrad)", _fmt(o_s3, 3)])
        fill_stats_row(table, 6, ["gain_intercept (诊断)", _fmt(icp, 9)])

    # ---------------- 写回 rz_gain（作业 done 后可用） ----------------
    def _on_run_clicked(self):
        self._apply_job_id = None
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("（作业完成后可写回机器常数）")
        super()._on_run_clicked()

    def _on_finish(self, output):
        super()._on_finish(output)
        jid = (output or {}).get("job_id")
        if jid:
            self._apply_job_id = jid
            self.ui.btn_apply.setEnabled(True)
            self.ui.lbl_apply.setText(
                "作业 #%d 已完成，可写回 rz_gain" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="rz-gain-apply", daemon=True).start()

    def _apply_worker(self, jid):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid, "fields": ["rz_gain"]},
                          timeout=15)
            if not r.get("ok"):
                raise RuntimeError("%s" % (r.get("error") or r))
            applied = r.get("applied") or []
            msg = "已写回 %s（落盘+热生效）" % ", ".join(applied)
            self.sig_apply_done.emit("作业 #%d %s" % (jid, msg))
        except Exception as e:
            self.sig_apply_done.emit("写回失败：%s" % e)

    def _on_apply_done(self, msg):
        self.ui.lbl_apply.setText(msg)
        self.ui.textEdit.append(msg)
        # 失败时允许重试（成功则 _apply_job_id 保留但幂等无妨，
        # 按钮保持禁用防重复写回）
        if msg.startswith("写回失败"):
            self.ui.btn_apply.setEnabled(True)


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = WsRotateGainCalib()
    test.show()
    sys.exit(myapp.exec_())
