# -*- coding: utf-8 -*-
"""
Filename: BondForceCalib.py
Description:
    键合力校准（界面三段式对照 AsHeadCenterCalib）。核心链路在后端作业
    bond_force_calib（cam 恒 up：晶圆已上片、键合头已拾取芯片后按
    setpoints_n 逐点真实下压，每点采 samples×sample_interval_ms 力序
    列求 mean/3σ，线性拟合 gain/offset_n），本页走 job_base 的"参数
    下发 + 作业轮询 + 结果展示"。

    前提：晶圆已上片、键合头已拾取芯片（页面顶部橙底醒目提示；校准
    将真实下压）。

    参数：setpoints_n 文本（默认 "5,10,20,30"）+ samples（默认 20）
    + sample_interval_ms（默认 100）。

    结果：measured={gain, offset_n, points:[{setpoint_n, mean_n, s3_n,
    n}]}；current={force_gain, force_offset_n}；series。结果区：
    mean_n vs setpoint_n 散点 + 拟合直线（标注 gain/offset）+ 统计表
    （每 setpoint 一行 mean/3σ + gain/offset_n measured vs current）
    + "写回力校准"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["force_gain","force_offset_n"]，
    照 AsHeadCenterCalib 的写回模式）。
"""
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import BondForceCalib_ui as ui

# 写回字段
APPLY_FIELDS = ["force_gain", "force_offset_n"]

# 统计表最大行数（= setpoint 上限 + gain/offset 两行）
_MAX_SETPOINT_ROWS = 8


class BondForceCalib(JobRoutineBase):
    ROUTINE = "bond_force_calib"
    TITLE = "键合力校准"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None

        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax = self.res_fig.add_subplot(111)      # 力散点+拟合
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.btn_apply.clicked.connect(self._on_apply_clicked)
        self.sig_apply_done.connect(self._on_apply_done)
        self.bind_run_controls()

    # ---------------- 参数 ----------------
    @staticmethod
    def _parse_setpoints(text):
        """逗号/空白分隔的力设定点文本 → float list；空或含非数值报错。"""
        parts = [p for p in (text or "").replace("，", ",")
                 .replace(" ", ",").split(",") if p.strip()]
        if not parts:
            raise ValueError("力设定点 setpoints 必填（逗号分隔，如 "
                             "5,10,20,30）")
        try:
            return [float(p) for p in parts]
        except ValueError:
            raise ValueError("力设定点 setpoints 含非数值：%r" % text)

    def get_saved_param(self):
        INPUT = {
            'setpoints_n': self.ui.setpoints.text().strip(),
            'samples': int(self.ui.samples.value()),
            'sample_interval_ms': float(self.ui.sample_interval.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.setpoints.setText(param_dict.get(
            'setpoints_n', ui.DEFAULT_SETPOINTS_N))
        self.ui.samples.setValue(int(param_dict.get('samples', 20)))
        self.ui.sample_interval.setValue(
            float(param_dict.get('sample_interval_ms', 100)))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "up"

    def build_params(self, dict_input):
        return {"setpoints_n": self._parse_setpoints(
                    dict_input['setpoints_n']),
                "samples": int(dict_input['samples']),
                "sample_interval_ms": dict_input['sample_interval_ms']}

    # ---------------- 结果解读（GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (gain, offset_n, points, current)：points 只保留 dict 项。"""
        res = res or {}
        measured = res.get("measured") or {}
        points = [p for p in (measured.get("points") or [])
                  if isinstance(p, dict)]
        return (measured.get("gain"), measured.get("offset_n"), points,
                res.get("current") or {})

    def result_log_lines(self, res):
        gain, offset_n, points, _cur = self._normalize(res)
        return ["gain=%s, offset_n=%s（N=%d 力设定点）"
                % (_fmt(gain, 6), _fmt(offset_n, 4), len(points))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        gain, offset_n, points, current = self._normalize(res)
        # 散点 mean_n vs setpoint_n + 拟合直线（标注 gain/offset）
        self.res_ax.clear()
        xs = [float(p["setpoint_n"]) for p in points
              if p.get("setpoint_n") is not None
              and p.get("mean_n") is not None]
        ys = [float(p["mean_n"]) for p in points
              if p.get("setpoint_n") is not None
              and p.get("mean_n") is not None]
        if xs and ys:
            self.res_ax.plot(xs, ys, "o", color="#4a90e2", markersize=7,
                             label="mean_n")
        if gain is not None and offset_n is not None and xs:
            g, b = float(gain), float(offset_n)
            x0, x1 = min(xs), max(xs)
            self.res_ax.plot([x0, x1], [g * x0 + b, g * x1 + b], "-",
                             color="#c0392b", linewidth=1.2,
                             label="fit: y=%s·x+%s"
                                   % (_fmt(g, 6), _fmt(b, 4)))
        if xs:
            self.res_ax.legend(fontsize=7, loc="best")
        self.res_ax.set_title("force calib: gain=%s, offset=%s N"
                              % (_fmt(gain, 6), _fmt(offset_n, 4)),
                              fontsize=9)
        self.res_ax.set_xlabel("setpoint (N)", fontsize=8)
        self.res_ax.set_ylabel("measured mean (N)", fontsize=8)
        self.res_ax.tick_params(labelsize=8)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表：每 setpoint 一行 mean/3σ + gain/offset_n（measured vs
        # current）；current 键 force_gain/force_offset_n，缺省 —
        table = self.ui.stats_table
        row = 0
        for p in points[:_MAX_SETPOINT_ROWS]:
            sp = p.get("setpoint_n")
            cell = "mean=%s N, 3σ=%s N" % (_fmt(p.get("mean_n"), 3),
                                           _fmt(p.get("s3_n"), 3))
            fill_stats_row(table, row,
                           ["setpoint %s N" % _fmt(sp, 1), cell, None])
            row += 1
        fill_stats_row(table, row, ["gain", _fmt(gain, 6),
                                    _fmt(current.get("force_gain"), 6)])
        row += 1
        fill_stats_row(table, row, ["offset_n", _fmt(offset_n, 4),
                                    _fmt(current.get("force_offset_n"), 4)])
        row += 1
        while row < table.rowCount():      # 余行清空（复跑少点时不留旧值）
            fill_stats_row(table, row, [None, None, None])
            row += 1

    # ---------------- 写回力校准（作业 done 后可用） ----------------
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
                "作业 #%d 已完成，可写回 force_gain/force_offset_n" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="bond-force-apply", daemon=True).start()

    def _apply_worker(self, jid):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid, "fields": list(APPLY_FIELDS)},
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
    test = BondForceCalib()
    test.show()
    sys.exit(myapp.exec_())
