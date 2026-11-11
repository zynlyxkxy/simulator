# -*- coding: utf-8 -*-
"""
Filename: BondHeightCalib.py
Description:
    键合头键合高度标定（界面三段式对照 AsHeadCenterCalib）。核心链路在
    后端作业 bond_height_calib（cam 恒 up：晶圆已上片、键合头已拾取
    芯片后低速下探晶圆面，力阈值判接触），本页走 job_base 的"参数下
    发 + 作业轮询 + 结果展示"。

    前提：晶圆已上片、键合头已拾取芯片（页面顶部橙底醒目提示；否则
    后端报错）。

    参数：chip_thickness_mm（默认 0.5）+ wafer_thickness_mm（默认
    1.0）+ force_threshold_n（默认 2.0）+ search_speed_mm_s（默认
    0.2）。

    结果：measured={head_z_contact_mm（裸值）, z_contact_mm,
    contact_force_n, ...}；series={t_s, z_mm, force_n}。结果区：力-
    位移曲线 + 接触点标注 + 统计表（裸接触位 head_z_contact_mm
    measured vs current / 实测接触位 z_contact_mm）+ "写回键合高度"
    按钮（作业 done 后可用，POST /api/calibration/apply fields=
    ["head_z_contact_mm"]，照 AsHeadCenterCalib 的写回模式）。
"""
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import BondHeightCalib_ui as ui

# 写回字段（裸接触位）
APPLY_FIELDS = ["head_z_contact_mm"]


class BondHeightCalib(JobRoutineBase):
    ROUTINE = "bond_height_calib"
    TITLE = "键合头键合高度标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None

        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax = self.res_fig.add_subplot(111)      # 力-位移曲线
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.btn_apply.clicked.connect(self._on_apply_clicked)
        self.sig_apply_done.connect(self._on_apply_done)
        self.bind_run_controls()

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        INPUT = {
            'chip_thickness_mm': float(self.ui.chip_thickness.value()),
            'wafer_thickness_mm': float(self.ui.wafer_thickness.value()),
            'force_threshold_n': float(self.ui.force_threshold.value()),
            'search_speed_mm_s': float(self.ui.search_speed.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.chip_thickness.setValue(
            float(param_dict.get('chip_thickness_mm', 0.5)))
        self.ui.wafer_thickness.setValue(
            float(param_dict.get('wafer_thickness_mm', 1.0)))
        self.ui.force_threshold.setValue(
            float(param_dict.get('force_threshold_n', 2.0)))
        self.ui.search_speed.setValue(
            float(param_dict.get('search_speed_mm_s', 0.2)))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "up"

    def build_params(self, dict_input):
        return {"chip_thickness_mm": float(dict_input['chip_thickness_mm']),
                "wafer_thickness_mm":
                    float(dict_input['wafer_thickness_mm']),
                "force_threshold_n": float(dict_input['force_threshold_n']),
                "search_speed_mm_s": float(dict_input['search_speed_mm_s'])}

    # ---------------- 结果解读（GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (measured, current, series)：缺省都给空容器。"""
        res = res or {}
        return (res.get("measured") or {}, res.get("current") or {},
                res.get("series") or {})

    def result_log_lines(self, res):
        measured, _cur, _series = self._normalize(res)
        return ["head_z_contact_mm=%s, z_contact_mm=%s, contact_force_n=%s"
                % (_fmt(measured.get("head_z_contact_mm"), 6),
                   _fmt(measured.get("z_contact_mm"), 6),
                   _fmt(measured.get("contact_force_n"), 3))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        measured, current, series = self._normalize(res)
        z_contact = measured.get("z_contact_mm")
        f_contact = measured.get("contact_force_n")
        # 力-位移曲线（x=z_mm, y=force_n）+ 接触点标注
        self.res_ax.clear()
        zs = [float(v) for v in (series.get("z_mm") or [])]
        fs = [float(v) for v in (series.get("force_n") or [])]
        if zs and fs:
            self.res_ax.plot(zs, fs, ".-", color="#4a90e2", markersize=3,
                             linewidth=1.0, label="force-z")
        if z_contact is not None and f_contact is not None:
            self.res_ax.plot([float(z_contact)], [float(f_contact)], "o",
                             color="#c0392b", markersize=9,
                             label="contact z=%s mm" % _fmt(z_contact, 5))
            self.res_ax.annotate("contact", (float(z_contact),
                                             float(f_contact)),
                                 textcoords="offset points", xytext=(8, 8),
                                 fontsize=8, color="#c0392b")
        if zs and fs:
            self.res_ax.legend(fontsize=7, loc="best")
        self.res_ax.set_title("force-displacement (contact %s N)"
                              % _fmt(f_contact, 3), fontsize=9)
        self.res_ax.set_xlabel("z (mm)", fontsize=8)
        self.res_ax.set_ylabel("force (N)", fontsize=8)
        self.res_ax.tick_params(labelsize=8)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表（裸接触位 measured vs current / 实测接触位）
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["裸接触位 head_z_contact_mm",
                                  _fmt(measured.get("head_z_contact_mm"), 6),
                                  _fmt(current.get("head_z_contact_mm"), 6)])
        fill_stats_row(table, 1, ["实测接触位 z_contact_mm",
                                  _fmt(z_contact, 6),
                                  _fmt(current.get("z_contact_mm"), 6)])

    # ---------------- 写回键合高度（作业 done 后可用） ----------------
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
                "作业 #%d 已完成，可写回 head_z_contact_mm" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="bond-height-apply", daemon=True).start()

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
    test = BondHeightCalib()
    test.show()
    sys.exit(myapp.exec_())
