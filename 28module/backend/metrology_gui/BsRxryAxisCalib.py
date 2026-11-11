# -*- coding: utf-8 -*-
"""
Filename: BsRxryAxisCalib.py
Description:
    键合头Rx/Ry旋转轴标定（界面三段式对照 AsHeadCenterCalib）。核心链路
    在后端作业 bs_rxry_axis（cam 恒 up：BS_Rx/BS_Ry 各按 ±angle_mrad
    摆动 neg/zero/pos 三位置，上相机追踪 4 个标记位移矢量，拟合旋转
    轴线），本页走 job_base 的"参数下发 + 作业轮询 + 结果展示"。

    参数：angle_mrad（默认 1.5）+ repeat（默认 3）+ wait_ms（默认 10）。

    结果：measured={rx_axis_point_mm, rx_axis_dir, ry_axis_point_mm,
    ry_axis_dir, rx_rmse_mm, ry_rmse_mm, angle_mrad}（向量为 3 元 list）；
    current 同 4 向量键；points={"RX":{"neg":[[x,y,z]×4],"zero":…,
    "pos":…}, "RY":{…}}。结果区：Rx/Ry 双 subplot（4 标记 neg→pos
    位移矢量 quiver，XY 投影 mm）+ 统计表（measured vs current）+
    "写回旋转轴"按钮（作业 done 后可用，POST /api/calibration/apply
    fields=["rx_axis_point_mm","rx_axis_dir","ry_axis_point_mm",
    "ry_axis_dir"]，照 AsHeadCenterCalib 的写回模式）。
"""
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import BsRxryAxisCalib_ui as ui

# 写回字段（轴点+轴方向，Rx/Ry 四条）
APPLY_FIELDS = ["rx_axis_point_mm", "rx_axis_dir",
                "ry_axis_point_mm", "ry_axis_dir"]


class BsRxryAxisCalib(JobRoutineBase):
    ROUTINE = "bs_rxry_axis"
    TITLE = "键合头Rx/Ry旋转轴标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None

        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax_rx = self.res_fig.add_subplot(121)   # Rx 位移矢量
        self.res_ax_ry = self.res_fig.add_subplot(122)   # Ry 位移矢量
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
            'angle_mrad': float(self.ui.angle_mrad.value()),
            'repeat': int(self.ui.repeat_times.value()),
            'wait_ms': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.angle_mrad.setValue(float(param_dict.get('angle_mrad', 1.5)))
        self.ui.repeat_times.setValue(int(param_dict.get('repeat', 3)))
        self.ui.wait_time.setValue(float(param_dict.get('wait_ms', 10)))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "up"

    def build_params(self, dict_input):
        return {"angle_mrad": float(dict_input['angle_mrad']),
                "repeat": int(dict_input['repeat']),
                "wait_ms": dict_input['wait_ms']}

    # ---------------- 结果解读（GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (measured, current, points)：measured/current 缺省 {}；
        points 只保留 RX/RY 下 neg/zero/pos 为 [[x,y,z],…] 的键。"""
        res = res or {}
        measured = res.get("measured") or {}
        current = res.get("current") or {}
        points = res.get("points") or {}
        return measured, current, points

    def result_log_lines(self, res):
        measured, _cur, _pts = self._normalize(res)
        return ["rx_axis_point_mm=%s, rx_axis_dir=%s, rx_rmse_mm=%s"
                % (_vec(measured.get("rx_axis_point_mm")),
                   _vec(measured.get("rx_axis_dir")),
                   _fmt(measured.get("rx_rmse_mm"), 6)),
                "ry_axis_point_mm=%s, ry_axis_dir=%s, ry_rmse_mm=%s"
                % (_vec(measured.get("ry_axis_point_mm")),
                   _vec(measured.get("ry_axis_dir")),
                   _fmt(measured.get("ry_rmse_mm"), 6))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        measured, current, points = self._normalize(res)
        # Rx/Ry 双 subplot：4 标记 neg→pos 位移矢量 quiver（XY 投影 mm）
        for ax, key, name in ((self.res_ax_rx, "RX", "Rx"),
                              (self.res_ax_ry, "RY", "Ry")):
            ax.clear()
            grp = points.get(key) or {}
            neg = [p for p in (grp.get("neg") or [])
                   if isinstance(p, (list, tuple)) and len(p) >= 2]
            pos = [p for p in (grp.get("pos") or [])
                   if isinstance(p, (list, tuple)) and len(p) >= 2]
            zero = [p for p in (grp.get("zero") or [])
                    if isinstance(p, (list, tuple)) and len(p) >= 2]
            n = min(len(neg), len(pos))
            if zero:
                ax.plot([p[0] for p in zero], [p[1] for p in zero], "x",
                        color="#27ae60", markersize=7, label="zero")
            if n:
                ax.plot([p[0] for p in neg[:n]], [p[1] for p in neg[:n]],
                        "o", color="#4a90e2", markersize=5, label="neg")
                ax.plot([p[0] for p in pos[:n]], [p[1] for p in pos[:n]],
                        "s", color="#e67e22", markersize=5, label="pos")
                ax.quiver([p[0] for p in neg[:n]], [p[1] for p in neg[:n]],
                          [pos[i][0] - neg[i][0] for i in range(n)],
                          [pos[i][1] - neg[i][1] for i in range(n)],
                          angles="xy", scale_units="xy", scale=1,
                          color="#c0392b", width=0.006,
                          label="neg→pos")
            if zero or n:
                ax.legend(fontsize=7, loc="best")
            rmse = measured.get("%s_rmse_mm" % key.lower())
            ax.set_title("%s neg->pos (mm): RMSE %s"
                         % (name, _fmt(rmse, 6)), fontsize=9)
            ax.set_xlabel("x (mm)", fontsize=8)
            ax.set_ylabel("y (mm)", fontsize=8)
            ax.tick_params(labelsize=8)
            ax.set_aspect("equal", adjustable="datalim")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表（measured vs current；RMSE 无 current 给 —）
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["rx_axis_point_mm",
                                  _vec(measured.get("rx_axis_point_mm")),
                                  _vec(current.get("rx_axis_point_mm"))])
        fill_stats_row(table, 1, ["rx_axis_dir",
                                  _vec(measured.get("rx_axis_dir")),
                                  _vec(current.get("rx_axis_dir"))])
        fill_stats_row(table, 2, ["rx_rmse_mm",
                                  _fmt(measured.get("rx_rmse_mm"), 6), None])
        fill_stats_row(table, 3, ["ry_axis_point_mm",
                                  _vec(measured.get("ry_axis_point_mm")),
                                  _vec(current.get("ry_axis_point_mm"))])
        fill_stats_row(table, 4, ["ry_axis_dir",
                                  _vec(measured.get("ry_axis_dir")),
                                  _vec(current.get("ry_axis_dir"))])
        fill_stats_row(table, 5, ["ry_rmse_mm",
                                  _fmt(measured.get("ry_rmse_mm"), 6), None])

    # ---------------- 写回旋转轴（作业 done 后可用） ----------------
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
                "作业 #%d 已完成，可写回旋转轴（Rx/Ry 四条）" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="bs-rxry-axis-apply", daemon=True).start()

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


def _vec(v, nd=6):
    """3 元向量格式化 "[x, y, z]"；缺省/非序列给 None（表格显示 —）。"""
    if not isinstance(v, (list, tuple)) or not v:
        return None
    return "[" + ", ".join("%.*f" % (nd, float(x)) for x in v) + "]"


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = BsRxryAxisCalib()
    test.show()
    sys.exit(myapp.exec_())
