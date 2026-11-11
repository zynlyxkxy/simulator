# -*- coding: utf-8 -*-
"""
Filename: AsHeadCenterCalib.py
Description:
    上吸盘位置标定（迭代 42 新测校项；界面三段式对照
    CameraMicrorotCalib）。核心链路在后端作业 as_head_center
    （cam 恒 up：前置走位后动态再逼近 identify，solve_pixel 转世界
    偏移求上相机认吸盘中心时的对准台轴位，见 metrology/
    as_head_center.py），本页走 job_base 的"参数下发 + 作业轮询 +
    结果展示"。

    参数：视觉模板名称（留空=后端缺省机器常数 vision_marks.
    tpl_head_chuck）+ repeat（默认 10）+ wait_ms。

    结果：measured.as_anchor_xy_mm / as_head_center_xy_mm=[x,y]（mm，
    双键同值联动——上/下相机同轴同一物理量）+ 逐 repeat 锚点散点
    （µm，相对均值）+ mean/3σ（nm）统计表 + "写回锚点"按钮（作业
    done 后可用，POST /api/calibration/apply fields=
    ["as_anchor_xy_mm","as_head_center_xy_mm"]，照
    WsRotateCenterCalib 的写回模式）。
"""
import math
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import AsHeadCenterCalib_ui as ui

# 写回字段（双键同值联动；apply 组路由 as_anchor_xy_mm→wafer_ega /
# as_head_center_xy_mm→focus_level）
APPLY_FIELDS = ["as_anchor_xy_mm", "as_head_center_xy_mm"]


class AsHeadCenterCalib(JobRoutineBase):
    ROUTINE = "as_head_center"
    TITLE = "上吸盘位置标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None

        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax = self.res_fig.add_subplot(111)    # 锚点散点（µm）
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
            'template_name': self.ui.cb_vision_template.text().strip(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_vision_template.setText(
            param_dict.get('template_name', ""))
        self.ui.repeat_times.setValue(int(param_dict.get('repeat_time', 10)))
        self.ui.wait_time.setValue(float(param_dict.get('wait_time', 10)))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "up"

    def build_params(self, dict_input):
        params = {"repeat": int(dict_input['repeat_time']),
                  "wait_ms": dict_input['wait_time']}
        tpl = dict_input.get('template_name') or ""
        if tpl:
            params["template_name"] = tpl   # 留空=后端缺省 vision_marks
        return params

    # ---------------- 结果解读（多 schema 归一，GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (ax_mm, ay_mm, xs_mm, ys_mm, s3_nm)：
        ax/ay = measured.as_anchor_xy_mm（mm）；xs/ys = 逐 repeat 锚点
        序列（mm，series.anchor_x_mm/anchor_y_mm）；s3_nm = 锚点 repeat
        3σ（nm，stats.anchor.s3_nm，缺省由序列径向散布推算）。"""
        res = res or {}
        measured = res.get("measured") or {}
        st = res.get("stats") or {}
        series = res.get("series") or {}
        ctr = (measured.get("as_anchor_xy_mm")
               or measured.get("as_head_center_xy_mm") or [])
        ax = float(ctr[0]) if len(ctr) > 0 and ctr[0] is not None else None
        ay = float(ctr[1]) if len(ctr) > 1 and ctr[1] is not None else None
        xs = [float(v) for v in (series.get("anchor_x_mm") or [])]
        ys = [float(v) for v in (series.get("anchor_y_mm") or [])]
        s3_nm = (st.get("anchor") or {}).get("s3_nm")
        if s3_nm is None and xs and ys:
            n = len(xs)
            mx, my = sum(xs) / n, sum(ys) / n
            var = sum((xs[i] - mx) ** 2 + (ys[i] - my) ** 2
                      for i in range(n)) / n
            s3_nm = 3.0 * math.sqrt(var) * 1e6
        return ax, ay, xs, ys, s3_nm

    def result_log_lines(self, res):
        ax, ay, xs, _y, s3_nm = self._normalize(res)
        return ["as_anchor_xy_mm = as_head_center_xy_mm = [%s, %s] mm"
                "（x %s µm, y %s µm）；锚点 repeat 3σ = %s nm, N = %d"
                % (_fmt(ax, 6), _fmt(ay, 6),
                   _fmt(None if ax is None else ax * 1e3, 3),
                   _fmt(None if ay is None else ay * 1e3, 3),
                   _fmt(s3_nm, 2), len(xs))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        ax, ay, xs, ys, s3_nm = self._normalize(res)
        # 锚点坐标大数字（mm 与 µm 双单位）
        if ax is not None and ay is not None:
            self.ui.lbl_anchor.setText(
                "锚点 x = %+.6f mm（%+.3f µm）　y = %+.6f mm（%+.3f µm）"
                % (ax, ax * 1e3, ay, ay * 1e3))
        else:
            self.ui.lbl_anchor.setText("锚点：—（结果缺 "
                                       "measured.as_anchor_xy_mm）")
        # 散点：逐 repeat 锚点相对均值（µm）+ 均值十字
        self.res_ax.clear()
        if xs and ys:
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            dxs = [(v - mx) * 1e3 for v in xs]
            dys = [(v - my) * 1e3 for v in ys]
            self.res_ax.plot(dxs, dys, ".", color="#4a90e2",
                             markersize=6, label="per-repeat")
            self.res_ax.plot([0.0], [0.0], "+", color="#c0392b",
                             markersize=12, markeredgewidth=2,
                             label="mean")
            self.res_ax.legend(fontsize=7, loc="best")
        self.res_ax.set_title("anchor scatter rel. mean (µm): "
                              "repeat 3σ %s nm" % _fmt(s3_nm, 2),
                              fontsize=9)
        self.res_ax.set_xlabel("Δx (µm)", fontsize=8)
        self.res_ax.set_ylabel("Δy (µm)", fontsize=8)
        self.res_ax.tick_params(labelsize=8)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["锚点 x (mm)", _fmt(ax, 6)])
        fill_stats_row(table, 1, ["锚点 y (mm)", _fmt(ay, 6)])
        fill_stats_row(table, 2, ["锚点 repeat 3σ (nm)", _fmt(s3_nm, 2)])
        st = (res or {}).get("stats") or {}
        anc = st.get("anchor") or {}
        fill_stats_row(table, 3, ["x 3σ (nm)",
                                  _fmt((anc.get("x") or {}).get("s3_nm"),
                                       2)])
        fill_stats_row(table, 4, ["y 3σ (nm)",
                                  _fmt((anc.get("y") or {}).get("s3_nm"),
                                       2)])

    # ---------------- 写回锚点（作业 done 后可用） ----------------
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
                "作业 #%d 已完成，可写回锚点（双键联动）" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="as-head-center-apply", daemon=True).start()

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
    test = AsHeadCenterCalib()
    test.show()
    sys.exit(myapp.exec_())
