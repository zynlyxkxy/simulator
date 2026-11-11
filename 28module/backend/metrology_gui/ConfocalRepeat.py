# -*- coding: utf-8 -*-
"""
Filename: ConfocalRepeat.py
Description:
    光谱共焦重复性（对照参考框架 SpectralConfocalStability 的测法：
    固定机位连读共焦读数统计 3σ；对照 SingleCameraAlignStability 的
    界面三段式）。核心链路在后端作业 confocal_repeat（读数无效按
    测校失败处理，见 metrology/extra_routines.py），本页走
    job_base 的"参数下发 + 作业轮询 + 结果展示"。

    结果区：逐次读数 µm 趋势曲线（标题 3σ µm）+ 统计表
    （N / mean / std / 3σ，单位 µm）。
"""
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase
from qt_custom_ui import fill_stats_row
import ConfocalRepeat_ui as ui


class ConfocalRepeat(JobRoutineBase):
    ROUTINE = "confocal_repeat"
    TITLE = "光谱共焦重复性"

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)

        self.res_fig = Figure()
        self.res_ax = self.res_fig.add_subplot(111)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.bind_run_controls()

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        INPUT = {
            'probe': self.ui.cb_probe.currentText(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_probe.setCurrentText(param_dict['probe'])
        self.ui.repeat_times.setValue(param_dict['repeat_time'])
        self.ui.wait_time.setValue(param_dict['wait_time'])

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "down" if "下" in dict_input['probe'] else "up"

    def build_params(self, dict_input):
        return {"repeat": int(dict_input['repeat_time']),
                "wait_ms": dict_input['wait_time']}

    def result_log_lines(self, res):
        st = (res.get("stats") or {}).get("reading") or {}
        return ["读数 3σ = %s µm（mean=%s µm, std=%s µm）"
                % (st.get("s3_um"), st.get("mean_um"), st.get("std_um"))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        series = ((res or {}).get("series") or {}).get("reading_um") or []
        st = ((res or {}).get("stats") or {}).get("reading") or {}
        self.res_ax.clear()
        self.res_ax.plot(list(range(len(series))), series, ".-")
        s3 = st.get("s3_um")
        self.res_ax.set_title("reading: %s µm (3σ)"
                              % ("%.4f" % s3 if s3 is not None else "—"))
        self.res_ax.set_xlabel("index")
        self.res_ax.set_ylabel("µm")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        fill_stats_row(self.ui.stats_table, 0, [
            len(series),
            _fmt(st.get("mean_um")),
            _fmt(st.get("std_um")),
            _fmt(st.get("s3_um"))])


def _fmt(v, nd=4):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = ConfocalRepeat()
    test.show()
    sys.exit(myapp.exec_())
