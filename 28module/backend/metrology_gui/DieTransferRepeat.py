# -*- coding: utf-8 -*-
"""
Filename: DieTransferRepeat.py
Description:
    芯片交接重复性（设计草稿"蓝膜台→顶针→Flip→键合臂 分段测量"；
    界面三段式对照 SingleCameraAlignStability）。核心链路在后端作业
    die_transfer_repeat（作业首取片一次（拾取头有余片则沿用、跳过
    取片），每循环 交接到键合头→回翻；循环内不放回蓝膜，结束留片
    在拾取头（flip）不归位、下次作业沿用；分段拍照反算世界坐标统计
    3σ，见 metrology/extra_routines.py），本页走 job_base 的"参数
    下发 + 作业轮询 + 结果展示"。

    模板（迭代 58 补）：粗对准相机模板（coarse_tpl，flip/back 段 die
    背面特征）+ 上相机模板（up_tpl，head 段键合头上 die 角标）两字段，
    启动时从机器常数 vision_marks 预填当前默认；留空=后端走机器常数
    缺省，非空才下发；模板须已在视觉页注册（后端 start 存在性 400）。
    back_tpl 不下发（后端缺省=coarse_tpl——back 段拍同一 die 背面特征）。

    结果区（分段可视化）：
    - flip/head/back 三段各自逐循环世界坐标偏差（µm，相对各自均值）
      趋势（X/Y 两个子图，三段各一条线）；
    - 三段 3σ(µm) 对比柱状图（X/Y 分组）；
    - 分段统计表（X 3σ / Y 3σ µm + col 3σ px）。
    世界坐标序列缺失时（旧后端字段）回退画 col/row px 趋势。
"""
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import DieTransferRepeat_ui as ui

SEG_CN = {"flip": "翻转段", "head": "键合头段", "back": "回翻段"}
SEG_ORDER = ("flip", "head", "back")
# 世界坐标序列字段候选（µm 直出 / mm 换算；新后端分段世界序列以实际
# 字段为准，逐个候选探测）
_WORLD_KEYS = (("world_x_um", "world_y_um", 1.0),
               ("world_x_mm", "world_y_mm", 1000.0),
               ("x_um", "y_um", 1.0),
               ("x_mm", "y_mm", 1000.0))


class DieTransferRepeat(JobRoutineBase):
    ROUTINE = "die_transfer_repeat"
    TITLE = "芯片交接重复性"

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)

        self.res_fig = Figure()
        self.res_ax_x = self.res_fig.add_subplot(311)
        self.res_ax_y = self.res_fig.add_subplot(312)
        self.res_ax_bar = self.res_fig.add_subplot(313)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.bind_run_controls()
        self._prefill_templates()

    def _prefill_templates(self):
        """启动时从机器常数 vision_marks 预填两台相机模板名（后端离线则
        留空不报错——build_params 留空=后端走同一机器常数缺省，语义一致）。"""
        try:
            body = http_json("GET", "/api/machine_constants", timeout=5)
            vmarks = (body.get("constants") or {}).get("vision_marks") or {}
            if vmarks.get("tpl_coarse_die"):
                self.ui.cb_tpl_coarse.setText(str(vmarks["tpl_coarse_die"]))
            if vmarks.get("tpl_die_head"):
                self.ui.cb_tpl_up.setText(str(vmarks["tpl_die_head"]))
        except Exception:
            pass   # 后端离线：留空，运行时后端按机器常数缺省

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        INPUT = {
            'tpl_coarse': self.ui.cb_tpl_coarse.text().strip(),
            'tpl_up': self.ui.cb_tpl_up.text().strip(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_tpl_coarse.setText(param_dict.get('tpl_coarse', ''))
        self.ui.cb_tpl_up.setText(param_dict.get('tpl_up', ''))
        self.ui.repeat_times.setValue(param_dict['repeat_time'])
        self.ui.wait_time.setValue(param_dict['wait_time'])

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "coarse"

    def build_params(self, dict_input):
        # 模板非空才下发（留空=后端走机器常数 vision_marks 缺省，calibration
        # .start 统一做存在性 400 校验）；back_tpl 不下发——后端缺省=coarse_tpl
        # （back 段拍同一 die 背面同一特征，同模板即可）
        p = {"repeat": int(dict_input['repeat_time']),
             "wait_ms": dict_input['wait_time']}
        if dict_input.get('tpl_coarse'):
            p["coarse_tpl"] = dict_input['tpl_coarse']
        if dict_input.get('tpl_up'):
            p["up_tpl"] = dict_input['tpl_up']
        return p

    def result_log_lines(self, res):
        st = res.get("stats") or {}
        lines = []
        for tag in SEG_ORDER:
            s = st.get(tag)
            if not s:
                continue
            lines.append("%s: 世界 3σ = (X %s, Y %s) µm, col 3σ = %s px"
                         % (SEG_CN[tag], s.get("world_x_s3_um"),
                            s.get("world_y_s3_um"), s.get("col_s3_px")))
        return lines

    # ---------------- 结果可视化（GUI 线程） ----------------
    @staticmethod
    def _seg_series(res, tag):
        """取分段世界坐标序列（µm）；找不到世界字段回退 col/row（px）。
        返回 (xs, ys, unit)。"""
        seg = ((res.get("series") or {}).get(tag)) or {}
        for kx, ky, scale in _WORLD_KEYS:
            xs, ys = seg.get(kx), seg.get(ky)
            if xs and ys:
                return ([v * scale for v in xs], [v * scale for v in ys],
                        "µm")
        cols, rows_ = seg.get("col"), seg.get("row")
        if cols and rows_:
            return list(cols), list(rows_), "px"
        return [], [], "µm"

    def plot_realtime_data(self, res):
        res = res or {}
        st = res.get("stats") or {}
        tags = [t for t in SEG_ORDER if (res.get("series") or {}).get(t)
                or st.get(t)]
        self.res_ax_x.clear()
        self.res_ax_y.clear()
        self.res_ax_bar.clear()
        unit = "µm"
        bar_labels, bar_x, bar_y = [], [], []
        for tag in tags:
            xs, ys, unit = self._seg_series(res, tag)
            if xs:
                dx = np.array(xs) - np.mean(xs)
                dy = np.array(ys) - np.mean(ys)
                idx = list(range(len(dx)))
                self.res_ax_x.plot(idx, dx, ".-", label=tag)
                self.res_ax_y.plot(idx, dy, ".-", label=tag)
            s = st.get(tag) or {}
            # 柱状图取 stats 的世界 3σ（µm），与趋势序列单位解耦——
            # 旧后端序列只有 col/row px 时柱状图仍应出（此前被
            # unit=="µm" 门挡成空图）
            wx, wy = s.get("world_x_s3_um"), s.get("world_y_s3_um")
            if wx is not None or wy is not None:
                bar_labels.append(tag)
                bar_x.append(wx if wx is not None else 0.0)
                bar_y.append(wy if wy is not None else 0.0)
        self.res_ax_x.set_title("world X deviation (vs mean, %s)" % unit)
        self.res_ax_x.set_ylabel(unit)
        self.res_ax_x.legend(fontsize=7)
        self.res_ax_y.set_title("world Y deviation (vs mean, %s)" % unit)
        self.res_ax_y.set_ylabel(unit)
        self.res_ax_y.legend(fontsize=7)
        if bar_labels:
            xpos = np.arange(len(bar_labels))
            w = 0.35
            self.res_ax_bar.bar(xpos - w / 2, bar_x, w, label="X 3σ")
            self.res_ax_bar.bar(xpos + w / 2, bar_y, w, label="Y 3σ")
            self.res_ax_bar.set_xticks(xpos)
            self.res_ax_bar.set_xticklabels(bar_labels)
            self.res_ax_bar.set_title("segment 3σ (µm)")
            self.res_ax_bar.set_ylabel("µm")
            self.res_ax_bar.legend(fontsize=7)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 分段统计表
        for i, tag in enumerate(tags):
            if i >= self.ui.stats_table.rowCount():
                break
            s = st.get(tag) or {}
            fill_stats_row(self.ui.stats_table, i, [
                SEG_CN.get(tag, tag),
                _fmt(s.get("world_x_s3_um"), 3),
                _fmt(s.get("world_y_s3_um"), 3),
                _fmt(s.get("col_s3_px"), 4)])


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = DieTransferRepeat()
    test.show()
    sys.exit(myapp.exec_())
