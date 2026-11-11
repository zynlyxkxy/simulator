# -*- coding: utf-8 -*-
"""
Filename: BestFocusCalib.py
Description:
    相机最佳焦平面标定（迭代 42 新测校项；界面三段式对照
    CameraMicrorotCalib）。核心链路在后端作业 best_focus
    （cam=up/down：区间收缩法扫 z-score 曲线 + 末轮三点抛物线拟合
    得亚步距 z*，见 metrology/best_focus.py），本页走 job_base 的
    "参数下发 + 作业轮询 + 结果展示"。

    参数：相机下拉（up/down——down 扫 AS_Z / up 扫 BS_Arm_Z）+ 视觉
    模板名称（留空=后端缺省按相机取机器常数 vision_marks）+
    z_start/z_end/step_mm（文本框，留空=后端缺省焦面常数 ±0.3mm/步
    0.02）+ rounds（搜索轮数，默认 2）+ frames_per_point（每点帧数，
    默认 3）+ wait_ms。

    结果：z-score 曲线（各轮不同颜色 + 峰值竖线 + best_z 竖线）+
    best_z 大数字 + 统计表 + "写回焦面"按钮（作业 done 后可用，
    POST /api/calibration/apply——按相机 fields=
    ["as_z_chuck_focus_mm"]（down）/ ["head_z_upfocus_mm"]（up），
    照 WsRotateCenterCalib 的写回模式）。
"""
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, http_json
from qt_custom_ui import fill_stats_row
import BestFocusCalib_ui as ui

# 各轮 z-score 曲线配色（超出循环取模）
_ROUND_COLORS = ("#4a90e2", "#e67e22", "#27ae60", "#8e44ad", "#c0392b")
# 写回字段（按相机）：focus_level 组两焦面键
_APPLY_FIELD_OF_CAM = {"down": "as_z_chuck_focus_mm",
                       "up": "head_z_upfocus_mm"}


class BestFocusCalib(JobRoutineBase):
    ROUTINE = "best_focus"
    TITLE = "相机最佳焦平面标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None
        self._last_cam = None             # 末次作业相机（写回字段路由）

        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax = self.res_fig.add_subplot(111)    # z-score 曲线
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
    def _num_or_none(text, key):
        """文本框数值：留空 None（后端缺省）；非数值抛清晰错误。"""
        t = (text or "").strip()
        if not t:
            return None
        try:
            return float(t)
        except ValueError:
            raise ValueError("%s 须为数值或留空（留空=后端缺省）：%r"
                             % (key, t))

    def get_saved_param(self):
        INPUT = {
            'camera': self.ui.cb_camera.currentText(),
            'template_name': self.ui.cb_vision_template.text().strip(),
            'z_start': self._num_or_none(self.ui.z_start.text(), "z_start"),
            'z_end': self._num_or_none(self.ui.z_end.text(), "z_end"),
            'step_mm': self._num_or_none(self.ui.step_mm.text(), "step"),
            'rounds': int(self.ui.rounds.value()),
            'frames_per_point': int(self.ui.frames_per_point.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_camera.setCurrentText(param_dict['camera'])
        self.ui.cb_vision_template.setText(
            param_dict.get('template_name', ""))
        for key, le in (('z_start', self.ui.z_start),
                        ('z_end', self.ui.z_end),
                        ('step_mm', self.ui.step_mm)):
            v = param_dict.get(key)
            le.setText("" if v is None else str(v))
        self.ui.rounds.setValue(int(param_dict.get('rounds', 2)))
        self.ui.frames_per_point.setValue(
            int(param_dict.get('frames_per_point', 3)))
        self.ui.wait_time.setValue(float(param_dict.get('wait_time', 10)))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "up" if "上" in dict_input['camera'] else "down"

    def build_params(self, dict_input):
        params = {"rounds": int(dict_input['rounds']),
                  "frames_per_point": int(dict_input['frames_per_point']),
                  "wait_ms": dict_input['wait_time']}
        tpl = dict_input.get('template_name') or ""
        if tpl:
            params["template_name"] = tpl   # 留空=后端缺省 vision_marks
        for key in ('z_start', 'z_end', 'step_mm'):
            if dict_input.get(key) is not None:
                params[key] = dict_input[key]   # 留空=后端缺省窗口/step
        return params

    # ---------------- 结果解读（多 schema 归一，GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (best_z, rounds, const_key, fit, confidence)：
        best_z = series.best_z_mm（缺省 measured 首值）；rounds =
        series.rounds=[{round,step_mm,z_mm[],score[],peak_z_mm}]；
        const_key = measured 首键（写回字段）；fit = detail.fit；
        confidence = 末轮峰 score。"""
        res = res or {}
        series = res.get("series") or {}
        measured = res.get("measured") or {}
        det = res.get("detail") or {}
        rounds = series.get("rounds") or []
        best_z = series.get("best_z_mm")
        const_key = None
        for k in measured:
            const_key = k
            break
        if best_z is None and const_key is not None:
            best_z = measured.get(const_key)
        return (best_z, rounds, const_key, det.get("fit"),
                res.get("confidence"))

    def result_log_lines(self, res):
        best_z, rounds, const_key, fit, conf = self._normalize(res)
        return ["%s = %s mm（best_z；拟合 %s，置信 %s，轮数 %d）"
                % (const_key or "best_z", _fmt(best_z, 6),
                   fit or "—", _fmt(conf, 4), len(rounds))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        best_z, rounds, const_key, fit, conf = self._normalize(res)
        cam = (res or {}).get("camera")
        self._last_cam = cam
        # best_z 大数字（mm 与 µm 偏差双单位：相对 current 现值）
        cur = ((res or {}).get("current") or {}).get(const_key) \
            if const_key else None
        if best_z is not None:
            txt = "best_z = %+.6f mm" % best_z
            if cur is not None:
                txt += "（相对现值 %+.3f µm）" % ((best_z - cur) * 1e3)
            txt += "　→ 写回 %s" % (const_key or "—")
            self.ui.lbl_best_z.setText(txt)
        else:
            self.ui.lbl_best_z.setText("最佳焦面：—（结果缺 "
                                       "series.best_z_mm）")
        # z-score 曲线：各轮不同颜色 + 峰值竖线 + best_z 竖线
        self.res_ax.clear()
        for k, rd in enumerate(rounds):
            color = _ROUND_COLORS[k % len(_ROUND_COLORS)]
            zs, ss = rd.get("z_mm") or [], rd.get("score") or []
            if not zs:
                continue
            self.res_ax.plot(zs, ss, ".-", color=color, markersize=3,
                             linewidth=1.0,
                             label="round %d (step %s)" % (
                                 rd.get("round"),
                                 _fmt(rd.get("step_mm"), 4)))
            pk = rd.get("peak_z_mm")
            if pk is not None:
                self.res_ax.axvline(pk, color=color, linestyle="--",
                                    linewidth=0.8, alpha=0.6)
        if best_z is not None:
            self.res_ax.axvline(best_z, color="#c0392b", linestyle="-",
                                linewidth=1.2, alpha=0.9,
                                label="best_z %s" % _fmt(best_z, 5))
        if rounds:
            self.res_ax.legend(fontsize=7, loc="best")
        self.res_ax.set_title("z-score curve (%s, conf %s)"
                              % (fit or "—", _fmt(conf, 4)), fontsize=9)
        self.res_ax.set_xlabel("%s (mm)"
                               % (((res or {}).get("detail") or {})
                                  .get("axis") or "z"), fontsize=8)
        self.res_ax.set_ylabel("score", fontsize=8)
        self.res_ax.tick_params(labelsize=8)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["best_z (mm)", _fmt(best_z, 6)])
        fill_stats_row(table, 1, ["写回键", const_key])
        fill_stats_row(table, 2, ["拟合方式", fit])
        fill_stats_row(table, 3, ["末轮峰 score", _fmt(conf, 4)])

    # ---------------- 写回焦面（作业 done 后可用） ----------------
    def _on_run_clicked(self):
        self._apply_job_id = None
        self._last_cam = None
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("（作业完成后可写回机器常数）")
        super()._on_run_clicked()

    def _on_finish(self, output):
        super()._on_finish(output)
        jid = (output or {}).get("job_id")
        if jid:
            self._apply_job_id = jid
            self.ui.btn_apply.setEnabled(True)
            field = _APPLY_FIELD_OF_CAM.get(self._last_cam) \
                or "as_z_chuck_focus_mm"
            self.ui.lbl_apply.setText(
                "作业 #%d 已完成，可写回 %s" % (jid, field))

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        cam = self._last_cam
        field = _APPLY_FIELD_OF_CAM.get(cam)
        if not field:
            self.ui.lbl_apply.setText("未知相机 %r，无法路由写回字段" % cam)
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid, field),
                         name="best-focus-apply", daemon=True).start()

    def _apply_worker(self, jid, field):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid, "fields": [field]}, timeout=15)
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
    test = BestFocusCalib()
    test.show()
    sys.exit(myapp.exec_())
