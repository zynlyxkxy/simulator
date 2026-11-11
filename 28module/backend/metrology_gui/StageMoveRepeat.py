# -*- coding: utf-8 -*-
"""
Filename: StageMoveRepeat.py
Description:
    运动台移动重复性（对照参考框架 AlignStabilityDynamic 的测法：
    起始位↔拍照位往复，到位拍照统计标记波动；界面三段式对照
    SingleCameraAlignStability）。核心链路在后端作业
    ws_move_repeat（晶圆台 WS_LS_X/Y，下相机看晶圆标记）/
    as_move_repeat（对准台 AS_X/Y，上相机看键合头芯片标记），
    本页走 job_base 的"参数下发 + 作业轮询 + 结果展示"。

    迭代 34 参数契约：routine 由运动台决定（晶圆台→ws_move_repeat /
    对准台→as_move_repeat），cam 由相机下拉决定（对准台可选 up/down
    缺省 up；晶圆台锁 down）；template_name 必填；对准台+下相机时
    附加 jog_group="align"（下相机看对准台运动，与
    AlignStabilityDynamic down+align 同一契约）。

    结果区：row/col 双曲线（series_nm，标题各带 3σ nm）+
    轴到位统计表（stats.axes：到位 mean mm / 3σ µm）。

    迭代 39：起始位置/拍照位置各配"取当前位"按钮（job_base.fetch_axes_pos
    纯 HTTP GET /api/axes）——按"运动台"下拉取 WS_LS_X/WS_LS_Y
    （晶圆台）或 AS_X/AS_Y（对准台）回填；后端离线仅追加日志不弹窗。
"""
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos
from qt_custom_ui import fill_stats_row
import StageMoveRepeat_ui as ui


class StageMoveRepeat(JobRoutineBase):
    TITLE = "运动台移动重复性"

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.cb_stage.currentTextChanged.connect(self._sync_camera_lock)
        self._sync_camera_lock()
        # 取当前位（纯 HTTP GET /api/axes，按运动台下拉映射轴组）
        self.ui.btn_snap_start.clicked.connect(
            lambda: self._snap_pos(self.ui.start_x, self.ui.start_y,
                                   "起始位置"))
        self.ui.btn_snap_photo.clicked.connect(
            lambda: self._snap_pos(self.ui.photo_x, self.ui.photo_y,
                                   "拍照位置"))
        self.bind_run_controls()

    # ---------------- 相机下拉（迭代 34） ----------------
    def _stage_is_wafer(self):
        return "晶圆台" in self.ui.cb_stage.currentText()

    def _cur_cam(self):
        if self._stage_is_wafer():
            return "down"
        return "down" if "下" in self.ui.cb_camera.currentText() else "up"

    def _sync_camera_lock(self, *_args):
        """晶圆台只由下相机观测：锁 down 并禁用下拉；对准台放开可选。"""
        wafer = self._stage_is_wafer()
        if wafer:
            self.ui.cb_camera.setCurrentText("下相机 (down)")
        self.ui.cb_camera.setEnabled(not wafer)

    # ---------------- 取当前位（GUI 线程） ----------------
    def _snap_axes(self):
        """取当前位轴组：按"运动台"下拉（晶圆台→WS_LS_X/Y /
        对准台→AS_X/Y）。"""
        return ["WS_LS_X", "WS_LS_Y"] if self._stage_is_wafer() \
            else ["AS_X", "AS_Y"]

    def _snap_pos(self, sb_x, sb_y, name):
        """GET /api/axes 按运动台轴组回填 (x,y)；失败仅追加日志。"""
        try:
            pos = fetch_axes_pos(timeout=5)
        except Exception as e:
            self.ui.textEdit.append("[warn] 取当前位失败：%s" % e)
            return
        axes = self._snap_axes()
        vx = pos.get(axes[0])
        vy = pos.get(axes[1])
        if vx is not None:
            sb_x.setValue(float(vx))
        if vy is not None:
            sb_y.setValue(float(vy))
        self.ui.textEdit.append(
            "%s 已回填当前位：x=%s y=%s（轴 %s）"
            % (name, sb_x.value(), sb_y.value(), "/".join(axes)))

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        tpl = self.ui.cb_vision_template.text().strip()
        if not tpl:
            raise ValueError("视觉模板名称必填（模板须在视觉页已注册）")
        INPUT = {
            'stage': self.ui.cb_stage.currentText(),
            'cam': self._cur_cam(),
            'vision_template': tpl,
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
            'start_pos_x': float(self.ui.start_x.value()),
            'start_pos_y': float(self.ui.start_y.value()),
            'photo_pos_x': float(self.ui.photo_x.value()),
            'photo_pos_y': float(self.ui.photo_y.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_stage.setCurrentText(param_dict['stage'])
        cam = param_dict.get('cam') or (
            "down" if self._is_wafer_stage(param_dict) else "up")
        self.ui.cb_camera.setCurrentText(
            "下相机 (down)" if cam == "down" else "上相机 (up)")
        self._sync_camera_lock()
        self.ui.cb_vision_template.setText(param_dict['vision_template'])
        self.ui.repeat_times.setValue(param_dict['repeat_time'])
        self.ui.wait_time.setValue(param_dict['wait_time'])
        self.ui.start_x.setValue(param_dict['start_pos_x'])
        self.ui.start_y.setValue(param_dict['start_pos_y'])
        self.ui.photo_x.setValue(param_dict['photo_pos_x'])
        self.ui.photo_y.setValue(param_dict['photo_pos_y'])

    # ---------------- 作业参数 ----------------
    def _is_wafer_stage(self, dict_input):
        return "晶圆台" in dict_input['stage']

    def build_routine(self, dict_input):
        return "ws_move_repeat" if self._is_wafer_stage(dict_input) \
            else "as_move_repeat"

    def build_cam(self, dict_input):
        return dict_input.get('cam') or (
            "down" if self._is_wafer_stage(dict_input) else "up")

    def build_params(self, dict_input):
        params = {"repeat": int(dict_input['repeat_time']),
                  "wait_ms": dict_input['wait_time'],
                  "start_xy": [dict_input['start_pos_x'],
                               dict_input['start_pos_y']],
                  "photo_xy": [dict_input['photo_pos_x'],
                               dict_input['photo_pos_y']],
                  "template_name": (dict_input['vision_template'] or "").strip()}
        # 对准台 + 下相机：下相机看对准台运动，jog_group=align
        # （与 AlignStabilityDynamic down+align 同一契约，后端透传）
        if dict_input.get('cam') == "down" \
                and not self._is_wafer_stage(dict_input):
            params["jog_group"] = "align"
        return params

    def result_log_lines(self, res):
        st = res.get("stats") or {}
        lines = []
        for ax in ("row", "col"):
            s = st.get(ax) or {}
            lines.append("%s 3σ = %s nm（mean=%s px）"
                         % (ax, s.get("s3_nm"), s.get("mean_px")))
        for axis, s in (st.get("axes") or {}).items():
            lines.append("轴 %s 到位 mean=%s mm, 3σ=%s µm"
                         % (axis, s.get("mean_mm"), s.get("s3_um")))
        return lines

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        res = res or {}
        series_nm = res.get("series_nm") or {}
        st = res.get("stats") or {}
        row_nm = series_nm.get("row") or []
        col_nm = series_nm.get("col") or []
        self.res_ax_row.clear()
        self.res_ax_col.clear()
        self.res_ax_row.plot(list(range(len(row_nm))), row_nm, ".-")
        self.res_ax_row.set_title("row: %s nm (3σ)"
                                  % _fmt((st.get("row") or {}).get("s3_nm"), 2))
        self.res_ax_row.set_ylabel("nm")
        self.res_ax_col.plot(list(range(len(col_nm))), col_nm, ".-")
        self.res_ax_col.set_title("col: %s nm (3σ)"
                                  % _fmt((st.get("col") or {}).get("s3_nm"), 2))
        self.res_ax_col.set_ylabel("nm")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 轴到位统计表（行序与 JOG 轴序一致：X 在前 Y 在后）
        axes_stats = st.get("axes") or {}
        for i, (axis, s) in enumerate(sorted(axes_stats.items())):
            if i >= self.ui.stats_table.rowCount():
                break
            fill_stats_row(self.ui.stats_table, i, [
                axis, _fmt(s.get("mean_mm"), 6), _fmt(s.get("s3_um"), 4)])


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = StageMoveRepeat()
    test.show()
    sys.exit(myapp.exec_())
