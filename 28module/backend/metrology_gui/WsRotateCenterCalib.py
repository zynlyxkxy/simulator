# -*- coding: utf-8 -*-
"""
Filename: WsRotateCenterCalib.py
Description:
    硅片台旋转中心标定（迭代 32 新测校项；界面三段式对照
    CameraMicrorotCalib）。核心链路在后端作业 ws_rotate_center
    （cam=down：示教点位上 WS_Rz 按 step_rad×n_angles 步进拍照，
    圆拟合实际旋转中心），本页走 job_base 的"参数下发 + 作业轮询 +
    结果展示"。

    参数：示教点表格（taught_points_table 共用行布局组件，2~10 点，
    迭代 35 起每点两位姿 {tpl, pose{AS_X,AS_Y,WS_LS_X,WS_LS_Y},
    pose_after{同结构可选}}：旋前走遍各点 pose 求 P_k，旋转后走遍
    pose_after 求 Q_k，后端按 find_rotation_center 口径拟合 θ 与中心；
    pose_after 缺省=pose（小角度），旋后位用于防止大角度旋转后标记出
    视场；读旧存档（无 pose_after）时旋后四格自动填旋前值。模板名
    必填、须已在视觉页注册，各配"→旋前"/"→旋后"取当前位按钮）
    + step_rad（rad，默认 0.01）+ n_angles（默认 3）+ repeat（默认
    10）+ wait_ms。

    结果：measured.rz_center_offset_mm=[cx,cy]（mm）+ 残差 RMS/3σ +
    中心 repeat 3σ(µm)；measured.rz_center_ref_xy_mm=[标定时
    WS_LS_X, WS_LS_Y]（标定机位，迭代 35 起 measured 双键——消费侧
    pivot=吸盘心+offset+(当前台位−ref) 平移复用）。结果区：中心
    坐标大数字显示（mm 与 µm）+ 逐 repeat 中心散点（µm）/趋势 +
    残差统计表 + 标定机位 ref_xy 只读行（缺省显示 —）+ "写回
    rz_center_offset_mm"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["rz_center_offset_mm"]，照
    CameraMicrorotCalib 的写回模式）。
"""
import math
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos, http_json
from qt_custom_ui import fill_stats_row
import WsRotateCenterCalib_ui as ui

# 示教位控件 attr → 后端契约轴名；"*_aft" 后缀=旋后位姿（迭代 35，
# 同旋转重复性）
_AXIS_OF_ATTR = {"as_x": "AS_X", "as_y": "AS_Y",
                 "ws_x": "WS_LS_X", "ws_y": "WS_LS_Y"}
_AXIS_OF_ATTR_AFT = {a + "_aft": ax for a, ax in _AXIS_OF_ATTR.items()}


class WsRotateCenterCalib(JobRoutineBase):
    ROUTINE = "ws_rotate_center"
    TITLE = "硅片台旋转中心标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None
        self._wire_point_groups()

        # 迭代 35：figure 收窄（默认 6.4in 过宽），给 8 列示教表让宽
        self.res_fig = Figure(figsize=(4.8, 3.6))
        self.res_ax_sc = self.res_fig.add_subplot(121)   # 中心散点
        self.res_ax_tr = self.res_fig.add_subplot(122)   # 逐 repeat 趋势
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
        """归一为 (cx_mm, cy_mm, xs_um, ys_um, rms_um, s3_um, ctr_s3_um)：
        cx/cy = measured.rz_center_offset_mm（mm）；xs/ys = 逐 repeat
        中心序列（µm，series.center_x_mm/center_y_mm 或 centers 点对）；
        rms/s3 = 残差 RMS/3σ（µm，stats.residual 的 rms_um/s3_um 或
        rms_mm/s3_mm×1e3）；ctr_s3 = 中心 repeat 3σ（µm，stats.center
        的 s3_um/s3_mm×1e3 或扁平 center_s3_um/repeat_s3_um；缺省由
        中心序列径向散布推算）。"""
        res = res or {}
        measured = res.get("measured") or {}
        st = res.get("stats") or {}
        series = res.get("series") or {}
        ctr = (measured.get("rz_center_offset_mm")
               or measured.get("center_mm") or [])
        cx = float(ctr[0]) if len(ctr) > 0 and ctr[0] is not None else None
        cy = float(ctr[1]) if len(ctr) > 1 and ctr[1] is not None else None
        # 逐 repeat 中心序列 → µm
        xs = series.get("center_x_mm") or series.get("cx_mm")
        ys = series.get("center_y_mm") or series.get("cy_mm")
        if (xs is None or ys is None) and series.get("centers"):
            pairs = [p for p in series["centers"]
                     if isinstance(p, (list, tuple)) and len(p) >= 2]
            xs = [p[0] for p in pairs]
            ys = [p[1] for p in pairs]
        xs_um = [float(v) * 1e3 for v in (xs or [])]
        ys_um = [float(v) * 1e3 for v in (ys or [])]
        # 残差 RMS/3σ（µm）
        resi = st.get("residual") or {}
        rms_um = _um(resi.get("rms_um"), resi.get("rms_mm"))
        s3_um = _um(resi.get("s3_um"), resi.get("s3_mm"))
        if rms_um is None:
            rms_um = _um(st.get("residual_rms_um"),
                         st.get("residual_rms_mm"))
        if s3_um is None:
            s3_um = _um(st.get("residual_s3_um"), st.get("residual_s3_mm"))
        # 中心 repeat 3σ（µm）
        cen = st.get("center") or {}
        ctr_s3_um = _um(cen.get("s3_um"), cen.get("s3_mm"))
        if ctr_s3_um is None:
            ctr_s3_um = _um(st.get("center_s3_um"), st.get("center_s3_mm"))
        if ctr_s3_um is None:
            ctr_s3_um = _um(st.get("repeat_s3_um"), st.get("repeat_s3_mm"))
        if ctr_s3_um is None and xs_um and ys_um:
            # 兜底：中心序列相对均值的径向散布 3σ（ddof=0）
            n = len(xs_um)
            mx = sum(xs_um) / n
            my = sum(ys_um) / n
            var = sum((xs_um[i] - mx) ** 2 + (ys_um[i] - my) ** 2
                      for i in range(n)) / n
            ctr_s3_um = 3.0 * math.sqrt(var)
        return cx, cy, xs_um, ys_um, rms_um, s3_um, ctr_s3_um

    @staticmethod
    def _ref_xy(res):
        """measured.rz_center_ref_xy_mm=[标定时 WS_LS_X, WS_LS_Y]
        （标定机位；消费侧 pivot=吸盘心+offset+(当前台位−ref) 平移
        复用）。缺省/旧 schema 返回 None。"""
        res = res or {}
        ref = (res.get("measured") or {}).get("rz_center_ref_xy_mm")
        if isinstance(ref, (list, tuple)) and len(ref) >= 2 \
                and ref[0] is not None and ref[1] is not None:
            return float(ref[0]), float(ref[1])
        return None

    def result_log_lines(self, res):
        cx, cy, xs_um, _y, rms_um, s3_um, ctr_s3_um = self._normalize(res)
        ref = self._ref_xy(res)
        return ["rz_center_offset_mm = [%s, %s] mm（cx %s µm, cy %s µm）"
                "；残差 RMS = %s µm, 残差 3σ = %s µm, 中心 repeat 3σ = "
                "%s µm, N = %d"
                % (_fmt(cx, 6), _fmt(cy, 6),
                   _fmt(None if cx is None else cx * 1e3, 3),
                   _fmt(None if cy is None else cy * 1e3, 3),
                   _fmt(rms_um, 3), _fmt(s3_um, 3),
                   _fmt(ctr_s3_um, 3), len(xs_um)),
                "标定机位 ref_xy = %s mm（台跑后消费侧按 当前台位−ref "
                "平移复用；换吸盘/维修旋转台后须重标）"
                % ("[%s, %s]" % (_fmt(ref[0], 6), _fmt(ref[1], 6))
                   if ref else "—")]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        cx, cy, xs_um, ys_um, rms_um, s3_um, ctr_s3_um = \
            self._normalize(res)
        # 中心坐标大数字（mm 与 µm 双单位）
        if cx is not None and cy is not None:
            self.ui.lbl_center.setText(
                "cx = %+.6f mm（%+.3f µm）　cy = %+.6f mm（%+.3f µm）"
                % (cx, cx * 1e3, cy, cy * 1e3))
        else:
            self.ui.lbl_center.setText("旋转中心：—（结果缺 "
                                       "measured.rz_center_offset_mm）")
        # 散点：逐 repeat 中心（µm）+ 均值十字
        self.res_ax_sc.clear()
        if xs_um and ys_um:
            self.res_ax_sc.plot(xs_um, ys_um, ".", color="#4a90e2",
                                markersize=6, label="per-repeat")
            mx = sum(xs_um) / len(xs_um)
            my = sum(ys_um) / len(ys_um)
            self.res_ax_sc.plot([mx], [my], "+", color="#c0392b",
                                markersize=12, markeredgewidth=2,
                                label="mean")
            self.res_ax_sc.legend(fontsize=7, loc="best")
        self.res_ax_sc.set_title("center scatter (µm): repeat 3σ %s"
                                 % _fmt(ctr_s3_um, 3), fontsize=9)
        self.res_ax_sc.set_xlabel("cx (µm)", fontsize=8)
        self.res_ax_sc.set_ylabel("cy (µm)", fontsize=8)
        self.res_ax_sc.tick_params(labelsize=8)
        # 趋势：逐 repeat cx/cy（µm）
        self.res_ax_tr.clear()
        if xs_um:
            self.res_ax_tr.plot(list(range(len(xs_um))), xs_um, ".-",
                                color="#4a90e2", markersize=3,
                                label="cx")
        if ys_um:
            self.res_ax_tr.plot(list(range(len(ys_um))), ys_um, ".-",
                                color="#e67e22", markersize=3,
                                label="cy")
        if xs_um or ys_um:
            self.res_ax_tr.legend(fontsize=7, loc="best")
        self.res_ax_tr.set_title("per-repeat center (µm)", fontsize=9)
        self.res_ax_tr.set_xlabel("repeat", fontsize=8)
        self.res_ax_tr.set_ylabel("µm", fontsize=8)
        self.res_ax_tr.tick_params(labelsize=8)
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 残差统计表 + 标定机位 ref_xy 只读行（迭代 38 可见化；
        # measured.rz_center_ref_xy_mm，缺省 —）
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["cx (mm)", _fmt(cx, 6)])
        fill_stats_row(table, 1, ["cy (mm)", _fmt(cy, 6)])
        fill_stats_row(table, 2, ["残差 RMS (µm)", _fmt(rms_um, 3)])
        fill_stats_row(table, 3, ["残差 3σ (µm)", _fmt(s3_um, 3)])
        fill_stats_row(table, 4, ["中心 repeat 3σ (µm)",
                                  _fmt(ctr_s3_um, 3)])
        ref = self._ref_xy(res)
        fill_stats_row(table, 5, ["标定机位 ref_xy (mm)",
                                  "[%s, %s]" % (_fmt(ref[0], 6),
                                                _fmt(ref[1], 6))
                                  if ref else None])

    # ---------------- 写回 rz_center_offset_mm（作业 done 后可用） ----------------
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
                "作业 #%d 已完成，可写回 rz_center_offset_mm" % jid)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="rz-center-apply", daemon=True).start()

    def _apply_worker(self, jid):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid,
                           "fields": ["rz_center_offset_mm"]},
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


def _um(v_um, v_mm):
    """µm 值优先，缺省 mm×1e3；皆无返 None。"""
    if v_um is not None:
        return float(v_um)
    if v_mm is not None:
        return float(v_mm) * 1e3
    return None


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = WsRotateCenterCalib()
    test.show()
    sys.exit(myapp.exec_())
