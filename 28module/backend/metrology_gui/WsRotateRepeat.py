# -*- coding: utf-8 -*-
"""
Filename: WsRotateRepeat.py
Description:
    硅片台旋转重复性（迭代 32 起 taught 直接测角法为唯一模式；界面
    三段式对照 SingleCameraAlignStability）。核心链路在后端作业
    ws_rotate_repeat 的 _run_ws_rotate_taught（基线逐点世界坐标 P_k
    →每循环 Rz+rot_rad→逐点预测走位→实测 Q_k→跨点 Kabsch 拟合
    θ_i→e_i=θ_i−rot_rad，stats.angle_err：mean=系统角偏差、
    3σ=旋转角重复性；auto 路径已于迭代 34 删除），本页走
    job_base 的"参数下发 + 作业轮询 + 结果展示"。

    迭代 31 多点化：参数 = 点数 n_points（2~10）+ taught_points
    [{tpl, pose{AS_X,AS_Y,WS_LS_X,WS_LS_Y}, pose_after{同结构}}, ...]
    （每组模板名必填、须已在视觉页注册，各配"→旋前"/"→旋后"取当前位
    按钮 GET /api/axes 分别回填旋前/旋后四格）+ rot_rad（rad，默认
    0.0175≈1°）/repeat/wait_ms。set_saved_param 兼容迭代 30 旧四键格式
    （tpl_before/tpl_after + rot_deg → 两点 + deg→rad 换算）与迭代
    32~34 旧档（无 pose_after → 旋后四格自动填旋前值）。

    迭代 35：每点两个位姿——旋前走遍各点 pose 求世界坐标 P_k，旋转后
    走遍 pose_after 求 Q_k（后端按 find_rotation_center 口径拟合 θ 与
    中心）；pose_after 缺省=pose（小角度旋转），旋后位用于防止大角度
    旋转后标记出视场。示教表扩 8 列（taught_points_table 迭代 35）。

    迭代 32：①示教组改表格式行布局（taught_points_table 共用组件，
    point_groups 接口不变，参数数组语义不变）；②结果区改直接测角
    语义——每循环角度误差 e=实测角−指令角（series.angle_err_urad，
    µrad）趋势（标题 mean/3σ）+ 各点位置残余（保留）+ 统计表角度行
    （stats.angle_err 的 mean_urad/std_urad/s3_urad）；旧字段
    （stats.residual/series.points[i].d_*/stats.dtheta）保留兼容
    判读。
"""
import math

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos
from qt_custom_ui import fill_stats_row
import WsRotateRepeat_ui as ui

# 示教位控件 attr → 后端契约轴名；"*_aft" 后缀=旋后位姿（迭代 35）
_AXIS_OF_ATTR = {"as_x": "AS_X", "as_y": "AS_Y",
                 "ws_x": "WS_LS_X", "ws_y": "WS_LS_Y"}
_AXIS_OF_ATTR_AFT = {a + "_aft": ax for a, ax in _AXIS_OF_ATTR.items()}
# 逐点残余序列字段候选：（col键, row键, 换算到 nm 的倍率；px 序列 ×k_nm
# 在调用处乘）
_PT_SER_KEYS = (("d_col_nm", "d_row_nm", 1.0),
                ("col_nm", "row_nm", 1.0),
                ("d_col_px", "d_row_px", None))     # None → ×k_nm
# 角度序列字段候选：（键, 换算到 µrad 的倍率）；迭代 32 首选直接测角
# angle_err_urad（每循环 e=实测角−指令角），旧 dtheta_* 候选保留兼容
_ANG_SER_KEYS = (("angle_err_urad", 1.0),
                 ("dtheta_urad", 1.0),
                 ("d_angle_urad", 1.0),
                 ("d_angle_deg", math.pi / 180.0 * 1e6),
                 ("dtheta_rad", 1e6))
# 角度统计容器候选（dict，含 mean_urad/std_urad/s3_urad）：迭代 32 首选
# stats.angle_err，旧 stats.dtheta/stats.angle 兼容
_ANG_STATS_KEYS = ("angle_err", "dtheta", "angle")


class WsRotateRepeat(JobRoutineBase):
    ROUTINE = "ws_rotate_repeat"
    TITLE = "硅片台旋转重复性"

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._wire_point_groups()

        # nm_per_px 启动刷新（后端不在线则保留初值 600 nm/px）
        refreshed = self.config.refresh_nm_per_px()

        # 迭代 35：figure 收窄（默认 6.4in=640px 会把结果组撑到 ~690），
        # 给 8 列示教表让宽，防 top_layout 缺口全压输入组导致横向滚动条
        self.res_fig = Figure(figsize=(4.2, 5.0))
        self.res_ax_col = self.res_fig.add_subplot(311)
        self.res_ax_row = self.res_fig.add_subplot(312)
        self.res_ax_ang = self.res_fig.add_subplot(313)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.n_points.valueChanged.connect(self._on_n_points_changed)
        self.bind_run_controls()
        self.ui.textEdit.append(
            "nm_per_px(down): %s（%s）"
            % (self.config.nm_per_px("down"),
               "已从 machine_constants 刷新" if refreshed
               else "初值，后端未在线"))

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
            'rot_rad': float(self.ui.rot_rad.value()),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        param_dict = dict(param_dict)
        if 'points' not in param_dict:
            # 迭代 30 旧四键格式兼容：旋前/旋后 → 两点，rot_deg → rad
            param_dict = {
                'n_points': 2,
                'points': [
                    {"tpl": param_dict['tpl_before'],
                     "pose": dict(param_dict['pose_before'])},
                    {"tpl": param_dict['tpl_after'],
                     "pose": dict(param_dict['pose_after'])}],
                'rot_rad': math.radians(float(param_dict.get(
                    'rot_deg', 1.0))),
                'repeat_time': param_dict['repeat_time'],
                'wait_time': param_dict['wait_time'],
            }
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
        self.ui.rot_rad.setValue(float(param_dict['rot_rad']))
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
                "rot_rad": dict_input['rot_rad'],
                "repeat": int(dict_input['repeat_time']),
                "wait_ms": dict_input['wait_time']}

    # ---------------- 结果解读（多 schema 归一，GUI 线程） ----------------
    def _normalize(self, res):
        """把后端结果归一为 (points, ang_ser_urad, ang_mean, ang_std,
        ang_s3)：
        points = [(点名, col残余nm序列, row残余nm序列, col_3σ_nm, row_3σ_nm)]
        角度 = 迭代 32 直接测角（series.angle_err_urad + stats.angle_err）
        优先，旧 Δθ 字段（series.dtheta_urad/stats.dtheta 等）与迭代 30
        两点示教（series.d_*_px + stats.residual）保留兼容。"""
        res = res or {}
        series = res.get("series") or {}
        st = res.get("stats") or {}
        k_nm = self.config.nm_per_px("down")
        tpl_names = []
        for pt in ((res.get("detail") or {}).get("taught_points") or []):
            if isinstance(pt, dict) and pt.get("tpl"):
                tpl_names.append(pt["tpl"])

        pts_ser = series.get("points") or series.get("per_point") or []
        pts_st = st.get("points") or st.get("per_point") or []
        points = []
        for i, ps in enumerate(pts_ser):
            if not isinstance(ps, dict):
                continue
            col, row = None, None
            for kc, kr, scale in _PT_SER_KEYS:
                if ps.get(kc) is not None and ps.get(kr) is not None:
                    f = (k_nm if scale is None else scale)
                    col = [v * f for v in (ps.get(kc) or [])]
                    row = [v * f for v in (ps.get(kr) or [])]
                    break
            if col is None:
                continue
            sst = pts_st[i] if i < len(pts_st) and \
                isinstance(pts_st[i], dict) else {}
            name = ps.get("tpl") or sst.get("tpl") or \
                (tpl_names[i] if i < len(tpl_names) else None) or \
                "pt %d" % (i + 1)
            points.append((name, col, row,
                           sst.get("col_s3_nm"), sst.get("row_s3_nm")))
        if not points:
            # 迭代 30 两点示教：旋前点残余（series.d_*_px + stats.residual）
            if series.get("d_col_px") is not None or \
                    (st.get("residual") or {}):
                rs = st.get("residual") or {}
                col = [v * k_nm for v in (series.get("d_col_px") or [])]
                row = [v * k_nm for v in (series.get("d_row_px") or [])]
                points.append(("taught pt (before)", col, row,
                               rs.get("col_s3_nm"), rs.get("row_s3_nm")))
        # 角度误差序列（µrad）与统计（mean/std/3σ）
        ang_ser = []
        for key, f in _ANG_SER_KEYS:
            if series.get(key):
                ang_ser = [v * f for v in series[key]]
                break
        ang_mean = ang_std = ang_s3 = None
        for key in _ANG_STATS_KEYS:
            cand = st.get(key)
            if not isinstance(cand, dict):
                continue
            if ang_mean is None:
                ang_mean = cand.get("mean_urad")
            if ang_std is None:
                ang_std = cand.get("std_urad")
            if ang_s3 is None:
                ang_s3 = cand.get("s3_urad")
        if ang_s3 is None:
            for key in ("angle_s3_urad", "dtheta_s3_urad"):
                if st.get(key) is not None:
                    ang_s3 = st.get(key)
                    break
        if ang_s3 is None:      # 迭代 30 两点示教 residual 内的角度 3σ
            ang_s3 = (st.get("residual") or {}).get("angle_s3_urad")
        # 兜底：由序列推算（总体口径 ddof=0，与后端统计惯例一致）
        if ang_ser and (ang_mean is None or ang_std is None):
            n = len(ang_ser)
            mean_u = sum(ang_ser) / n
            var = sum((v - mean_u) ** 2 for v in ang_ser) / n
            if ang_mean is None:
                ang_mean = mean_u
            if ang_std is None:
                ang_std = math.sqrt(var)
        if ang_s3 is None and ang_std is not None:
            ang_s3 = 3.0 * ang_std
        return points, ang_ser, ang_mean, ang_std, ang_s3

    def result_log_lines(self, res):
        points, _s, ang_mean, ang_std, ang_s3 = self._normalize(res)
        lines = []
        for name, _c, _r, c3, r3 in points:
            lines.append("示教点 %s 回零残余：col 3σ = %s nm, "
                         "row 3σ = %s nm" % (name, c3, r3))
        lines.append("角度误差 e（实测−指令）：mean = %s µrad, "
                     "std = %s µrad, 3σ = %s µrad"
                     % (_fmt(ang_mean, 3), _fmt(ang_std, 3),
                        _fmt(ang_s3, 3)))
        return lines

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        points, ang_ser, ang_mean, ang_std, ang_s3 = self._normalize(res)
        self.res_ax_col.clear()
        self.res_ax_row.clear()
        self.res_ax_ang.clear()
        for name, col, row, _c3, _r3 in points:
            self.res_ax_col.plot(list(range(len(col))), col, ".-",
                                 label=name, markersize=3)
            self.res_ax_row.plot(list(range(len(row))), row, ".-",
                                 label=name, markersize=3)
        s3_col = _wrap_s3([(n, c3) for n, _c, _r, c3, _r3 in points])
        s3_row = _wrap_s3([(n, r3) for n, _c, _r, _c3, r3 in points])
        self.res_ax_col.set_title("re-home col residual 3σ (nm):\n%s"
                                  % s3_col, fontsize=9)
        self.res_ax_col.set_ylabel("nm")
        if points:
            self.res_ax_col.legend(fontsize=7, loc="upper left")
        self.res_ax_row.set_title("re-home row residual 3σ (nm):\n%s"
                                  % s3_row, fontsize=9)
        self.res_ax_row.set_ylabel("nm")
        if points:
            self.res_ax_row.legend(fontsize=7, loc="upper left")
        if ang_ser:
            self.res_ax_ang.plot(list(range(len(ang_ser))), ang_ser, ".-",
                                 color="#c0392b", markersize=3)
        self.res_ax_ang.set_title("angle error e = meas − cmd: "
                                  "mean %s µrad / 3σ %s µrad"
                                  % (_fmt(ang_mean, 3), _fmt(ang_s3, 3)),
                                  fontsize=9)
        self.res_ax_ang.set_ylabel("µrad")
        self.res_ax_ang.set_xlabel("cycle")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表：N 点行 + 3 行角度误差 e（mean/std/3σ）
        table = self.ui.stats_table
        n_rows = len(points) + 3
        table.setRowCount(n_rows)
        table.setFixedHeight(min(34 + 30 * n_rows, 240))
        for i, (name, _c, _r, c3, r3) in enumerate(points):
            fill_stats_row(table, i, [name, _fmt(c3, 2), _fmt(r3, 2), "—"])
        fill_stats_row(table, n_rows - 3,
                       ["角度误差 e mean", "—", "—", _fmt(ang_mean, 3)])
        fill_stats_row(table, n_rows - 2,
                       ["角度误差 e std", "—", "—", _fmt(ang_std, 3)])
        fill_stats_row(table, n_rows - 1,
                       ["角度误差 e 3σ", "—", "—", _fmt(ang_s3, 3)])


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


def _wrap_s3(pairs, per_line=3):
    """各点 3σ 标题串：每行最多 per_line 个点（点多了单行标题超出图幅）。"""
    if not pairs:
        return "—"
    segs = ["%s %s" % (n, _fmt(v, 1)) for n, v in pairs]
    return "\n".join(" / ".join(segs[i:i + per_line])
                     for i in range(0, len(segs), per_line))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = WsRotateRepeat()
    test.show()
    sys.exit(myapp.exec_())
