# -*- coding: utf-8 -*-
"""
Filename: AsStageNonortho.py
Description:
    对准台非正交测校（迭代 37 改栅格跳测法、迭代 38 十字对称跳测 +
    θ_y 偏差口径；界面三段式对照 WsRotateCenterCalib）。核心链路在
    后端作业 as_stage_nonortho（cam=down：以晶圆 die 阵列为正交基准
    尺，X/Y 两线十字跳测 K 点——K 必须奇数，节点=center±(K−1)/2
    步——逐点识别标记求世界偏移拟合方向角，φ=两线角差−90°，杠杆臂
    =(K−1)×pitch），本页走 job_base 的"参数下发 + 作业轮询 +
    结果展示"。

    参数（迭代 37 契约；迭代 38 起 k_points 限奇数 3~21，spinbox
    singleStep=2）：template_name 必填 + ref_axis（"x"|"y" 默认
    "x"——X 基准=Y 导轨相对 X 的偏角，与机器常数 as_y_rail_dir_urad
    语义一致、为写回值；Y 基准=其负值，交叉核对用）+ k_points（3~21
    奇数 默认5）+ pitch_mm（0.1~50 默认=处方 wafer_sites 间距；启动
    时 GET /api/recipe 预填，auto=size+gap（size 为 null 跟随
    film.dies 芯片大小）、manual=pitch_x/y_mm；失败回退 20.0；后端
    校验与处方一致，不一致 400）+ frames_per_point（1~10 默认3）+
    repeat（2~10 默认3）+ wait_ms（默认10）+ center_xy 可选 [x,y]
    （"指定中心点"勾选才下发，缺省不下发=当前位；"取当前位"按钮按
    AS_X/AS_Y 回填）。

    结果：measured.phi_urad（非正交角 mean）/ as_nonortho_urad（写回
    值，X 基准）/ phi_std_urad / phi_3sigma_urad / theta_x_urad /
    theta_y_urad（迭代 38 起 θ_y 改偏差口径：wrap(θ_y−π/2)=φ−ρ̂，
    µrad 小数值；旧为绝对角 ~1.57e6）/ resid_rms_nm / resid_max_nm
    （线法向残差=弓形诊断）/ series_phi_urad；series 新增 nodes_x/
    nodes_y（末循环逐节点 {i,pos,col,row,ox,oy}）与 fit_x/fit_y=
    {slope:[sx,sy],mean:[mx,my]}（单位 mm）。结果区：φ mean 大数字
    （3σ µrad）+ 两个子图（上=逐次 φ 趋势；下=末循环节点散点
    (ox,oy) 平面 X/Y 两线两色 + 按 fit_x/fit_y slope/mean 的拟合
    直线，数据缺省留空不报错）+ θ_x/θ_y−90° 只读行 + resid_rms/max
    行 + "写回 as_nonortho_urad"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["as_nonortho_urad"] → 机器常数
    stage_geo 组，照 WsRotateCenterCalib 写回模式）。
    迭代 39：measured 新增 matrix_l2p（逻辑→物理矩阵 A，指令→实际）
    / matrix_p2l（物理→逻辑矩阵 A⁻¹，补偿前馈）——统计表下方只读
    矩阵块显示两个 2×2（9 位有效数字）；旧 schema 无矩阵键留空不崩。
    迭代 42b：矩阵升级为全矩阵（[col_x|col_y]=各线步进向量/pitch
    实测像，列模长=ε，X 线方向不再强制贴逻辑 X）；矩阵块下方新增
    比例误差只读行（measured.scale_ppm_x/y → εx/εy ppm，含相机像元
    当量误差别名；旧 schema 无键显示 —）。
    迭代 45：规范对齐（《XY运动台非正交（垂直度）校准与补偿技术规范》
    Rev2.0）+ 结果区直观化——measured 增 exy_urad（规范式 4 口径，
    夹角>90°为正，=−X 基准 φ）/beta_urad（上板倾斜 β=ρ̄）/exy_u_urad/
    exy_U_urad（附录 D 不确定度 k=2），detail 增 ac_check_norm（6.4
    ‖AC−I‖ 核验）；series 节点增 mwx/mwy（mark_world 迭代 44 口径）。
    结果区图改 2×2：左上=逐次 φ 趋势；右上=测量十字图（mark_world
    真比例散点+理想十字+拟合线+角度文本框）；左下/右下=X/Y 线拟合
    细节（横向/沿轨偏差 nm vs 沿线位置 mm+拟合线）；旧 (ox,oy) 散点
    子图退役。统计表 4→7 行（θ_x/θ_y−90°/EXY/U(k=2)/β/残差 RMS/
    MAX），矩阵块附 6.4 核验行。与 WsStageNonortho 页镜像同步。
"""
import math
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos, http_json
from qt_custom_ui import fill_stats_row
import AsStageNonortho_ui as ui

# 取当前位运动组（cam=down 看对准台，与 jog_group=align 同一轴组）
_SNAP_AXES = ["AS_X", "AS_Y"]
# 写回字段（→ 机器常数 stage_geo 组；恒为 X 基准值）
_APPLY_FIELDS = ["as_nonortho_urad"]
# 处方拉取失败时的 pitch 缺省（默认处方 wafer_sites auto=size+gap=20mm）
_PITCH_FALLBACK_MM = 20.0


def fetch_recipe_pitch(timeout=3.0):
    """GET /api/recipe → (pitch_x_mm, pitch_y_mm)（wafer_sites 栅格间距）。
    auto：pitch=size+gap（size 为 null 跟随 film.dies 芯片大小）；
    manual：pitch_x_mm/pitch_y_mm。后端离线/字段缺失抛异常（调用方
    回退 _PITCH_FALLBACK_MM）。"""
    body = http_json("GET", "/api/recipe", timeout=timeout)
    rec = body.get("recipe") if isinstance(body, dict) else body
    ws = rec["wafer_sites"]
    mode = ws.get("mode", "manual")
    if mode == "manual":
        return float(ws["pitch_x_mm"]), float(ws["pitch_y_mm"])
    dies = (rec.get("film") or {}).get("dies") or {}
    sx, sy = ws.get("size_x_mm"), ws.get("size_y_mm")
    if sx is None:
        sx = dies["size_x_mm"]
    if sy is None:
        sy = dies["size_y_mm"]
    return float(sx) + float(ws["gap_x_mm"]), \
        float(sy) + float(ws["gap_y_mm"])


class AsStageNonortho(JobRoutineBase):
    ROUTINE = "as_stage_nonortho"
    TITLE = "对准台非正交测校"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None

        # 结果区 2×2（迭代 45 直观化重构）：左上=逐次 φ 趋势；右上=测量
        # 十字图（末循环 mark_world 真比例散点 + 理想十字 + 拟合线 + 角度
        # 文本框）；左下/右下=X/Y 线拟合细节（横向/沿轨偏差 nm vs 沿线
        # 位置 mm + 拟合线——「沿线位置-偏差」参考图形式）
        self.res_fig = Figure(figsize=(7.0, 5.2))
        self.res_ax = self.res_fig.add_subplot(221)
        self.res_ax_cross = self.res_fig.add_subplot(222)
        self.res_ax_xl = self.res_fig.add_subplot(223)
        self.res_ax_yl = self.res_fig.add_subplot(224)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.chk_center.toggled.connect(self._sync_center_enabled)
        self._sync_center_enabled()
        self.ui.btn_snap.clicked.connect(self._snap_center)
        self.ui.btn_apply.clicked.connect(self._on_apply_clicked)
        self.sig_apply_done.connect(self._on_apply_done)
        self.ui.k_points.setToolTip(
            "十字对称跳测点数（奇数 3~21）。规范 5.3 建议每线 ≥11 点："
            "不确定度 ∝ 1/(d·K^1.5)，加大点数/跨距的收益大于增加帧数")
        self.bind_run_controls()
        self._prefill_pitch_from_recipe()

    # ---------------- 处方点距预填（迭代 37 栅格跳测法） ----------------
    def _cur_ref_axis(self):
        return "x" if "X" in self.ui.cb_ref_axis.currentText() else "y"

    def _prefill_pitch_from_recipe(self):
        """启动时 GET /api/recipe 预填处方 wafer_sites 栅格间距（按当前
        基准轴取向；失败静默回退 20.0=默认处方 auto 值）。"""
        try:
            px, py = fetch_recipe_pitch(timeout=3.0)
        except Exception:
            return          # 后端离线/字段缺失：保持缺省 20.0
        pitch = px if self._cur_ref_axis() == "x" else py
        self.ui.pitch_mm.setValue(min(max(pitch, self.ui.pitch_mm.minimum()),
                                      self.ui.pitch_mm.maximum()))
        self.ui.pitch_mm.setToolTip(
            self.ui.pitch_mm.toolTip()
            + "。当前处方 wafer_sites 间距：x=%s, y=%s mm" % (px, py))

    # ---------------- 取当前位（GUI 线程） ----------------
    def _sync_center_enabled(self, *_args):
        """"指定中心点"勾选才使能中心输入与取当前位（缺省=不下发）。"""
        on = self.ui.chk_center.isChecked()
        self.ui.center_x.setEnabled(on)
        self.ui.center_y.setEnabled(on)
        self.ui.btn_snap.setEnabled(on)

    def _snap_center(self):
        try:
            pos = fetch_axes_pos(timeout=5)
        except Exception as e:
            self.ui.textEdit.append("[warn] 取当前位失败：%s" % e)
            return
        vx = pos.get(_SNAP_AXES[0])
        vy = pos.get(_SNAP_AXES[1])
        if vx is not None:
            self.ui.center_x.setValue(float(vx))
        if vy is not None:
            self.ui.center_y.setValue(float(vy))
        self.ui.textEdit.append(
            "扫描中心已回填当前位：x=%s y=%s（轴 %s）"
            % (self.ui.center_x.value(), self.ui.center_y.value(),
               "/".join(_SNAP_AXES)))

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        tpl = self.ui.template_name.text().strip()
        if not tpl:
            raise ValueError("视觉模板名称必填（模板须在视觉页已注册）")
        INPUT = {
            'template_name': tpl,
            'ref_axis': self._cur_ref_axis(),
            'center_specified': self.ui.chk_center.isChecked(),
            'center_x': float(self.ui.center_x.value()),
            'center_y': float(self.ui.center_y.value()),
            'k_points': int(self.ui.k_points.value()),
            'pitch_mm': float(self.ui.pitch_mm.value()),
            'frames_per_point': int(self.ui.frames_per_point.value()),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.template_name.setText(param_dict.get('template_name', ""))
        self.ui.cb_ref_axis.setCurrentText(
            "X 基准 (x)" if param_dict.get('ref_axis', "x") == "x"
            else "Y 基准 (y)")
        self.ui.chk_center.setChecked(
            bool(param_dict.get('center_specified')))
        self.ui.center_x.setValue(float(param_dict.get('center_x', 0.0)))
        self.ui.center_y.setValue(float(param_dict.get('center_y', 0.0)))
        self.ui.k_points.setValue(int(param_dict.get('k_points', 5)))
        self.ui.pitch_mm.setValue(
            float(param_dict.get('pitch_mm', _PITCH_FALLBACK_MM)))
        self.ui.frames_per_point.setValue(
            int(param_dict.get('frames_per_point', 3)))
        self.ui.repeat_times.setValue(int(param_dict.get('repeat_time', 3)))
        self.ui.wait_time.setValue(float(param_dict.get('wait_time', 10)))
        self._sync_center_enabled()

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        return "down"

    def build_params(self, dict_input):
        params = {"template_name": dict_input['template_name'],
                  "ref_axis": dict_input.get('ref_axis') or "x",
                  "k_points": int(dict_input['k_points']),
                  "pitch_mm": dict_input['pitch_mm'],
                  "frames_per_point": int(dict_input['frames_per_point']),
                  "repeat": int(dict_input['repeat_time']),
                  "wait_ms": dict_input['wait_time']}
        # center_xy 可选：缺省不下发 = 后端取当前位
        if dict_input.get('center_specified'):
            params["center_xy"] = [dict_input['center_x'],
                                   dict_input['center_y']]
        return params

    # ---------------- 结果解读（GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """归一为 (phi_mean, phi_std, phi_s3, theta_x, theta_y,
        resid_rms, resid_max, series_phi)。后端事实源 measured：
        phi_urad / phi_std_urad / phi_3sigma_urad / theta_x_urad /
        theta_y_urad / resid_rms_nm / resid_max_nm / series_phi_urad；
        统计缺省时由 series_phi_urad 兜底推算（ddof=0，与后端惯例
        一致）。"""
        res = res or {}
        measured = res.get("measured") or {}
        series = res.get("series") or {}
        phi_mean = measured.get("phi_urad")
        phi_std = measured.get("phi_std_urad")
        phi_s3 = measured.get("phi_3sigma_urad")
        theta_x = measured.get("theta_x_urad")
        theta_y = measured.get("theta_y_urad")
        resid_rms = measured.get("resid_rms_nm")
        resid_max = measured.get("resid_max_nm")
        ser = measured.get("series_phi_urad") \
            or series.get("phi_urad") or series.get("series_phi_urad") or []
        phi = [float(v) for v in ser]
        if phi and (phi_mean is None or phi_std is None):
            n = len(phi)
            mean_u = sum(phi) / n
            var = sum((v - mean_u) ** 2 for v in phi) / n
            if phi_mean is None:
                phi_mean = mean_u
            if phi_std is None:
                phi_std = math.sqrt(var)
        if phi_s3 is None and phi_std is not None:
            phi_s3 = 3.0 * phi_std
        return (phi_mean, phi_std, phi_s3, theta_x, theta_y,
                resid_rms, resid_max, phi)

    @staticmethod
    def _normalize_nodes(res):
        """归一末循环节点散点/拟合（迭代 38：series.nodes_x/nodes_y
        逐节点 {i,pos,col,row,ox,oy}，fit_x/fit_y={slope:[sx,sy],
        mean:[mx,my]}，单位 mm；迭代 45 节点增 mwx/mwy=mark_world
        台基准/相机链实测位，供测量十字图）。返回 (nodes_x, nodes_y,
        fit_x, fit_y)；旧 schema/字段缺失对应项为 None（调用方留空
        不报错，向后兼容）。"""
        res = res or {}
        series = res.get("series") or {}

        def _nodes(key):
            raw = series.get(key)
            if not isinstance(raw, list):
                return None
            out = []
            for p in raw:
                if not isinstance(p, dict):
                    continue
                try:
                    out.append({"i": float(p.get("i", len(out))),
                                "pos": float(p.get("pos")),
                                "ox": float(p["ox"]),
                                "oy": float(p["oy"]),
                                "mwx": (None if p.get("mwx") is None
                                        else float(p["mwx"])),
                                "mwy": (None if p.get("mwy") is None
                                        else float(p["mwy"]))})
                except (TypeError, ValueError, KeyError):
                    continue
            return out or None

        def _fit(key):
            f = series.get(key)
            if not isinstance(f, dict):
                return None
            try:
                return {"slope": (float(f["slope"][0]),
                                  float(f["slope"][1])),
                        "mean": (float(f["mean"][0]),
                                 float(f["mean"][1]))}
            except (TypeError, ValueError, KeyError, IndexError):
                return None

        return (_nodes("nodes_x"), _nodes("nodes_y"),
                _fit("fit_x"), _fit("fit_y"))

    @staticmethod
    def _normalize_detail(res):
        """归一 detail 的展示参数（迭代 45）：center_xy/node_pitch_mm/
        k_points/ac_check_norm（规范 6.4 ‖AC−I‖ 核验）。旧 schema 缺项
        → 对应 None。"""
        det = (res or {}).get("detail") or {}

        def _f(key):
            v = det.get(key)
            try:
                return None if v is None else float(v)
            except (TypeError, ValueError):
                return None

        center = det.get("center_xy")
        try:
            center = (float(center[0]), float(center[1]))
        except (TypeError, ValueError, IndexError):
            center = None
        k = det.get("k_points")
        try:
            k = None if k is None else int(k)
        except (TypeError, ValueError):
            k = None
        return {"center": center, "pitch": _f("node_pitch_mm"),
                "k": k, "ac_check": _f("ac_check_norm")}

    @staticmethod
    def _normalize_spec(res):
        """归一 measured 的规范对齐字段（迭代 45）：exy_urad（规范式 4
        口径，夹角>90°为正）/beta_urad（上板倾斜 β=ρ̄）/exy_u_urad/
        exy_U_urad（附录 D 标准/扩展 k=2 不确定度）。旧 schema 全 None。"""
        measured = (res or {}).get("measured") or {}

        def _f(key):
            v = measured.get(key)
            try:
                return None if v is None else float(v)
            except (TypeError, ValueError):
                return None

        return (_f("exy_urad"), _f("beta_urad"),
                _f("exy_u_urad"), _f("exy_U_urad"))

    @staticmethod
    def _normalize_matrix(res):
        """归一 measured.matrix_l2p / matrix_p2l（2×2 嵌套数组，
        迭代 39）。旧 schema 无矩阵键 → (None, None)（调用方留空
        不崩，向后兼容）。"""
        measured = (res or {}).get("measured") or {}

        def _mat(key):
            m = measured.get(key)
            if not (isinstance(m, (list, tuple)) and len(m) == 2):
                return None
            try:
                return [[float(m[0][0]), float(m[0][1])],
                        [float(m[1][0]), float(m[1][1])]]
            except (TypeError, ValueError, IndexError):
                return None

        return _mat("matrix_l2p"), _mat("matrix_p2l")

    @staticmethod
    def _normalize_scale(res):
        """归一 measured.scale_ppm_x / scale_ppm_y（迭代 42b）。旧
        schema 无键 → (None, None)（调用方显示 —，向后兼容）。"""
        measured = (res or {}).get("measured") or {}

        def _f(key):
            v = measured.get(key)
            try:
                return None if v is None else float(v)
            except (TypeError, ValueError):
                return None

        return _f("scale_ppm_x"), _f("scale_ppm_y")

    @staticmethod
    def _scale_row_text(spx, spy):
        """比例误差行显示文本（两值均缺=旧 schema：—）。"""
        if spx is None and spy is None:
            return "比例误差 εx/εy：—（结果缺 measured.scale_ppm_x/y）"
        return ("比例误差 εx = %s ppm，εy = %s ppm"
                "（含相机像元当量误差别名；阿贝/直线度位置项在残差/LUT）"
                % (_fmt_ppm(spx), _fmt_ppm(spy)))

    @staticmethod
    def _matrix_block_text(m_l2p, m_p2l):
        """矩阵块显示文本（两矩阵均缺=旧 schema，整块留空）。"""
        if m_l2p is None and m_p2l is None:
            return ""
        return ("逻辑→物理矩阵 A（指令→实际）= %s\n"
                "物理→逻辑矩阵 A⁻¹（补偿前馈）= %s\n"
                "[col_x|col_y]=各线步进向量/pitch 实测像（列模长=ε，"
                "比例见下行）" % (_fmt_mat(m_l2p), _fmt_mat(m_p2l)))

    def result_log_lines(self, res):
        (phi_mean, phi_std, phi_s3, theta_x, theta_y,
         resid_rms, resid_max, phi) = self._normalize(res)
        lines = ["非正交角 φ = %s µrad（std %s, 3σ %s, N = %d）；"
                 "θ_x = %s µrad, θ_y−90° = %s µrad（轴-栅格角偏差）；"
                 "线法向残差 RMS = %s nm, MAX = %s nm（弓形诊断）"
                 % (_fmt(phi_mean, 3), _fmt(phi_std, 3), _fmt(phi_s3, 3),
                    len(phi), _fmt(theta_x, 3), _fmt(theta_y, 3),
                    _fmt(resid_rms, 3), _fmt(resid_max, 3))]
        # 迭代 39：矩阵行（旧 schema 无矩阵键不加）
        m_l2p, m_p2l = self._normalize_matrix(res)
        if m_l2p is not None or m_p2l is not None:
            lines.append("逻辑→物理矩阵 A = %s；物理→逻辑矩阵 A⁻¹ = %s"
                         "（[col_x|col_y]=步进向量/pitch 实测像，列模长=ε）"
                         % (_fmt_mat(m_l2p), _fmt_mat(m_p2l)))
        # 迭代 42b：比例误差行（旧 schema 无键不加）
        spx, spy = self._normalize_scale(res)
        if spx is not None or spy is not None:
            lines.append("比例误差 εx = %s ppm，εy = %s ppm"
                         % (_fmt_ppm(spx), _fmt_ppm(spy)))
        # 迭代 45：规范对齐行（EXY 式 4 口径 / 附录 D 不确定度 / β）
        exy, beta, u_u, U_u = self._normalize_spec(res)
        if exy is not None or U_u is not None or beta is not None:
            lines.append("EXY（规范口径，夹角>90°为正）= %s µrad"
                         "（U = %s µrad, k=2）；β 上板倾斜 = %s µrad"
                         % (_fmt(exy, 3), _fmt(U_u, 3), _fmt(beta, 3)))
        return lines

    # ---------------- 结果可视化（GUI 线程） ----------------
    def _plot_cross(self, nodes_x, nodes_y, fit_x, fit_y, det, angles):
        """测量十字图（迭代 45）：末循环 mark_world 真比例散点（X 蓝/Y
        橙）+ 理想十字（锚=全部节点 mark_world 均值，臂=±(K−1)/2·pitch
        沿逻辑轴，灰虚线+端点）+ 拟合线（mark_world 空间：均值点+方向
        (pitch+s_trk, s_lat)/步）+ 角度文本框（µrad 级在 mm 跨距上不可
        见，数值标注）。mark_world 缺省（旧 schema）留空不崩。"""
        ax = self.res_ax_cross
        ax.clear()
        pts = {"X": ([], []), "Y": ([], [])}
        for nodes, key in ((nodes_x, "X"), (nodes_y, "Y")):
            if nodes:
                for p in nodes:
                    if p["mwx"] is not None and p["mwy"] is not None:
                        pts[key][0].append(p["mwx"])
                        pts[key][1].append(p["mwy"])
        if not (pts["X"][0] or pts["Y"][0]):
            ax.set_title("measured cross vs ideal — no mark_world"
                         " (old schema)", fontsize=9)
            return
        allx = pts["X"][0] + pts["Y"][0]
        ally = pts["X"][1] + pts["Y"][1]
        ax0 = sum(allx) / len(allx)
        ay0 = sum(ally) / len(ally)
        pitch = det.get("pitch")
        k = det.get("k")
        if pitch and k:                  # 理想十字（逻辑轴方向）
            half = (k - 1) / 2.0
            xs = [ax0 + (i - half) * pitch for i in range(k)]
            ys = [ay0 + (i - half) * pitch for i in range(k)]
            ax.plot(xs, [ay0] * k, "--+", color="#999999", linewidth=0.8,
                    markersize=4, label="ideal")
            ax.plot([ax0] * k, ys, "--+", color="#999999", linewidth=0.8,
                    markersize=4)
        ax.plot(pts["X"][0], pts["X"][1], ".", color="#4a90e2",
                markersize=6, label="X line")
        ax.plot(pts["Y"][0], pts["Y"][1], ".", color="#e67e22",
                markersize=6, label="Y line")
        for is_x, nodes, fit, color in (
                (True, nodes_x, fit_x, "#4a90e2"),
                (False, nodes_y, fit_y, "#e67e22")):
            if not (nodes and fit and pitch):
                continue
            mw = [(p["mwx"], p["mwy"]) for p in nodes
                  if p["mwx"] is not None and p["mwy"] is not None]
            if not mw:
                continue
            mxw = sum(p[0] for p in mw) / len(mw)
            myw = sum(p[1] for p in mw) / len(mw)
            sx, sy = fit["slope"]
            dx, dy = (pitch + sx, sy) if is_x else (sx, pitch + sy)
            half = (len(mw) - 1) / 2.0 + 0.3
            ax.plot([mxw - dx * half, mxw + dx * half],
                    [myw - dy * half, myw + dy * half], "-",
                    color=color, linewidth=1.2)
        phi_mean, theta_x, theta_y = angles
        ax.text(0.02, 0.98,
                "phi = %s urad\ntheta_x = %s\ntheta_y-90 = %s"
                % (_fmt(phi_mean, 3), _fmt(theta_x, 3), _fmt(theta_y, 3)),
                transform=ax.transAxes, fontsize=8, va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f7f7",
                          ec="#cccccc", lw=0.5))
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title("measured cross (mark_world) vs ideal (mm)",
                     fontsize=9)
        ax.set_xlabel("x (mm)", fontsize=8)
        ax.set_ylabel("y (mm)", fontsize=8)
        ax.tick_params(labelsize=8)
        ax.legend(fontsize=7, loc="lower right")

    def _plot_line_detail(self, ax, nodes, fit, pitch, line):
        """X/Y 线拟合细节（迭代 45）：横向/沿轨偏差（nm，相对本线均值）
        vs 沿线位置（mm）+ 拟合线；标题标注横向斜率（µrad）与沿轨斜率
        （ppm）。X 线：横向=oy、沿轨=ox；Y 线互换。数据缺省留空不崩。"""
        ax.clear()
        if not nodes:
            ax.set_title("%s line fit detail — no data" % line,
                         fontsize=9)
            return
        mpos = sum(p["pos"] for p in nodes) / len(nodes)
        mox = sum(p["ox"] for p in nodes) / len(nodes)
        moy = sum(p["oy"] for p in nodes) / len(nodes)
        taus = [p["pos"] - mpos for p in nodes]
        lat = [((p["oy"] if line == "X" else p["ox"])
                - (moy if line == "X" else mox)) * 1e6 for p in nodes]
        trk = [((p["ox"] if line == "X" else p["oy"])
                - (mox if line == "X" else moy)) * 1e6 for p in nodes]
        ax.plot(taus, lat, ".", color="#4a90e2", markersize=5,
                label="lateral")
        ax.plot(taus, trk, ".", color="#27ae60", markersize=5,
                label="along-track")
        note = []
        if fit and pitch:
            sx, sy = fit["slope"]
            s_lat = (sy if line == "X" else sx) / pitch    # mm/mm
            s_trk = (sx if line == "X" else sy) / pitch
            t0, t1 = min(taus), max(taus)
            ax.plot([t0, t1], [s_lat * t0 * 1e6, s_lat * t1 * 1e6], "-",
                    color="#4a90e2", linewidth=1.2)
            ax.plot([t0, t1], [s_trk * t0 * 1e6, s_trk * t1 * 1e6], "-",
                    color="#27ae60", linewidth=1.2)
            note = ["lat slope %+.3f urad" % (s_lat * 1e6),
                    "trk slope %+.1f ppm" % (s_trk * 1e6)]
        ax.axhline(0.0, color="#cccccc", linewidth=0.8)
        if note:
            ax.text(0.02, 0.02, "  ".join(note), transform=ax.transAxes,
                    fontsize=7.5, va="bottom", ha="left",
                    bbox=dict(boxstyle="round,pad=0.25", fc="#f7f7f7",
                              ec="#cccccc", lw=0.5))
        ax.set_title("%s line fit detail (nm vs mm)" % line, fontsize=9)
        ax.set_xlabel("position along line (mm)", fontsize=8)
        ax.set_ylabel("deviation (nm)", fontsize=8)
        ax.tick_params(labelsize=8)
        ax.legend(fontsize=7, loc="best")

    def plot_realtime_data(self, res):
        (phi_mean, phi_std, phi_s3, theta_x, theta_y,
         resid_rms, resid_max, phi) = self._normalize(res)
        exy, beta, u_u, U_u = self._normalize_spec(res)
        det = self._normalize_detail(res)
        nodes_x, nodes_y, fit_x, fit_y = self._normalize_nodes(res)
        # φ mean 大数字（3σ µrad）
        if phi_mean is not None:
            self.ui.lbl_phi.setText(
                "非正交角 φ = %+.3f µrad（3σ %s µrad）"
                % (phi_mean, _fmt(phi_s3, 3)))
        else:
            self.ui.lbl_phi.setText("非正交角 φ：—（结果缺 "
                                    "measured.phi_urad）")
        # 左上：逐次 φ 趋势
        self.res_ax.clear()
        if phi:
            self.res_ax.plot(list(range(len(phi))), phi, ".-",
                             color="#4a90e2", markersize=4)
        self.res_ax.set_title("per-repeat φ: mean %s µrad / std %s µrad"
                              % (_fmt(phi_mean, 3), _fmt(phi_std, 3)),
                              fontsize=9)
        self.res_ax.set_xlabel("repeat", fontsize=8)
        self.res_ax.set_ylabel("µrad", fontsize=8)
        self.res_ax.tick_params(labelsize=8)
        # 右上：测量十字图；左下/右下：X/Y 线拟合细节（迭代 45）
        self._plot_cross(nodes_x, nodes_y, fit_x, fit_y, det,
                         (phi_mean, theta_x, theta_y))
        self._plot_line_detail(self.res_ax_xl, nodes_x, fit_x,
                               det.get("pitch"), "X")
        self._plot_line_detail(self.res_ax_yl, nodes_y, fit_y,
                               det.get("pitch"), "Y")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        # 统计表（迭代 45 起 7 行：θ_x/θ_y−90°/EXY 规范口径/U(k=2)/β/
        # 残差 RMS/MAX）
        table = self.ui.stats_table
        fill_stats_row(table, 0, ["θ_x 轴-栅格角 (µrad)", _fmt(theta_x, 3)])
        fill_stats_row(table, 1, ["θ_y−90°（轴-栅格角偏差, µrad）",
                                  _fmt(theta_y, 3)])
        fill_stats_row(table, 2, ["EXY 垂直度误差（规范口径，夹角>90°为正,"
                                  " µrad）", _fmt(exy, 3)])
        fill_stats_row(table, 3, ["扩展不确定度 U（k=2, µrad）",
                                  "—" if U_u is None else
                                  "%s（u = %s）" % (_fmt(U_u, 3),
                                                   _fmt(u_u, 3))])
        fill_stats_row(table, 4, ["β 上板倾斜（栅格旋转 ρ̂, µrad）",
                                  _fmt(beta, 3)])
        fill_stats_row(table, 5, ["线法向残差 RMS (nm)", _fmt(resid_rms, 3)])
        fill_stats_row(table, 6, ["线法向残差 MAX (nm)", _fmt(resid_max, 3)])
        # 非正交矩阵块（迭代 39 measured.matrix_l2p/matrix_p2l；
        # 旧 schema 无矩阵键留空不崩）+ 规范 6.4 数值核验行（迭代 45
        # detail.ac_check_norm）+ 比例误差行（迭代 42b
        # measured.scale_ppm_x/y；旧 schema 无键显示 —）
        m_l2p, m_p2l = self._normalize_matrix(res)
        mtxt = self._matrix_block_text(m_l2p, m_p2l)
        ac = det.get("ac_check")
        ac_txt = ("数值核验 ‖A·A⁻¹−I‖ = %s（规范 6.4，应 <1e-12）"
                  % ("—" if ac is None else "%.3g" % ac))
        self.ui.lbl_matrix.setText((mtxt + "\n" + ac_txt)
                                   if mtxt else ac_txt)
        spx, spy = self._normalize_scale(res)
        self.ui.lbl_scale.setText(self._scale_row_text(spx, spy))

    # ---------------- 写回 as_nonortho_urad（作业 done 后可用） ----------------
    def _on_run_clicked(self):
        self._apply_job_id = None
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("（作业完成后可写回机器常数）")
        self.ui.lbl_matrix.setText("")      # 矩阵块随新作业清空
        self.ui.lbl_scale.setText("比例误差 εx/εy：—（作业完成后显示）")
        super()._on_run_clicked()

    def _on_finish(self, output):
        super()._on_finish(output)
        jid = (output or {}).get("job_id")
        if jid:
            self._apply_job_id = jid
            self.ui.btn_apply.setEnabled(True)
            self.ui.lbl_apply.setText(
                "作业 #%d 已完成，可写回 %s" % (jid, ", ".join(_APPLY_FIELDS)))

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="as-nonortho-apply", daemon=True).start()

    def _apply_worker(self, jid):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid, "fields": list(_APPLY_FIELDS)},
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


def _fmt_ppm(v):
    """比例误差 ppm 显示（None → —；1 位小数带符号）。"""
    return "—" if v is None else ("%+.1f" % v)


def _fmt_mat(m):
    """2×2 矩阵显示文本（9 位有效数字；None → —）。"""
    if m is None:
        return "—"
    return "[[%s, %s], [%s, %s]]" % tuple(
        "%.9g" % v for row in m for v in row)


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = AsStageNonortho()
    test.show()
    sys.exit(myapp.exec_())
