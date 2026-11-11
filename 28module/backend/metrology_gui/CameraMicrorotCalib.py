# -*- coding: utf-8 -*-
"""
Filename: CameraMicrorotCalib.py
Description:
    相机微小旋转角标定（参数形态对照参考框架
    CameraTiltAngleCalibration：相机 + 模板名 + 两个拍照位 + 重复
    次数；界面三段式对照 SingleCameraAlignStability）。核心链路在
    后端作业 camera_microrot（拍照位1/2 各拍标记，纯 X 杠杆拟合
    装配微小旋转角 rot_mrad，见 metrology/ 侧实现），本页走
    job_base 的"参数下发 + 作业轮询 + 结果展示"。

    参数（迭代 34 契约）：相机（up/down/coarse）+ 模板名（必填，须
    视觉页已注册）+ 拍照位1/2 (x,y)（mm，必填常使能，各配"取当前位"
    按钮按 相机+运动台 映射运动组回填）+ move_stage（"align"/
    "wafer"，仅 down 相机下发；上相机恒对准台、粗对准相机走翻转
    模块不参与）+ repeat（默认10）/wait_ms。方法要求纯 X 两点
    （y 容差 0.01mm），间距不限——迭代 34 起后端间距/FOV/可达
    窗口硬校验全删，识别质量由所选模板与机位决定。

    结果区：逐轮 φ 趋势（µrad，标题 mean/std µrad）+ 统计表
    （N / mean mrad / mean µrad / std µrad / 3σ µrad）+
    "当前 rot_mrad（机器常数·控制器认知）"与"差值 = 实测−当前"
    只读行（迭代 39：结果 current.rot_mrad，缺省显示 —）+
    "写回 rot_mrad"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["rot_mrad"]，结果显示写回结果）+
    视场旋转建议块（迭代 38 任务 6，物理/认知分离：view_rot_deg=
    相机物理旋转角（逆时针正，图像内容反向转），rot_mrad=控制器
    认知（机器常数），两者换算相等=补偿到位）——两条二选一建议：
    ①写回 rot_mrad=带入计算修正世界坐标（反算链生效，图像不变）
    ——沿用已有写回按钮；②物理转正=直接旋转视场（图像去旋，
    rot_mrad 置 0）：视场旋转调为 cur−measured×0.0572958°——
    新增按钮，POST /api/vision/camera/config {cam, view_rot_deg}
    （限 ±5°，超限钳位并提示），成功后回读新值一并落日志。两按钮
    旁注"二选一，都做=双重补偿"。迭代 42：删"当前视场旋转"常驻
    显示（真机读不到 view_rot_deg 物理真值，GET /api/vision/cameras
    无此接口）——view_rot 改为点击"物理转正"时才惰性读取，读取
    不可用仅落日志区（不抛异常不弹窗）。
"""
import math
import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from job_base import JobRoutineBase, fetch_axes_pos, http_json
from qt_custom_ui import fill_stats_row
import CameraMicrorotCalib_ui as ui

# 逐轮 φ 序列字段候选（统一换算到 rad）
_PHI_SER_KEYS = (("phi_rad", 1.0), ("phi", 1.0), ("rot_rad", 1.0),
                 ("phi_mrad", 1e-3), ("phi_urad", 1e-6))
# mrad → ° 换算（物理转正建议：view_rot 目标=cur−measured×本系数）
_MRAD2DEG = 0.05729577951
# view_rot_deg 后端允许范围（±5°，超限钳位并提示）
_VIEW_ROT_LIM = 5.0


class CameraMicrorotCalib(JobRoutineBase):
    ROUTINE = "camera_microrot"
    TITLE = "相机微小旋转角标定"

    sig_apply_done = pyqtSignal(str)      # 写回结果（GUI 线程落标签/日志）
    sig_view_rot_done = pyqtSignal(str)   # 视场旋转建议块状态（迭代 42 起仅落日志区）

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self._apply_job_id = None
        self._last_measured_mrad = None   # 末次作业实测 rot_mrad（物理转正用）

        self.res_fig = Figure()
        self.res_ax = self.res_fig.add_subplot(111)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.ui.cb_camera.currentTextChanged.connect(
            self._sync_move_stage_enabled)
        self._sync_move_stage_enabled()
        self.ui.btn_snap_pos1.clicked.connect(
            lambda: self._snap_photo_pos(self.ui.get_pic_pos_x_1,
                                         self.ui.get_pic_pos_y_1, 1))
        self.ui.btn_snap_pos2.clicked.connect(
            lambda: self._snap_photo_pos(self.ui.get_pic_pos_x_2,
                                         self.ui.get_pic_pos_y_2, 2))
        self.ui.btn_apply.clicked.connect(self._on_apply_clicked)
        self.sig_apply_done.connect(self._on_apply_done)
        self.ui.btn_derot.clicked.connect(self._on_derot_clicked)
        self.sig_view_rot_done.connect(self._on_view_rot_done)
        self.bind_run_controls()

    # ---------------- 运动台使能/取当前位（GUI 线程） ----------------
    def _sync_move_stage_enabled(self, *_args):
        """运动台下拉仅下相机可选；上相机恒对准台（禁用），粗对准
        相机走翻转模块不参与本选择。"""
        self.ui.cb_move_stage.setEnabled(self._cur_cam() == "down")

    def _cur_move_stage(self):
        return "wafer" if "晶圆" in self.ui.cb_move_stage.currentText() \
            else "align"

    def _snap_stage_key(self):
        """取当前位用的 camera_stage_map 键：down 按运动台
        （wafer→"down" 晶圆台 / align→"align" 对准台），up/coarse
        按相机。"""
        cam = self._cur_cam()
        if cam == "down":
            return "down" if self._cur_move_stage() == "wafer" else "align"
        return cam

    def _cur_cam(self):
        t = self.ui.cb_camera.currentText()
        if "上" in t:
            return "up"
        if "粗" in t:
            return "coarse"
        return "down"

    def _snap_photo_pos(self, sb_x, sb_y, idx):
        try:
            pos = fetch_axes_pos(timeout=5)
        except Exception as e:
            self.ui.textEdit.append("[warn] 取当前位失败：%s" % e)
            return
        axes = self.config.camera_stage_map[self._snap_stage_key()]["axes"]
        vx = pos.get(axes[0])
        vy = pos.get(axes[1]) if len(axes) > 1 else None
        if vx is not None:
            sb_x.setValue(float(vx))
        if vy is not None:
            sb_y.setValue(float(vy))
        self.ui.textEdit.append(
            "拍照位%d 已回填当前位：x=%s y=%s（运动组 %s）"
            % (idx, sb_x.value(), sb_y.value(), "/".join(axes)))

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        tpl = self.ui.cb_vision_template.text().strip()
        if not tpl:
            raise ValueError("视觉模板名称必填（模板须在视觉页已注册）")
        INPUT = {
            'camera': self.ui.cb_camera.currentText(),
            'move_stage': self._cur_move_stage(),
            'template_name': tpl,
            'pos1_x': float(self.ui.get_pic_pos_x_1.value()),
            'pos1_y': float(self.ui.get_pic_pos_y_1.value()),
            'pos2_x': float(self.ui.get_pic_pos_x_2.value()),
            'pos2_y': float(self.ui.get_pic_pos_y_2.value()),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': float(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.cb_camera.setCurrentText(param_dict['camera'])
        self.ui.cb_move_stage.setCurrentText(
            "晶圆台 (wafer)" if param_dict.get('move_stage') == "wafer"
            else "对准台 (align)")
        self._sync_move_stage_enabled()
        self.ui.cb_vision_template.setText(
            param_dict.get('template_name', ""))
        self.ui.get_pic_pos_x_1.setValue(float(param_dict['pos1_x']))
        self.ui.get_pic_pos_y_1.setValue(float(param_dict['pos1_y']))
        self.ui.get_pic_pos_x_2.setValue(float(param_dict['pos2_x']))
        self.ui.get_pic_pos_y_2.setValue(float(param_dict['pos2_y']))
        self.ui.repeat_times.setValue(int(param_dict['repeat_time']))
        self.ui.wait_time.setValue(float(param_dict['wait_time']))

    # ---------------- 作业参数 ----------------
    def build_cam(self, dict_input):
        t = dict_input['camera']
        if "上" in t:
            return "up"
        if "粗" in t:
            return "coarse"
        return "down"

    def build_params(self, dict_input):
        # 迭代 34 契约：template_name/photo_pos1/photo_pos2 必填常下发；
        # move_stage 仅 down 相机下发（align 对准台 / wafer 晶圆台）
        params = {"repeat": int(dict_input['repeat_time']),
                  "wait_ms": dict_input['wait_time'],
                  "template_name": dict_input['template_name'],
                  "photo_pos1": [dict_input['pos1_x'],
                                 dict_input['pos1_y']],
                  "photo_pos2": [dict_input['pos2_x'],
                                 dict_input['pos2_y']]}
        if self.build_cam(dict_input) == "down":
            params["move_stage"] = dict_input.get('move_stage') or "align"
        return params

    # ---------------- 结果解读（多 schema 归一，GUI 线程） ----------------
    @staticmethod
    def _normalize(res):
        """返回 (phi_urad序列, mean_mrad, std_urad, s3_urad, cur_mrad)。
        后端事实源：measured.rot_mrad（mrad）+ stats 内 std/3σ(µrad)
        + 逐轮 φ(rad) 序列；序列/统计缺失时互相兜底推算。
        cur_mrad=当前 rot_mrad（机器常数·控制器认知，迭代 39）：
        结果顶层 current.rot_mrad 优先、兼容 measured.current 容器，
        缺省 None（调用方显示 —）。"""
        res = res or {}
        series = res.get("series") or {}
        st = res.get("stats") or {}
        measured = res.get("measured") or {}
        phi = []
        for key, f in _PHI_SER_KEYS:
            if series.get(key):
                phi = [v * f * 1e6 for v in series[key]]   # → µrad
                break
        mean_mrad = measured.get("rot_mrad")
        std_urad = s3_urad = None
        # stats 容器候选：stats.rot / stats.phi / stats.angle /
        # stats.rot_mrad（dict）或扁平 stats.rot_std_urad 等
        for cand in (st.get("rot"), st.get("phi"), st.get("angle"),
                     st.get("rot_mrad"), st):
            if not isinstance(cand, dict):
                continue
            if mean_mrad is None:
                mean_mrad = cand.get("mean_mrad")
            if std_urad is None:
                std_urad = cand.get("std_urad")
            if s3_urad is None:
                s3_urad = cand.get("s3_urad")
            if std_urad is None and cand.get("std_mrad") is not None:
                std_urad = cand.get("std_mrad") * 1e3
            if s3_urad is None and cand.get("s3_mrad") is not None:
                s3_urad = cand.get("s3_mrad") * 1e3
        # 兜底：由 φ 序列推算（总体口径 ddof=0，与后端统计惯例一致）
        if phi and (mean_mrad is None or std_urad is None):
            n = len(phi)
            mean_u = sum(phi) / n
            var = sum((v - mean_u) ** 2 for v in phi) / n
            if mean_mrad is None:
                mean_mrad = mean_u / 1e3
            if std_urad is None:
                std_urad = math.sqrt(var)
        if s3_urad is None and std_urad is not None:
            s3_urad = 3.0 * std_urad
        # 当前 rot_mrad（机器常数·控制器认知）：顶层 current 优先，
        # 兼容 measured.current 容器；缺省 None
        cur_mrad = None
        cur_d = res.get("current") or measured.get("current") or {}
        if isinstance(cur_d, dict):
            cur_mrad = cur_d.get("rot_mrad")
        return phi, mean_mrad, std_urad, s3_urad, cur_mrad

    def result_log_lines(self, res):
        phi, mean_mrad, std_urad, s3_urad, _cur = self._normalize(res)
        return ["rot_mrad = %s mrad（mean = %s µrad, std = %s µrad, "
                "3σ = %s µrad, N = %d）"
                % (_fmt(mean_mrad, 6),
                   None if mean_mrad is None else "%.3f"
                   % (mean_mrad * 1e3),
                   _fmt(std_urad, 3), _fmt(s3_urad, 3), len(phi))]

    # ---------------- 结果可视化（GUI 线程） ----------------
    def plot_realtime_data(self, res):
        phi, mean_mrad, std_urad, s3_urad, cur_mrad = self._normalize(res)
        self._last_measured_mrad = mean_mrad   # 物理转正建议消费
        self.res_ax.clear()
        self.res_ax.plot(list(range(len(phi))), phi, ".-")
        mean_u = None if mean_mrad is None else mean_mrad * 1e3
        self.res_ax.set_title("per-cycle φ: mean %s µrad / std %s µrad"
                              % (_fmt(mean_u, 3), _fmt(std_urad, 3)))
        self.res_ax.set_xlabel("cycle")
        self.res_ax.set_ylabel("µrad")
        self.res_fig.tight_layout()
        self.res_canvas.draw()
        fill_stats_row(self.ui.stats_table, 0, [
            len(phi), _fmt(mean_mrad, 6), _fmt(mean_u, 3),
            _fmt(std_urad, 3), _fmt(s3_urad, 3)])
        # 当前 rot_mrad（机器常数·控制器认知）+ 差值=实测−当前
        # （迭代 39；current 缺省显示 —）
        if cur_mrad is not None:
            self.ui.lbl_rot_current.setText(
                "当前 rot_mrad（机器常数·控制器认知）= %s mrad"
                % _fmt(cur_mrad, 6))
        else:
            self.ui.lbl_rot_current.setText(
                "当前 rot_mrad（机器常数·控制器认知）：—"
                "（结果缺 current.rot_mrad）")
        if cur_mrad is not None and mean_mrad is not None:
            self.ui.lbl_rot_diff.setText(
                "差值 = 实测−当前 = %+.3f mrad" % (mean_mrad - cur_mrad))
        else:
            self.ui.lbl_rot_diff.setText("差值 = 实测−当前：—")

    # ---------------- 写回 rot_mrad（作业 done 后可用） ----------------
    def _on_run_clicked(self):
        self._apply_job_id = None
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("（作业完成后可写回机器常数）")
        self.ui.btn_derot.setEnabled(False)
        self.ui.lbl_rot_current.setText(
            "当前 rot_mrad（机器常数·控制器认知）：—（作业完成后显示）")
        self.ui.lbl_rot_diff.setText("差值 = 实测−当前：—")
        super()._on_run_clicked()

    def _on_finish(self, output):
        super()._on_finish(output)
        jid = (output or {}).get("job_id")
        if jid:
            self._apply_job_id = jid
            self.ui.btn_apply.setEnabled(True)
            self.ui.lbl_apply.setText(
                "作业 #%d 已完成，可写回 rot_mrad" % jid)
        # 视场旋转建议（迭代 38）：作业完成后使能"物理转正"
        # （①写回 rot_mrad / ②物理转正 二选一）；当前 view_rot 不再
        # 常驻显示——迭代 42 起点击"物理转正"时才惰性读取
        self.ui.btn_derot.setEnabled(self._last_measured_mrad is not None)

    def _on_apply_clicked(self):
        jid = self._apply_job_id
        if not jid:
            self.ui.lbl_apply.setText("无可写回的已完成作业")
            return
        self.ui.btn_apply.setEnabled(False)
        self.ui.lbl_apply.setText("写回中…")
        threading.Thread(target=self._apply_worker, args=(jid,),
                         name="microrot-apply", daemon=True).start()

    def _apply_worker(self, jid):
        try:
            r = http_json("POST", "/api/calibration/apply",
                          {"job_id": jid, "fields": ["rot_mrad"]},
                          timeout=15)
            if not r.get("ok"):
                raise RuntimeError("%s" % (r.get("error") or r))
            applied = r.get("applied") or []
            der = r.get("derived") or {}
            msg = "已写回 %s（落盘+热生效）" % ", ".join(applied)
            if der.get("k_um_px") is not None:
                msg += "；派生 k=%s µm/px" % der["k_um_px"]
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

    # ---------------- 视场旋转建议（迭代 38 任务 6：真值/认知分离；
    # 迭代 42：删常驻显示，view_rot 点击"物理转正"时才惰性读取） ----------------
    @staticmethod
    def _fetch_view_rot_deg(cam):
        """GET /api/vision/cameras → cameras.<cam>.view_rot_deg（相机
        物理安装旋转角，°，逆时针正）。仅在点击"物理转正"时被惰性
        调用——真机无此接口，失败由调用方落日志（不抛给 UI、不弹窗）。"""
        body = http_json("GET", "/api/vision/cameras", timeout=5)
        info = (body.get("cameras") or {}).get(cam) or {}
        v = info.get("view_rot_deg")
        if v is None:
            raise RuntimeError("相机 %s 无 view_rot_deg 字段" % cam)
        return float(v)

    def _on_derot_clicked(self):
        """物理转正（建议②）：view_rot_deg 调为 cur−measured×0.0572958°
        （图像去旋，rot_mrad 置 0；限 ±5°，超限钳位并提示）。
        当前 view_rot 在此点击时才惰性读取（真机无此接口则仅落日志）。"""
        measured = self._last_measured_mrad
        if measured is None:
            self.ui.textEdit.append("物理转正：无可用实测 rot_mrad"
                                    "（先完成作业）")
            return
        cam = self._cur_cam()
        self.ui.btn_derot.setEnabled(False)
        self.ui.textEdit.append("物理转正写回中…（惰性读取当前视场旋转）")
        threading.Thread(target=self._derot_worker,
                         args=(cam, float(measured)),
                         name="microrot-derot", daemon=True).start()

    def _derot_worker(self, cam, measured_mrad):
        try:
            cur = self._fetch_view_rot_deg(cam)
        except Exception as e:
            # 真机读不到 view_rot_deg 物理真值：只落日志区，不抛异常
            # 不弹窗；按钮经 _on_view_rot_done 重新使能
            self.sig_view_rot_done.emit(
                "视场旋转读取不可用（真机无此接口）：%s" % e)
            return
        try:
            target = cur - measured_mrad * _MRAD2DEG
            note = ""
            if target > _VIEW_ROT_LIM:
                target, note = _VIEW_ROT_LIM, "（超 +5° 上限，已钳位）"
            elif target < -_VIEW_ROT_LIM:
                target, note = -_VIEW_ROT_LIM, "（超 −5° 下限，已钳位）"
            r = http_json("POST", "/api/vision/camera/config",
                          {"cam": cam, "view_rot_deg": target}, timeout=15)
            if not r.get("ok"):
                raise RuntimeError("%s" % (r.get("error") or r))
            # 成功后回读新值（同惰性路径；回读失败不影响已写回事实，
            # 不回退按钮禁用——写回已成，防二次旋转=双重补偿）
            try:
                new_v = self._fetch_view_rot_deg(cam)
                tail = "——当前 view_rot_deg = %+.4f°" % new_v
            except Exception:
                tail = "——当前值回读不可用（真机无此接口）"
            self.sig_view_rot_done.emit(
                "已物理转正：%s view_rot_deg %+.4f° → %+.4f°%s（图像去旋；"
                "机器常数 rot_mrad 须置 0，二者都做=双重补偿）%s"
                % (cam, cur, target, note, tail))
        except Exception as e:
            self.sig_view_rot_done.emit("物理转正失败：%s" % e)

    def _on_view_rot_done(self, msg):
        # 迭代 42 起仅落日志区（原"当前视场旋转"常驻显示行已删）
        self.ui.textEdit.append(msg)
        # 失败/读取不可用时允许重试（成功则按钮保持禁用防二次旋转=
        # 双重补偿）
        if msg.startswith(("物理转正失败", "视场旋转读取不可用")):
            self.ui.btn_derot.setEnabled(True)


def _fmt(v, nd):
    return None if v is None else ("%.*f" % (nd, v))


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = CameraMicrorotCalib()
    test.show()
    sys.exit(myapp.exec_())
