# -*- coding: utf-8 -*-
"""
Filename: AsStageNonortho_ui.py
Description:
    对准台非正交测校 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    迭代 37 契约（栅格跳测法）：后端 routine as_stage_nonortho（cam=down）
    ——以晶圆 die 阵列为正交基准尺，沿单轴按标记间距跳测 K 点，逐点
    识别标记求世界偏移拟合方向角，φ=两线角差−90°（杠杆臂=(K−1)×pitch）。
    迭代 38：十字对称跳测（K 必须奇数，节点=center±(K−1)/2 步，
    k_points spinbox singleStep=2）；θ_y 改偏差口径（wrap(θ_y−π/2)，
    µrad 小数值）；series 新增 nodes_x/nodes_y + fit_x/fit_y（末循环
    节点散点与拟合直线，结果区下图）。

    参数区：视觉模板名称（必填）+ 基准轴 ref_axis（X 基准/Y 基准下拉，
    默认 X；两基准互为相反数，写回恒为 X 基准值）+ 扫描中心 (x,y)
    （"指定中心点"勾选才下发 center_xy，缺省=当前位；配"取当前位"
    按钮按 AS_X/AS_Y 回填）+ k_points（3~21 奇数 默认5）+ pitch_mm
    （0.1~50 默认 20=处方 auto 值，启动时 GET /api/recipe 预填，失败
    回退 20.0）+ frames_per_point（1~10 默认3）+ repeat（2~10 默认3）+
    wait_ms（默认10）。
    结果区：φ mean 大数字（3σ µrad）+ 两个子图（上=逐次 φ 趋势；
    下=末循环节点散点 + 拟合直线）+ θ_x/θ_y−90° 只读行 + 残差
    RMS/MAX 行（弓形诊断）+ "写回 as_nonortho_urad"按钮（作业 done
    后可用，POST /api/calibration/apply fields=["as_nonortho_urad"]）。
    迭代 39：统计表下方新增非正交矩阵块（measured.matrix_l2p=
    逻辑→物理矩阵 A（指令→实际）/ matrix_p2l=物理→逻辑矩阵 A⁻¹
    （补偿前馈），2×2 文本 9 位有效数字；旧 schema 无矩阵键留空
    不崩）。迭代 42b：矩阵升级为全矩阵（[col_x|col_y]=各线步进向量
    /pitch 实测像，列模长=ε），矩阵块下方新增比例误差只读行
    （measured.scale_ppm_x/y，εx/εy ppm；旧 schema 无键显示 —）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel,
    QCheckBox)
from PyQt5 import QtWidgets

# 页面说明文案（栅格跳测法口径，迭代 37 契约；迭代 38 十字对称）
METHOD_HINT = (
    "方法口径（栅格跳测法）：以晶圆 die 阵列为正交基准尺，十字跳测"
    "（中心±(K−1)/2 步，K 须奇数），逐点识别标记求世界偏移拟合方向角，"
    "φ = 两线角差 − 90°（杠杆臂 = (K−1)×pitch）。X 基准 = Y 导轨相对 X "
    "的偏角（写回值，与机器常数 as_y_rail_dir_urad 语义一致）；Y 基准 = "
    "其负值（交叉核对）；相机安装旋转在角差中抵消。θ_y−90° = 轴-栅格角"
    "偏差（µrad 小数值）。线法向残差 RMS/MAX = 弓形诊断。点间距默认 = "
    "处方 wafer_sites 间距（启动时自动预填）。")


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1120, 820)
        Widget.setWindowTitle("Align Stage Non-Orthogonality")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Align Stage Non-Orthogonality"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # φ mean 大数字（图上方；作业完成后由逻辑层填值）
        self.lbl_phi = QLabel("非正交角 φ：—（作业完成后显示）")
        self.lbl_phi.setAlignment(Qt.AlignCenter)
        self.lbl_phi.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #2c3e50; "
            "padding: 4px;")
        result_group.layout().insertWidget(0, self.lbl_phi)
        # 统计表（迭代 45 起 7 行：θ_x/θ_y−90°/EXY 规范口径/U(k=2)/β/
        # 线法向残差 RMS/MAX）
        self.stats_table = create_stats_table(
            ["统计量", "数值"], rows=7, height=250)
        result_group.layout().addWidget(self.stats_table)
        # 非正交矩阵块（迭代 39：measured.matrix_l2p/matrix_p2l 两个
        # 2×2 文本块，9 位有效数字；旧 schema 无矩阵键留空不崩）
        self.lbl_matrix = QLabel("")
        self.lbl_matrix.setStyleSheet(
            "color: #2c3e50; font-family: Consolas, 'Courier New', "
            "monospace; font-size: 9pt; padding: 2px;")
        self.lbl_matrix.setWordWrap(True)
        result_group.layout().addWidget(self.lbl_matrix)
        # 比例误差行（迭代 42b：measured.scale_ppm_x/y，矩阵块下方只读；
        # 旧 schema 无键显示 —）
        self.lbl_scale = QLabel("比例误差 εx/εy：—（作业完成后显示）")
        self.lbl_scale.setStyleSheet(
            "color: #2c3e50; font-size: 10pt; padding: 2px;")
        self.lbl_scale.setWordWrap(True)
        result_group.layout().addWidget(self.lbl_scale)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回 as_nonortho_urad")
        self.btn_apply.setEnabled(False)
        self.btn_apply.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 18px;
                font-weight: bold;
            }
            QPushButton:hover { background: #357abd; }
            QPushButton:disabled { background: #b0c4de; }
        """)
        self.lbl_apply = QLabel("（作业完成后可写回机器常数）")
        self.lbl_apply.setStyleSheet("color: #666;")
        apply_row.addWidget(self.btn_apply)
        apply_row.addWidget(self.lbl_apply, 1)
        result_group.layout().addLayout(apply_row)
        top_layout.addWidget(result_group)
        main_layout.addLayout(top_layout)

        # 日志区域
        log_group, self.textEdit = create_log_group(160)
        main_layout.addWidget(log_group)

        Widget.setLayout(main_layout)

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # 视觉模板名称（必填，须已在视觉页注册）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.template_name = create_lineedit(
            "", "必填：模板须在视觉页已注册")
        layout.addWidget(self.template_name, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 基准轴 ref_axis（迭代 37）：X 基准=Y 导轨相对 X 偏角（写回值），
        # Y 基准=其负值（交叉核对）；两基准互为相反数
        layout.addWidget(create_label("基准轴"), cur_row_index, 0)
        self.cb_ref_axis = create_combobox(["X 基准 (x)", "Y 基准 (y)"])
        self.cb_ref_axis.setToolTip(
            "基准轴：X 基准 = Y 导轨相对 X 的偏角（与机器常数 "
            "as_y_rail_dir_urad 语义一致，写回恒为 X 基准值）；"
            "Y 基准 = 其负值（交叉核对用）。两基准互为相反数。")
        layout.addWidget(self.cb_ref_axis, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 扫描中心 (x, y)（"指定中心点"勾选才下发 center_xy）
        layout.addWidget(create_label("扫描中心 (x, y)"), cur_row_index, 0)
        self.center_x = create_spinbox(-250, 250, 3)
        self.center_y = create_spinbox(-250, 250, 3)
        layout.addWidget(self.center_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.center_y, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1

        # 指定中心点勾选 + 取当前位
        self.chk_center = QCheckBox(
            "指定中心点（缺省 = 当前位，不下发 center_xy）")
        self.chk_center.setStyleSheet("color: #333;")
        layout.addWidget(self.chk_center, cur_row_index, 0, 1, 2)
        self.btn_snap = self._make_snap_button()
        layout.addWidget(self.btn_snap, cur_row_index, 2, 1, 1)
        cur_row_index += 1

        # 每线点数 k_points（3~21 奇数，默认 5；迭代 38 十字对称跳测：
        # 节点=center±(K−1)/2 步，K 须奇数，singleStep=2 保证）
        layout.addWidget(create_label("每线点数 K"), cur_row_index, 0)
        self.k_points = create_spinbox(3, 21, 0)
        self.k_points.setValue(5)
        self.k_points.setSingleStep(2)
        self.k_points.setToolTip(
            "k_points：[3,21] 奇数（步进 2 保证），默认 5；十字跳测"
            "（中心±(K−1)/2 步），杠杆臂=(K−1)×pitch")
        layout.addWidget(self.k_points, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 点间距 pitch_mm（0.1~50，默认 20=处方 wafer_sites auto 间距；
        # 迭代 37 栅格跳测法按处方间距跳测，后端校验与处方一致；
        # 启动时 GET /api/recipe 预填，失败回退 20.0）
        layout.addWidget(create_label("点间距"), cur_row_index, 0)
        self.pitch_mm = create_spinbox(0.1, 50, 3)
        self.pitch_mm.setValue(20.0)
        self.pitch_mm.setToolTip(
            "pitch_mm：[0.1,50] mm，默认 = 处方 wafer_sites 间距"
            "（启动时自动预填；auto=size+gap，manual=pitch_x/y_mm）；"
            "后端校验与处方一致，不一致 400。杠杆臂=(K−1)×pitch")
        layout.addWidget(self.pitch_mm, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 2)
        cur_row_index += 1

        # 每点拍照帧数 frames_per_point（1~10，默认 3）
        layout.addWidget(create_label("每点帧数"), cur_row_index, 0)
        self.frames_per_point = create_spinbox(1, 10, 0)
        self.frames_per_point.setValue(3)
        layout.addWidget(self.frames_per_point, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 重复次数 repeat（2~10，默认 3；整轮 φ mean/3σ）
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(2, 10, 0)
        self.repeat_times.setValue(3)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间 wait_ms（默认 10）
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 60000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 页面说明（方法口径）
        hint = QLabel(METHOD_HINT)
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #888; font-size: 11px;")
        hint.setAlignment(Qt.AlignLeft)
        layout.addWidget(hint, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 运行控制（开始/中断 + 进度条）
        btn_layout = QHBoxLayout()
        self.btn_run = create_run_button("开始")
        self.btn_interrupt = create_stop_button("中断")
        self.btn_interrupt.setEnabled(False)
        btn_layout.addWidget(self.btn_run)
        btn_layout.addWidget(self.btn_interrupt)
        layout.addLayout(btn_layout, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                background-color: white;
                text-align: center;
                min-height: 16px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.progress_bar, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,
                                      QSizePolicy.Expanding)
        layout.addItem(vertical_spacer, cur_row_index, 0)

        group.setLayout(layout)
        return group

    @staticmethod
    def _make_snap_button():
        btn = QPushButton("取当前位")
        btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #357abd;
                border: 1px solid #4a90e2;
                border-radius: 4px;
                padding: 5px 12px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #eaf3fd; }
        """)
        return btn


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
