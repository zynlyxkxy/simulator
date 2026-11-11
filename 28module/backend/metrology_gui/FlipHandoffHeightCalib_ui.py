# -*- coding: utf-8 -*-
"""
Filename: FlipHandoffHeightCalib_ui.py
Description:
    Flip与键合头承接高度标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条
    + Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    后端 routine flip_handoff_height（cam 恒 up）：键合头低速下探
    接触 Flip 上芯片面，力阈值判接触——输出 head_z_touch_mm（键合头
    承接高度）/ flip_surface_mm（Flip 承接面高度）/ contact_force_n。

    页面顶部醒目提示（橙底）：前提：Flip 上已有芯片（将检查 IO/工件
    状态，无芯片会报错）。

    参数区：chip_thickness_mm（默认 0.5）+ force_threshold_n（默认
    2.0）+ search_speed_mm_s（默认 0.2）。
    结果区：力-位移曲线（x=z_mm, y=force_n，标注接触点）+ 统计表
    （measured vs current）+ "写回承接高度"按钮（作业 done 后可用，
    POST /api/calibration/apply fields=["head_z_touch_mm"]）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 800)
        Widget.setWindowTitle("Flip-Head Handoff Height Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Flip-Head Handoff Height Calibration"))

        # 页面顶部醒目前提提示（橙底）
        self.lbl_prereq = QLabel(
            "前提：Flip 上已有芯片（将检查 IO/工件状态，无芯片会报错）")
        self.lbl_prereq.setAlignment(Qt.AlignCenter)
        self.lbl_prereq.setStyleSheet(
            "background-color: #e67e22; color: white; font-weight: bold; "
            "font-size: 11pt; padding: 8px; border-radius: 5px;")
        main_layout.addWidget(self.lbl_prereq)

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # 统计表（承接高度/承接面/接触力：measured vs current）
        self.stats_table = create_stats_table(
            ["统计量", "measured", "current"], rows=3, height=132)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回承接高度")
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
        self.btn_apply.setToolTip(
            "POST /api/calibration/apply：fields=[head_z_touch_mm]"
            "（落盘+热生效）")
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

        # 相机（恒 up，只读提示）
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        lbl_cam = create_label("上精对准 (up)")
        layout.addWidget(lbl_cam, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 芯片厚度 chip_thickness_mm（默认 0.5）
        layout.addWidget(create_label("芯片厚度"), cur_row_index, 0)
        self.chip_thickness = create_spinbox(0.01, 5, 3)
        self.chip_thickness.setValue(0.5)
        layout.addWidget(self.chip_thickness, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 2)
        cur_row_index += 1

        # 接触力阈值 force_threshold_n（默认 2.0）
        layout.addWidget(create_label("接触力阈值"), cur_row_index, 0)
        self.force_threshold = create_spinbox(0.1, 100, 2)
        self.force_threshold.setValue(2.0)
        layout.addWidget(self.force_threshold, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("N"), cur_row_index, 2)
        cur_row_index += 1

        # 下探速度 search_speed_mm_s（默认 0.2）
        layout.addWidget(create_label("下探速度"), cur_row_index, 0)
        self.search_speed = create_spinbox(0.01, 10, 3)
        self.search_speed.setValue(0.2)
        layout.addWidget(self.search_speed, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("mm/s"), cur_row_index, 2)
        cur_row_index += 1

        # 页面说明
        hint = QLabel(
            "流程：键合头移至 Flip 上方后按 search_speed 低速下探，力达"
            " force_threshold 判接触——head_z_touch_mm=接触时键合头 Z"
            "（承接高度，写回值），flip_surface_mm=head_z_touch 扣除芯片"
            "厚度推算的 Flip 承接面高度。series 带 t_s/z_mm/force_n/"
            "chuck_vac_kpa/chip_vac_kpa 采样序列。")
        hint.setStyleSheet("color: #888; font-size: 9pt;")
        hint.setWordWrap(True)
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


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
