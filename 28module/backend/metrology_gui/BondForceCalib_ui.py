# -*- coding: utf-8 -*-
"""
Filename: BondForceCalib_ui.py
Description:
    键合力校准 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    后端 routine bond_force_calib（cam 恒 up）：晶圆已上片、键合头已
    拾取芯片后，按 setpoints_n 逐点真实下压，每点采 samples×
    sample_interval_ms 力序列求 mean/3σ，线性拟合 gain/offset_n——
    写回 force_gain/force_offset_n。

    页面顶部醒目提示（橙底）：请确认：晶圆已上片、键合头已拾取芯片
    （校准将真实下压）。

    参数区：setpoints_n 文本（默认 "5,10,20,30"）+ samples（默认 20）
    + sample_interval_ms（默认 100）。
    结果区：mean_n vs setpoint_n 散点 + 拟合直线（标注 gain/offset）
    + 统计表（每 setpoint 一行 mean/3σ + gain/offset_n，measured vs
    current）+ "写回力校准"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["force_gain","force_offset_n"]）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets

# 默认力设定点（N）
DEFAULT_SETPOINTS_N = "5,10,20,30"


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 800)
        Widget.setWindowTitle("Bond Force Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Bond Force Calibration"))

        # 页面顶部醒目前提提示（橙底）
        self.lbl_prereq = QLabel(
            "请确认：晶圆已上片、键合头已拾取芯片（校准将真实下压）")
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
        # 统计表（每 setpoint 一行 mean/3σ + gain/offset_n；预留 8 点）
        self.stats_table = create_stats_table(
            ["统计量", "measured", "current"], rows=10, height=320)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回力校准")
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
            "POST /api/calibration/apply：fields=[force_gain, "
            "force_offset_n]（落盘+热生效）")
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

        # 力设定点 setpoints_n（文本，逗号分隔，默认 "5,10,20,30"）
        layout.addWidget(create_label("力设定点 setpoints"), cur_row_index, 0)
        self.setpoints = create_lineedit(
            DEFAULT_SETPOINTS_N, "逗号分隔，如 5,10,20,30")
        layout.addWidget(self.setpoints, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("N"), cur_row_index, 2)
        cur_row_index += 1

        # 每点采样数 samples（默认 20）
        layout.addWidget(create_label("每点采样数"), cur_row_index, 0)
        self.samples = create_spinbox(5, 500, 0)
        self.samples.setValue(20)
        layout.addWidget(self.samples, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 采样间隔 sample_interval_ms（默认 100）
        layout.addWidget(create_label("采样间隔"), cur_row_index, 0)
        self.sample_interval = create_spinbox(10, 60000, 0)
        self.sample_interval.setValue(100)
        layout.addWidget(self.sample_interval, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 页面说明
        hint = QLabel(
            "流程：按 setpoints 逐点真实下压到设定力，每点采 samples×"
            "间隔力序列求 mean/3σ，线性拟合 gain/offset_n（实测力 = "
            "gain×设定 + offset_n）。")
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
