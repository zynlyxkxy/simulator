# -*- coding: utf-8 -*-
"""
Filename: DieTransferRepeat_ui.py
Description:
    芯片交接重复性 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+分段统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_title_label, create_log_group, create_result_group,
    create_run_button, create_stop_button, create_stats_table,
    create_lineedit, apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Die Transfer Repeatability")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(create_title_label("Die Transfer Repeatability"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(550)
        # 分段统计表（flip/head/back 各一行）
        self.stats_table = create_stats_table(
            ["段", "X 3σ (µm)", "Y 3σ (µm)", "col 3σ (px)"],
            rows=3, height=120)
        result_group.layout().addWidget(self.stats_table)
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

        # 视觉模板（留空=机器常数 vision_marks 默认，启动时从后端预填当前值；
        # 模板须在视觉页已注册，否则后端 start 400）
        layout.addWidget(create_label("粗对准相机模板"), cur_row_index, 0)
        self.cb_tpl_coarse = create_lineedit(
            "", "留空=机器常数默认（coarse 相机，flip/back 段 die 背面特征）")
        layout.addWidget(self.cb_tpl_coarse, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        layout.addWidget(create_label("上相机模板"), cur_row_index, 0)
        self.cb_tpl_up = create_lineedit(
            "", "留空=机器常数默认（up 相机，head 段键合头上 die 角标）")
        layout.addWidget(self.cb_tpl_up, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(3, 1000, 0)
        self.repeat_times.setValue(10)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 60000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 流程说明
        hint = QLabel("首循环取片一次（拾取头有余片则沿用）；每循环：交接到键合头\n"
                      "→回翻（循环内不放回蓝膜）；粗对准看翻转段（粗对准模板）、\n"
                      "上相机看键合头段（上相机模板），分段统计世界坐标散布。需蓝膜\n"
                      "在位；模板留空=机器常数 vision_marks 默认，须在视觉页已注册。")
        hint.setStyleSheet("color: #666; font-size: 8.5pt;")
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
