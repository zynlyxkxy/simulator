# -*- coding: utf-8 -*-
"""
Filename: FlipRepeatability_ui.py
Date: 2026/8/19
Description: 翻转重复性测试界面
"""
from qt_custom_ui import create_spinbox, create_label, create_groupbox, StatisticsTableWidget

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QSizePolicy, QSpacerItem
)
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 750)
        Widget.setWindowTitle("翻转重复性测试")
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        project_label = QtWidgets.QLabel("翻转重复性测试")
        project_label.setAlignment(QtCore.Qt.AlignCenter)
        project_label.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
            padding: 10px;
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
            color: white;
            border-radius: 5px;
        """)
        main_layout.addWidget(project_label)

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        top_layout.addWidget(self.create_result_group())
        main_layout.addLayout(top_layout)

        # 日志区域
        log_group = QtWidgets.QGroupBox("Operation Log")
        log_group.setFixedHeight(160)
        log_group.setStyleSheet("""
            QGroupBox {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #333;
            }
        """)
        log_layout = QtWidgets.QVBoxLayout()

        self.textEdit = QtWidgets.QTextEdit()
        self.textEdit.setReadOnly(True)
        self.textEdit.setFixedHeight(110)
        self.textEdit.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 8px;
                font-family: Consolas, 'Courier New', monospace;
                font-size: 9pt;
            }
        """)
        log_layout.addWidget(self.textEdit)

        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        Widget.setLayout(main_layout)

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # RY避让角度
        layout.addWidget(create_label("RY避让角度"), cur_row_index, 0)
        self.flip_ry_avoid = create_spinbox(-250, 250, 3)
        layout.addWidget(self.flip_ry_avoid, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("°"), cur_row_index, 2)
        cur_row_index += 1

        # RY拍照角度
        layout.addWidget(create_label("RY拍照角度"), cur_row_index, 0)
        self.flip_ry_photo = create_spinbox(-250, 250, 3)
        layout.addWidget(self.flip_ry_photo, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("°"), cur_row_index, 2)
        cur_row_index += 1

        # 等待时间
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 60000, 0)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(1, 100000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        layout.addItem(vertical_spacer, cur_row_index, 0)
        cur_row_index += 1

        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = create_groupbox("Result Display", 700)
        layout = QVBoxLayout()

        self.res_display = QtWidgets.QWidget()
        self.res_display.setMinimumSize(700, 400)
        layout.addWidget(self.res_display)

        self.table_widget = StatisticsTableWidget(
            ["序号", "翻转 Y/mm", "翻转 Z/mm", "翻转 Rz/deg", "翻转 RY/deg",
             "芯片 X/um", "芯片 Y/um", "芯片 Rz/deg"])
        res_layout = QVBoxLayout()
        res_layout.addWidget(self.table_widget)
        self.res_display.setLayout(res_layout)

        group.setLayout(layout)
        return group


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
