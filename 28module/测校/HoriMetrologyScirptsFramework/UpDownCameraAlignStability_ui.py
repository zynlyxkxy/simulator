# -*- coding: utf-8 -*-
from qt_custom_ui import create_combobox, create_spinbox, create_label, create_groupbox, create_lineedit

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar, QTextEdit, QSpacerItem, QSizePolicy
)
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Single Camera Alignment Stability")
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("Up and Down Camera Alignment Stability")
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

        # 相机ID
        layout.addWidget(create_label("相机1 ID"), cur_row_index, 0)
        self.camera_ID = create_combobox([])
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称
        layout.addWidget(create_label("相机1 视觉模板名称"), cur_row_index, 0)
        self.led_vision_template = create_lineedit("请输入视觉模板名称")
        layout.addWidget(self.led_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 两次拍照等待时间
        layout.addWidget(create_label("两次拍照等待时间"), cur_row_index, 0)
        self.snap_wait_time = create_spinbox(0, 300000, 0)
        layout.addWidget(self.snap_wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 相机2ID
        layout.addWidget(create_label("相机2 ID"), cur_row_index, 0)
        self.camera_2_ID = create_combobox([])
        layout.addWidget(self.camera_2_ID, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板2名称
        layout.addWidget(create_label("相机2 视觉模板名称"), cur_row_index, 0)
        self.led_cam_2_vision_template = create_lineedit("请输入视觉模板名称")
        layout.addWidget(self.led_cam_2_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(0, 100000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 两次拍照等待时间
        layout.addWidget(create_label("每轮等待时间"), cur_row_index, 0)
        self.cycle_wait_time = create_spinbox(0, 300000, 0)
        layout.addWidget(self.cycle_wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)

        group.setLayout(layout)
        return group

    def add_camera_dict(self, camera_dict):
        for key, value in camera_dict.items():
            self.camera_ID.addItem(key)
            self.camera_2_ID.addItem(key)

    def create_result_group(self):
        group = create_groupbox("Result Display", 550)
        layout = QVBoxLayout()

        self.res_display = QtWidgets.QWidget()
        self.res_display.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #2c3e50, stop:1 #34495e);
            border: 1px solid #34495e;
            border-radius: 6px;
        """)
        self.res_display.setMinimumSize(550, 450)
        layout.addWidget(self.res_display)

        group.setLayout(layout)
        return group


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    # 设置应用程序样式
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
