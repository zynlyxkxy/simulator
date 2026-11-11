# -*- coding: utf-8 -*-
from qt_custom_ui import create_combobox, create_spinbox, create_label, create_groupbox, create_lineedit

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont
from PyQt5.QtWidgets import (
    QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QSpacerItem, QSizePolicy, QTextEdit
)

import ChipPositionShow
from AlgorithmServer.common_algorithm import create_chip_position, ega_nominal_list_to_real

import numpy as np
import json


class Ui_Widget(QWidget):
    sig_move_to_coarse_stage_position = pyqtSignal(list)
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.setWindowTitle("Optimal Focal Calibration")
        Widget.resize(1000, 750)
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("Metrology: Focus Plane Calibration")
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

        self.textEdit = QTextEdit()
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
        self.Widget = Widget
        self.wafer_viewer = None

        self.child = None

        self.ega_param = [-49, 188, 1, 1, 0, 0]
        try:
            with open(r"D:\App_Data\ProcessData\egaResult.json", 'r', encoding='utf-8') as file:
                ega_data = json.load(file)
                self.ega_param = ega_data['egaSixParams']
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            # 如果文件不存在或解析失败，使用默认值
            pass

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # 检测位置X
        detect_x_label = create_label("检测坐标X")
        layout.addWidget(detect_x_label, cur_row_index, 0)
        self.detect_position_X = create_spinbox(-300, 300, 3)
        self.detect_position_X.setEnabled(False)
        layout.addWidget(self.detect_position_X, cur_row_index, 1, 1, 1)
        cur_row_index += 1
        
        # 检测位置Y
        detect_y_label = create_label("检测坐标Y")
        layout.addWidget(detect_y_label, cur_row_index, 0)
        self.detect_position_Y = create_spinbox(-300, 300, 3)
        self.detect_position_Y.setEnabled(False)
        layout.addWidget(self.detect_position_Y, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 选择芯片按钮
        self.btn_select_point = QPushButton('选择芯片位置')
        self.btn_select_point.setStyleSheet("""
            QPushButton {
                border: 1px solid #4a90e2;
                border-radius: 4px;
                padding: 6px 12px;
                background-color: #4a90e2;
                color: white;
                font-weight: 500;
            }
            QPushButton:hover {
                background-color: #357abd;
            }
            QPushButton:pressed {
                background-color: #2c68a3;
            }
        """)
        layout.addWidget(self.btn_select_point, cur_row_index, 1, 1, 1)
        cur_row_index += 1
        self.btn_select_point.clicked.connect(self.open_chip_window)

        # 相机种类
        camera_label = create_label("相机ID")
        layout.addWidget(camera_label, cur_row_index, 0)
        self.camera_ID = create_spinbox(0, 5, 0)
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 焦面开始搜索位置
        z_start_label = create_label("焦面开始搜索位置")
        layout.addWidget(z_start_label, cur_row_index, 0)
        self.z_start = create_lineedit("-1.0")
        layout.addWidget(self.z_start, cur_row_index, 1)
        cur_row_index += 1

        # 焦面结束搜索位置
        z_end_label = create_label("焦面结束搜索位置")
        layout.addWidget(z_end_label, cur_row_index, 0)
        self.z_end = create_lineedit("1.0")
        layout.addWidget(self.z_end, cur_row_index, 1)
        cur_row_index += 1

        # 搜索步长
        step_label = create_label("搜索步长")
        layout.addWidget(step_label, cur_row_index, 0)
        self.step = create_lineedit("0.0500")
        layout.addWidget(self.step, cur_row_index, 1)
        cur_row_index += 1

        # 最大搜索次数
        max_iter_label = create_label("搜索次数")
        layout.addWidget(max_iter_label, cur_row_index, 0)
        self.max_iterations = create_lineedit("5.00")
        layout.addWidget(self.max_iterations, cur_row_index, 1)
        cur_row_index += 1

        # 容差
        tolerance_label = create_label("容差")
        layout.addWidget(tolerance_label, cur_row_index, 0)
        self.tolerance = create_lineedit("0.0500")
        layout.addWidget(self.tolerance, cur_row_index, 1)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)

        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = create_groupbox("Result Display", 600)
        layout = QVBoxLayout()

        self.socre_display = QtWidgets.QWidget()
        self.socre_display.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #2c3e50, stop:1 #34495e);
            border: 1px solid #34495e;
            border-radius: 6px;
        """)
        self.socre_display.setMinimumSize(600, 400)
        layout.addWidget(self.socre_display)

        group.setLayout(layout)
        return group

    def update_vision_template(self, template_list):
        # 此方法可能需要根据实际情况实现
        pass

    def set_wafer_chip_info(self, wafer_radius, chip_width, chip_height, spacing, ega_param):
        self.ega_param = ega_param

    def open_chip_window(self):
        # 如果已经打开就不再重复创建
        if self.child is not None:
            self.child.show()
            self.child.raise_()
            self.child.activateWindow()
            return
        self.wafer_radius = 8 * 25.4 / 2 - 3  # 12英寸晶圆减去3mm边缘
        self.chip_width = 10  # 芯片宽度
        self.chip_height = 10  # 芯片高度
        self.spacing = 0.08  # 芯片间距
        # self.ega_param = []
        chip_list = create_chip_position(self.wafer_radius, self.chip_width, self.chip_height, self.spacing)
        chip_list_in_stage = ega_nominal_list_to_real(chip_list, self.ega_param)
        chip_list = np.hstack([np.array(chip_list), chip_list_in_stage])

        self.child = ChipPositionShow.WaferViewer(self.wafer_radius, chip_list, self.chip_width, self.chip_height)
        self.child.setAttribute(Qt.WA_DeleteOnClose, True)

        self.child.sig_selected_points.connect(self.set_new_chip_mark_position)
        self.child.sig_move_to_mark_position.connect(self.emit_move_coarse_stage_to_position)

        self.child.destroyed.connect(lambda: setattr(self, 'child', None))
        self.child.show()

    def close_chip_window(self):
        if self.child is not None and self.child.isVisible():
            self.child.close()
            self.child = None

    def set_new_chip_mark_position(self, chips_info):
        if chips_info is not None and len(chips_info) > 0:
            self.detect_position_X.setValue(chips_info[0]['real_center_x'])
            self.detect_position_Y.setValue(chips_info[0]['real_center_y'])

    def emit_move_coarse_stage_to_position(self, position):
        self.sig_move_to_coarse_stage_position.emit(position)
