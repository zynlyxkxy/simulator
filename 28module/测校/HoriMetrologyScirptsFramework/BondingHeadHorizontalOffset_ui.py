# -*- coding: utf-8 -*-
from qt_custom_ui import create_combobox, create_spinbox, create_label, create_groupbox, create_lineedit

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QTableWidget, QTableWidgetItem, QProgressBar, QTextEdit, QSpacerItem, QSizePolicy
)
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont
from metrology_config import MetrologyConfig


class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.setWindowTitle("Test Manager")
        Widget.resize(1000, 750)
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("BondingHeadHorizontalOffset")
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

        # 键合起始位置
        layout.addWidget(create_label("键合起始位置"), cur_row_index, 0)
        self.bonding_start_position = create_spinbox(-20, 20, 3)
        layout.addWidget(self.bonding_start_position, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合终止位置
        layout.addWidget(create_label("键合终止位置"), cur_row_index, 0)
        self.bonding_end_position = create_spinbox(-20, 20, 3)
        layout.addWidget(self.bonding_end_position, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(1, 10000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1


        # 运动到位等待时间
        layout.addWidget(create_label("运动到位等待时间/second"), cur_row_index, 0)
        self.movement_wait_time = create_spinbox(0, 10, 0)
        layout.addWidget(self.movement_wait_time, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合头序号
        layout.addWidget(create_label("键合头:"), cur_row_index, 0)
        self.cb_bonding_head_index = create_combobox(MetrologyConfig().bonding_head_dict.keys())
        layout.addWidget(self.cb_bonding_head_index, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 创建垂直弹簧
        # 参数：宽度(固定)，最小高度，水平策略，垂直策略
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer)

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
        self.TemplateName.clear()
        for template_name in template_list:
            self.TemplateName.addItem(template_name)
