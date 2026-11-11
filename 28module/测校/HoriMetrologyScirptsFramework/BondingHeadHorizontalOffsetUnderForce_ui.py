# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QTableWidget, QTableWidgetItem, QProgressBar,QTextEdit, QSpacerItem , QSizePolicy
)
from PyQt5 import QtCore,QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor
from qt_custom_ui import create_spinbox, create_combobox


class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.setWindowTitle("Test Manager")
        Widget.resize(1000, 750)
        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)
        project_label = QtWidgets.QLabel("Metrology: Focus Plane Calibration")
        project_label.setAlignment(QtCore.Qt.AlignCenter)
        project_label.setStyleSheet("font-size: 12pt; padding: 5px; background-color: #f0f0f0;")
        main_layout.addWidget(project_label)
        # Top layout: Input and Result
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.create_input_group())
        top_layout.addWidget(self.create_result_group())
        main_layout.addLayout(top_layout)

        log_group = QtWidgets.QGroupBox("Operation Log")
        log_group.setFixedHeight(140)
        log_layout = QtWidgets.QVBoxLayout()

        self.textEdit = QtWidgets.QTextEdit()
        self.textEdit.setReadOnly(True)
        self.textEdit.setFixedHeight(100)
        log_layout.addWidget(self.textEdit)

        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        Widget.setLayout(main_layout)

    def paintEvent(self, event):
        painter = QPainter(self)
        gradient = QLinearGradient(0, 0, 0, self.height())

        gradient.setColorAt(0.0, QColor(240, 240, 240))
        gradient.setColorAt(0.5, QColor(225, 225, 225))
        gradient.setColorAt(1.0, QColor(210, 210, 210))

        painter.fillRect(self.rect(), gradient)

    def create_input_group(self):
        group = QGroupBox("Input")
        group.setStyleSheet("QGroupBox { background-color: rgba(255, 255, 255, 180); }")
        layout = QGridLayout()
        cur_row_index = 0

        # 键合起始位置
        layout.addWidget(QLabel("键合起始位置"), cur_row_index, 0)
        self.bonding_start_position = create_spinbox(-20, 20, 3)
        layout.addWidget(self.bonding_start_position, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合终止位置
        layout.addWidget(QLabel("键合终止位置"), cur_row_index, 0)
        self.bonding_end_position = create_spinbox(-20, 20, 3)
        layout.addWidget(self.bonding_end_position, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合终止位置
        layout.addWidget(QLabel("键合终止位置"), cur_row_index, 0)
        self.bonding_end_position = create_spinbox(-20, 20, 3)
        layout.addWidget(self.bonding_end_position, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合时间
        layout.addWidget(QLabel("键合时间"), cur_row_index, 0)
        self.bonding_duration = create_spinbox(1, 100, 3)
        layout.addWidget(self.bonding_duration, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(QLabel("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(1, 10000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 运动到位等待时间
        layout.addWidget(QLabel("运动到位等待时间"), cur_row_index, 0)
        self.movement_wait_time = create_spinbox(0, 10, 0)
        layout.addWidget(self.movement_wait_time, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合头序号
        layout.addWidget(QLabel("键合头:"), cur_row_index, 0)
        self.cb_bonding_head_index = create_combobox(['YN XN', 'YN XP', 'YP XN', 'YP XP', ])
        layout.addWidget(self.cb_bonding_head_index, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合力大小
        layout.addWidget(QLabel("键合力大小:"), cur_row_index, 0)
        self.cb_bonding_force = create_combobox(['10', '15', '20', '25', '30'])
        layout.addWidget(self.cb_bonding_force, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 创建垂直弹簧
        # 参数：宽度(固定)，最小高度，水平策略，垂直策略
        vertical_spacer = QSpacerItem(10, 30,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer)

        layout.setVerticalSpacing(8)
        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = QGroupBox("Result")
        group.setStyleSheet("QGroupBox { background-color: rgba(255, 255, 255, 180); }")
        layout = QVBoxLayout()

        self.socre_display = QtWidgets.QWidget()
        self.socre_display.setStyleSheet("background-color: #202020; border: 1px solid #777;")
        self.socre_display.setMinimumSize(600, 400)
        layout.addWidget(self.socre_display)

        group.setLayout(layout)
        return group

    def update_vision_template(self, template_list):
        self.TemplateName.clear()
        for template_name in template_list:
            self.TemplateName.addItem(template_name)
