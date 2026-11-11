# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar,QTextEdit, QSpacerItem , QSizePolicy
)
from PyQt5 import QtCore,QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor



class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1000, 750)
        self.setWindowTitle("Test Manager")

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)
        project_label = QtWidgets.QLabel("EGA")
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

    def create_input_group(self):
        group = QGroupBox("Input")
        group.setStyleSheet("QGroupBox { background-color: rgba(255, 255, 255, 180); }")
        layout = QGridLayout()

        cur_row_index = 0

        # wafer 半径
        layout.addWidget(QLabel("wafer 半径"), cur_row_index, 0)
        self.wafer_radius = self._create_spinbox(0, 500.0)
        layout.addWidget(self.wafer_radius, cur_row_index, 1, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 2)
        cur_row_index += 1

        # chip 宽度
        layout.addWidget(QLabel("chip 宽度"), cur_row_index, 0)
        self.chip_width = self._create_spinbox(0, 500.0)
        layout.addWidget(self.chip_width, cur_row_index, 1, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 2)
        cur_row_index += 1

        # chip 高度
        layout.addWidget(QLabel("chip 高度"), cur_row_index, 0)
        self.chip_height = self._create_spinbox(0, 500.0)
        layout.addWidget(self.chip_height, cur_row_index, 1, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 2)
        cur_row_index += 1

        # chip 间距
        layout.addWidget(QLabel("chip 间距"), cur_row_index, 0)
        self.chip_spacing = self._create_spinbox(0, 500.0)
        layout.addWidget(self.chip_spacing, cur_row_index, 1, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 2)
        cur_row_index += 1

        # x y偏移
        layout.addWidget(QLabel("x y偏移"), cur_row_index, 0)
        self.offset_x = self._create_spinbox(-500, 500.0)
        layout.addWidget(self.offset_x, cur_row_index, 1, 1, 1)
        self.offset_y = self._create_spinbox(-500, 500.0)
        layout.addWidget(self.offset_y, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1

        # ega 参数
        layout.addWidget(QLabel("ega 参数"), cur_row_index, 0)
        self.ega_param_1 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_1, cur_row_index, 1, 1, 1)
        self.ega_param_2 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_2, cur_row_index, 2, 1, 1)
        self.ega_param_3 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_3, cur_row_index, 3, 1, 1)
        cur_row_index += 1

        self.ega_param_4 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_4, cur_row_index, 1, 1, 1)
        self.ega_param_5 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_5, cur_row_index, 2, 1, 1)
        self.ega_param_6 = self._create_spinbox(-500, 500.0, 6)
        layout.addWidget(self.ega_param_6, cur_row_index, 3, 1, 1)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(QLabel("重复次数"), cur_row_index, 0)
        self.repeat_time = self._create_spinbox(1, 1000, 0)
        layout.addWidget(self.repeat_time, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 相机ID
        layout.addWidget(QLabel("相机ID"), cur_row_index, 0)
        self.camera_ID = self._create_spinbox(0, 5, 0)
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 视觉模板名称
        layout.addWidget(QtWidgets.QLabel("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = QLineEdit()
        # self.cb_vision_template.addItems()
        # self.cb_vision_template.setCurrentIndex(0)
        layout.addWidget(self.cb_vision_template, cur_row_index, 1)
        cur_row_index += 1

        # 光谱共焦到相机位置偏移
        layout.addWidget(QLabel("光谱共焦到相机位置偏移"), cur_row_index, 0)
        self.fls_to_camera_offset_x = self._create_spinbox(-500, 500.0)
        layout.addWidget(self.fls_to_camera_offset_x, cur_row_index, 1, 1, 1)
        self.fls_to_camera_offset_y = self._create_spinbox(-500, 500.0)
        layout.addWidget(self.fls_to_camera_offset_y, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1

        # 旋转值
        layout.addWidget(QLabel("旋转值"), cur_row_index, 0)
        self.rotation_angle = self._create_spinbox(-500, 500.0)
        layout.addWidget(self.rotation_angle, cur_row_index, 1, 1, 1)
        layout.addWidget(QLabel("urad"), cur_row_index, 2)
        cur_row_index += 1

        # 创建垂直弹簧
        # 参数：宽度(固定)，最小高度，水平策略，垂直策略
        vertical_spacer = QSpacerItem(10, 30,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        cur_row_index += 1

        layout.setVerticalSpacing(8)
        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = QGroupBox("Result")
        group.setStyleSheet("QGroupBox { background-color: rgba(255, 255, 255, 180); }")
        layout = QVBoxLayout()

        self.res_text = QtWidgets.QTextEdit()
        self.res_text.setReadOnly(True)
        layout.addWidget(self.res_text)

        group.setLayout(layout)
        return group

    def _create_spinbox(self, min_val, max_val, decimals=3):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setDecimals(decimals)
        spinbox.setButtonSymbols(QAbstractSpinBox.NoButtons)
        spinbox.setStyleSheet("""
            QDoubleSpinBox {
                padding: 2px;
                border: 1px solid #c0c0c0;
                border-radius: 3px;
                min-width: 120px;
            }
        """)
        return spinbox



if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
