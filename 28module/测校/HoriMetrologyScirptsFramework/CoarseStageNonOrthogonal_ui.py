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
        Widget.setWindowTitle("粗动台非正交校准")

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)
        project_label = QtWidgets.QLabel("粗动台非正交校准")
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

        # x、y拍照位置
        layout.addWidget(QLabel("x向拍照位置1(x,y)"), cur_row_index, 0)
        self.x_get_pic_pos_x_1 = self._create_spinbox(-250, 250, 3)
        self.x_get_pic_pos_y_1 = self._create_spinbox(-250, 250, 3)
        layout.addWidget(self.x_get_pic_pos_x_1, cur_row_index, 1, 1, 1)
        layout.addWidget(self.x_get_pic_pos_y_1, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1

        # x、y拍照位置
        layout.addWidget(QLabel("x向拍照位置2(x,y)"), cur_row_index, 0)
        self.x_get_pic_pos_x_2 = self._create_spinbox(-250, 250, 3)
        self.x_get_pic_pos_y_2 = self._create_spinbox(-250, 250, 3)
        layout.addWidget(self.x_get_pic_pos_x_2, cur_row_index, 1, 1, 1)
        layout.addWidget(self.x_get_pic_pos_y_2, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1
        self.x_get_pic_pos_y_2.lineEdit().setReadOnly(True)
        # X向只动X轴，位置2的y跟随位置1的y
        self.x_get_pic_pos_y_2.setValue(self.x_get_pic_pos_y_1.value())
        self.x_get_pic_pos_y_1.valueChanged.connect(self.x_get_pic_pos_y_2.setValue)

        # x、y拍照位置
        layout.addWidget(QLabel("y向拍照位置1(x,y)"), cur_row_index, 0)
        self.y_get_pic_pos_x_1 = self._create_spinbox(-250, 250, 3)
        self.y_get_pic_pos_y_1 = self._create_spinbox(-250, 250, 3)
        layout.addWidget(self.y_get_pic_pos_x_1, cur_row_index, 1, 1, 1)
        layout.addWidget(self.y_get_pic_pos_y_1, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1

        # x、y拍照位置
        layout.addWidget(QLabel("y向拍照位置2(x,y)"), cur_row_index, 0)
        self.y_get_pic_pos_x_2 = self._create_spinbox(-250, 250, 3)
        self.y_get_pic_pos_y_2 = self._create_spinbox(-250, 250, 3)
        layout.addWidget(self.y_get_pic_pos_x_2, cur_row_index, 1, 1, 1)
        layout.addWidget(self.y_get_pic_pos_y_2, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
        cur_row_index += 1
        self.y_get_pic_pos_x_2.lineEdit().setReadOnly(True)
        # Y向只动Y轴，位置2的x跟随位置1的x
        self.y_get_pic_pos_x_2.setValue(self.y_get_pic_pos_x_1.value())
        self.y_get_pic_pos_x_1.valueChanged.connect(self.y_get_pic_pos_x_2.setValue)

        # x、y理论间隔
        layout.addWidget(QLabel("x、y理论间隔"), cur_row_index, 0)
        self.x_interval = self._create_spinbox(-500, 500, 3)
        self.y_interval = self._create_spinbox(-500, 500, 3)
        layout.addWidget(self.x_interval, cur_row_index, 1, 1, 1)
        layout.addWidget(self.y_interval, cur_row_index, 2, 1, 1)
        layout.addWidget(QLabel("mm"), cur_row_index, 3)
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

        self.res_textedit = QtWidgets.QTextEdit()
        self.res_textedit.setReadOnly(True)
        self.res_textedit.setFixedWidth(600)
        layout.addWidget(self.res_textedit)

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
