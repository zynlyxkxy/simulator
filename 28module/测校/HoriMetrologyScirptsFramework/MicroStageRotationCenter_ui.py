# -*- coding: utf-8 -*-
from qt_custom_ui import create_combobox, create_spinbox, create_label, create_groupbox, create_lineedit

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar,QTextEdit, QSpacerItem , QSizePolicy
)
from PyQt5 import QtCore,QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont
from qt_custom_ui import StatisticsTableWidget



class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1000, 750)
        Widget.setWindowTitle("Test Manager")
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("Camera Stability")
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
        layout.addWidget(create_label("相机ID"), cur_row_index, 0)
        self.camera_ID = create_combobox([])
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit("视觉模板名称")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1)
        cur_row_index += 1

        # 粗动台x、y拍照位置
        layout.addWidget(create_label("粗动台拍照位置1(x,y)"), cur_row_index, 0)
        self.coarse_stage_get_pic_pos_x_1 = create_spinbox(-250, 250, 3)
        self.coarse_stage_get_pic_pos_y_1 = create_spinbox(-250, 250, 3)
        layout.addWidget(self.coarse_stage_get_pic_pos_x_1, cur_row_index, 1, 1, 1)
        layout.addWidget(self.coarse_stage_get_pic_pos_y_1, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1

        # 粗动台x、y拍照位置
        layout.addWidget(create_label("粗动台拍照位置2(x,y)"), cur_row_index, 0)
        self.coarse_stage_get_pic_pos_x_2 = create_spinbox(-250, 250, 3)
        self.coarse_stage_get_pic_pos_y_2 = create_spinbox(-250, 250, 3)
        layout.addWidget(self.coarse_stage_get_pic_pos_x_2, cur_row_index, 1, 1, 1)
        layout.addWidget(self.coarse_stage_get_pic_pos_y_2, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1

        # 水平向微动台Rz位置1
        layout.addWidget(create_label("水平向微动台Rz位置1"), cur_row_index, 0)
        self.horizontal_micro_stage_rz_1 = create_spinbox(-250, 250, 3)
        layout.addWidget(self.horizontal_micro_stage_rz_1, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("urad"), cur_row_index, 2)
        cur_row_index += 1

        # 水平向微动台Rz位置2
        layout.addWidget(create_label("水平向微动台Rz位置2"), cur_row_index, 0)
        self.horizontal_micro_stage_rz_2 = create_spinbox(-250, 250, 3)
        layout.addWidget(self.horizontal_micro_stage_rz_2, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("urad"), cur_row_index, 2)
        cur_row_index += 1

        # 创建垂直弹簧
        # 参数：宽度(固定)，最小高度，水平策略，垂直策略
        vertical_spacer = QSpacerItem(10, 30,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        cur_row_index += 1

        group.setLayout(layout)
        return group

    def add_camera_dict(self, camera_dict):
        for key, value in camera_dict.items():
            self.camera_ID.addItem(key)

    def create_result_group(self):
        group = create_groupbox("Result Display", 600)
        layout = QVBoxLayout()

        self.res_display = QtWidgets.QWidget()
        self.res_display.setStyleSheet("""
            border: 1px solid #34495e;
            border-radius: 6px;
        """)
        self.res_display.setMinimumSize(600, 400)
        layout.addWidget(self.res_display)

        self.table_widget = StatisticsTableWidget(["序号", "中心点 x", "中心点 y"])
        res_fig_layout = QVBoxLayout()
        res_fig_layout.addWidget(self.table_widget)
        self.res_display.setLayout(res_fig_layout)

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
