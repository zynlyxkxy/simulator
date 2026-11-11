# -*- coding: utf-8 -*-
from qt_custom_ui import (create_spinbox, create_label, create_groupbox, create_lineedit, create_combobox,
                          StatisticsTableWidget, CustomTableWidget)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar,QTextEdit, QSpacerItem , QSizePolicy
)
from PyQt5 import QtCore,QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont
from metrology_config import MetrologyConfig



class Ui_Widget(QWidget):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1000, 750)
        Widget.setWindowTitle("键合头旋转中心标定")
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("键合头旋转中心标定")
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

        # 检测点信息
        detect_point_label = create_label("检测点旋转角度（°）:")
        layout.addWidget(detect_point_label, cur_row_index, 0, 1, 1)
        layout.addItem(QSpacerItem(10, 10, QSizePolicy.Expanding, QSizePolicy.Minimum), cur_row_index, 2, 1, 1)
        cur_row_index += 1
        self.detect_point_info_table_widget = CustomTableWidget(['旋转角度/°'], {0: 'float'})
        layout.addWidget(self.detect_point_info_table_widget, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 相机选择
        camera_label = create_label("相机")
        layout.addWidget(camera_label, cur_row_index, 0)
        self.cb_camera = create_combobox(['下精对准_XP', '下精对准_XN'])
        layout.addWidget(self.cb_camera, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 键合头选择
        head_label = create_label("键合头")
        layout.addWidget(head_label, cur_row_index, 0)
        self.cb_bonding_head = create_combobox(list(MetrologyConfig().bonding_head_dict.keys()))
        layout.addWidget(self.cb_bonding_head, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(1, 100000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        cur_row_index += 1

        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = create_groupbox("Result Display", 600)
        layout = QVBoxLayout()

        self.res_display = QtWidgets.QWidget()

        self.res_display.setMinimumSize(600, 400)
        layout.addWidget(self.res_display)

        self.table_widget = StatisticsTableWidget(["序号", "旋转中心 x/pixel", "旋转中心 y/pixel", "偏心半径/um"])
        res_fig_layout = QVBoxLayout()
        res_fig_layout.addWidget(self.table_widget)
        self.res_display.setLayout(res_fig_layout)

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
