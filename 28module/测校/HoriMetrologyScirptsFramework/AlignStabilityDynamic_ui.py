# -*- coding: utf-8 -*-
from qt_custom_ui import create_combobox, create_spinbox, create_label, create_groupbox

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox, QStackedWidget,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar, QTextEdit, QSpacerItem, QSizePolicy
)
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QLinearGradient, QPainter, QColor, QFont
import ChipPositionShow


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Camera Alignment Stability Test")
        Widget.setStyleSheet("background-color: #f5f7fa;")

        # 设置全局字体
        font = QFont("Microsoft YaHei", 9)
        Widget.setFont(font)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # 标题区域
        project_label = QtWidgets.QLabel("Camera Alignment Stability (Dynamic)")
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

        self.cb_stage_move_type.currentIndexChanged.connect(self.stacked_widget.setCurrentIndex)

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # 相机ID
        camera_label = create_label("相机")
        layout.addWidget(camera_label, cur_row_index, 0)
        # self.camera_ID = create_spinbox(0, 5, 0)
        self.camera_ID = create_combobox([])
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称
        vision_label = create_label("视觉模板名称")
        layout.addWidget(vision_label, cur_row_index, 0)
        self.cb_vision_template = QLineEdit()
        self.cb_vision_template.setPlaceholderText("请输入视觉模板名称")
        self.cb_vision_template.setStyleSheet("""
            QLineEdit {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 6px 10px;
                background-color: white;
                min-width: 200px;
            }
            QLineEdit:hover {
                border-color: #4a90e2;
            }
            QLineEdit:focus {
                border-color: #4a90e2;
                outline: none;
            }
        """)
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数
        repeat_label = create_label("重复次数")
        layout.addWidget(repeat_label, cur_row_index, 0)
        self.repeat_times = create_spinbox(0, 100000, 0)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间
        wait_label = create_label("等待时间")
        layout.addWidget(wait_label, cur_row_index, 0)
        self.wait_time = create_spinbox(0, 100000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 运动类型
        move_label = create_label("运动类型")
        layout.addWidget(move_label, cur_row_index, 0)
        self.cb_stage_move_type = create_combobox(['粗动台', '键合臂', '翻转模块'])
        layout.addWidget(self.cb_stage_move_type, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 运动参数区域
        param_label = QLabel("运动参数")
        param_label.setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;")
        layout.addWidget(param_label, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        self.stacked_widget = QStackedWidget()
        self.stacked_widget.setStyleSheet("""
            QStackedWidget {
                background-color: #f8f9fa;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                padding: 10px;
            }
        """)
        self.create_coarse_stage_page()
        self.create_bonding_arm_page()
        self.create_flip_arm_page()
        layout.addWidget(self.stacked_widget, cur_row_index, 0, 1, 3)

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index + 1, 0)

        group.setLayout(layout)
        return group

    def create_result_group(self):
        group = QGroupBox("Result Display")
        group.setStyleSheet("""
            QGroupBox {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #333;
            }
        """)
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

    def create_coarse_stage_page(self):
        page = QWidget()
        layout = QGridLayout(page)
        layout.setVerticalSpacing(10)
        cur_row_index = 0

        # 粗动台x、y起始位置
        start_label = QLabel("粗动台起始位置(x,y)")
        start_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(start_label, cur_row_index, 0)
        self.coarse_stage_start_x = create_spinbox(-250, 250, 3)
        self.coarse_stage_start_y = create_spinbox(-250, 250, 3)
        self.coarse_stage_start_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        self.coarse_stage_start_y.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.coarse_stage_start_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.coarse_stage_start_y, cur_row_index, 2, 1, 1)
        unit_label = QLabel("mm")
        unit_label.setStyleSheet("color: #666;")
        layout.addWidget(unit_label, cur_row_index, 3)
        cur_row_index += 1

        # 粗动台x、y拍照位置
        pic_label = QLabel("粗动台拍照位置(x,y)")
        pic_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(pic_label, cur_row_index, 0)
        self.coarse_stage_get_pic_pos_x = create_spinbox(-250, 250, 3)
        self.coarse_stage_get_pic_pos_y = create_spinbox(-250, 250, 3)
        self.coarse_stage_get_pic_pos_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        self.coarse_stage_get_pic_pos_y.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.coarse_stage_get_pic_pos_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.coarse_stage_get_pic_pos_y, cur_row_index, 2, 1, 1)
        unit_label2 = QLabel("mm")
        unit_label2.setStyleSheet("color: #666;")
        layout.addWidget(unit_label2, cur_row_index, 3)
        cur_row_index += 1

        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        self.stacked_widget.addWidget(page)

    def create_bonding_arm_page(self):
        page = QWidget()
        layout = QGridLayout(page)
        layout.setVerticalSpacing(10)
        cur_row_index = 0

        # 键合臂起始位置
        start_label = QLabel("键合臂起始位置")
        start_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(start_label, cur_row_index, 0)
        self.bonding_arm_start_x = create_spinbox(-250, 250, 3)
        self.bonding_arm_start_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.bonding_arm_start_x, cur_row_index, 1, 1, 1)
        unit_label = QLabel("mm")
        unit_label.setStyleSheet("color: #666;")
        layout.addWidget(unit_label, cur_row_index, 3)
        cur_row_index += 1

        # 键合臂拍照位置
        pic_label = QLabel("键合臂拍照位置")
        pic_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(pic_label, cur_row_index, 0)
        self.bonding_arm_get_pic_pos_x = create_spinbox(-250, 250, 3)
        self.bonding_arm_get_pic_pos_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.bonding_arm_get_pic_pos_x, cur_row_index, 1, 1, 1)
        unit_label2 = QLabel("mm")
        unit_label2.setStyleSheet("color: #666;")
        layout.addWidget(unit_label2, cur_row_index, 3)
        cur_row_index += 1

        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        self.stacked_widget.addWidget(page)

    def create_flip_arm_page(self):
        page = QWidget()
        layout = QGridLayout(page)
        layout.setVerticalSpacing(10)
        cur_row_index = 0

        # 翻转模块Ry起始位置
        start_label = QLabel("翻转模块Ry起始位置")
        start_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(start_label, cur_row_index, 0)
        self.filp_arm_start_x = create_spinbox(-250, 250, 3)
        self.filp_arm_start_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.filp_arm_start_x, cur_row_index, 1, 1, 1)
        unit_label = QLabel("°")
        unit_label.setStyleSheet("color: #666;")
        layout.addWidget(unit_label, cur_row_index, 3)
        cur_row_index += 1

        # 翻转模块Ry拍照位置
        pic_label = QLabel("翻转模块Ry拍照位置")
        pic_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(pic_label, cur_row_index, 0)
        self.filp_arm_get_pic_pos_x = create_spinbox(-250, 250, 3)
        self.filp_arm_get_pic_pos_x.setStyleSheet("""
            QSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
            }
        """)
        layout.addWidget(self.filp_arm_get_pic_pos_x, cur_row_index, 1, 1, 1)
        unit_label2 = QLabel("°")
        unit_label2.setStyleSheet("color: #666;")
        layout.addWidget(unit_label2, cur_row_index, 3)
        cur_row_index += 1

        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,  # 水平策略 - 固定
                                      QSizePolicy.Expanding)  # 垂直策略 - 可扩展
        layout.addItem(vertical_spacer, cur_row_index, 0)
        self.stacked_widget.addWidget(page)

    def add_camera_dict(self, camera_dict):
        for key, value in camera_dict.items():
            self.camera_ID.addItem(key)

    def open_chip_window(self):
        # 如果已经打开就不再重复创建
        if hasattr(self, 'wafer_viewer') and self.wafer_viewer is not None:
            self.wafer_viewer.show()
            self.wafer_viewer.raise_()
            self.wafer_viewer.activateWindow()
            return

        self.wafer_viewer = ChipPositionShow.WaferViewer()
        self.wafer_viewer.setAttribute(Qt.WA_DeleteOnClose, True)

        # 连接信号（如果WaferViewer类定义了这些信号）
        if hasattr(self.wafer_viewer, 'sig_selected_points'):
            self.wafer_viewer.sig_selected_points.connect(self.set_new_chip_mark_position)
        if hasattr(self.wafer_viewer, 'sig_move_to_mark_position'):
            self.wafer_viewer.sig_move_to_mark_position.connect(self.emit_move_coarse_stage_to_position)

        self.wafer_viewer.destroyed.connect(lambda: setattr(self, 'wafer_viewer', None))
        self.wafer_viewer.show()

    def set_new_chip_mark_position(self, points):
        """处理芯片标记位置选择"""
        pass

    def emit_move_coarse_stage_to_position(self, position):
        """发送移动到标记位置的信号"""
        pass

    def close_chip_window(self):
        if hasattr(self, 'wafer_viewer') and self.wafer_viewer is not None and self.wafer_viewer.isVisible():
            self.wafer_viewer.close()
            self.wafer_viewer = None


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
