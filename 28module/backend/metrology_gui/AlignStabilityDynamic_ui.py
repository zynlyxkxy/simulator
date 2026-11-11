# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic_ui.py
Description:
    相机对准重复性·动态 界面（布局/QSS 复刻参考框架
    AlignStabilityDynamic_ui.py；运动类型由参考的三选一 combo 改为
    随相机映射：up/coarse 只读标签（up→对准台 AS_X/Y、coarse→翻转模块
    FA_X 仅X，coarse 时 Y 输入禁用）；down 相机为运动组下拉——
    晶圆台 (WS_LS_X/Y) 默认 / 对准台 (AS_X/Y)，对应后端 jog_group
    wafer/align）。
    迭代 39：起始位置/拍照位置各配"取当前位"按钮（GET /api/axes
    按相机+运动组映射轴组回填，coarse 仅回填 X）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Camera Alignment Stability Test (Dynamic)")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(create_title_label("Camera Alignment Stability (Dynamic)"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(550)
        top_layout.addWidget(result_group)
        main_layout.addLayout(top_layout)

        # 日志区域
        log_group, self.textEdit = create_log_group(160)
        main_layout.addWidget(log_group)

        Widget.setLayout(main_layout)

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # 相机ID
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        self.camera_ID = create_combobox([])
        layout.addWidget(self.camera_ID, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称（必填：模板须在视觉页已注册）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit(
            "cv_cross_0824", "请输入已注册的视觉模板名称")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(0, 100000, 0)
        self.repeat_times.setValue(100)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 100000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 运动类型（随相机映射：up/coarse 只读标签；down 相机为运动组下拉——
        # 晶圆台 (WS_LS_X/Y) 默认 / 对准台 (AS_X/Y)，对应后端 jog_group）
        layout.addWidget(create_label("运动类型"), cur_row_index, 0)
        self.stage_move_type = create_lineedit("")
        self.stage_move_type.setReadOnly(True)
        layout.addWidget(self.stage_move_type, cur_row_index, 1, 1, 2)
        self.stage_move_combo = create_combobox(
            ["晶圆台 (WS_LS_X/Y)", "对准台 (AS_X/Y)"])
        self.stage_move_combo.setVisible(False)   # 仅 down 相机时显示
        layout.addWidget(self.stage_move_combo, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 起始位置 x/y
        start_label = QLabel("起始位置 (x, y)")
        start_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(start_label, cur_row_index, 0)
        self.start_x = create_spinbox(-250, 250, 3)
        self.start_y = create_spinbox(-250, 250, 3)
        layout.addWidget(self.start_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.start_y, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1
        # 起始位置取当前位（GET /api/axes 按相机+运动组映射轴组回填）
        self.btn_snap_start = self._make_snap_button()
        layout.addWidget(self.btn_snap_start, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 拍照位置 x/y
        pic_label = QLabel("拍照位置 (x, y)")
        pic_label.setStyleSheet("font-weight: 500; color: #333;")
        layout.addWidget(pic_label, cur_row_index, 0)
        self.photo_x = create_spinbox(-250, 250, 3)
        self.photo_y = create_spinbox(-250, 250, 3)
        layout.addWidget(self.photo_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.photo_y, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1
        # 拍照位置取当前位（同上映射）
        self.btn_snap_photo = self._make_snap_button()
        layout.addWidget(self.btn_snap_photo, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 运行控制（开始/中断 + 进度条）
        btn_layout = QHBoxLayout()
        self.btn_run = create_run_button("开始")
        self.btn_interrupt = create_stop_button("中断")
        self.btn_interrupt.setEnabled(False)
        btn_layout.addWidget(self.btn_run)
        btn_layout.addWidget(self.btn_interrupt)
        layout.addLayout(btn_layout, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                background-color: white;
                text-align: center;
                min-height: 16px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.progress_bar, cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 创建垂直弹簧
        vertical_spacer = QSpacerItem(10, 20,
                                      QSizePolicy.Minimum,
                                      QSizePolicy.Expanding)
        layout.addItem(vertical_spacer, cur_row_index, 0)

        group.setLayout(layout)
        return group

    def add_camera_dict(self, camera_dict):
        for key in camera_dict.keys():
            self.camera_ID.addItem(key)

    @staticmethod
    def _make_snap_button():
        btn = QPushButton("取当前位")
        btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #357abd;
                border: 1px solid #4a90e2;
                border-radius: 4px;
                padding: 5px 12px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #eaf3fd; }
        """)
        return btn


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
