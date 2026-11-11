# -*- coding: utf-8 -*-
"""
Filename: StageMoveRepeat_ui.py
Description:
    运动台移动重复性 界面（布局/QSS 复刻参考框架 AlignStabilityDynamic
    三段式：蓝标题条 + Input Parameters / Result Display(matplotlib
    嵌入+轴到位统计表) / Operation Log；开始/中断按钮与进度条为本
    软件无宿主自备）。

    迭代 34：相机下拉（对准台可选 上相机 up/下相机 down，缺省 up；
    晶圆台恒锁下相机）+ 视觉模板名称必填。
    迭代 39：起始位置/拍照位置各配"取当前位"按钮（GET /api/axes
    按运动台下拉取 WS_LS_X/Y 或 AS_X/Y 回填）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Stage Move Repeatability")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(create_title_label("Stage Move Repeatability"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(550)
        # 轴到位统计表（每轴 mean / 3σ）
        self.stats_table = create_stats_table(
            ["轴", "到位 mean (mm)", "到位 3σ (µm)"], rows=2, height=100)
        result_group.layout().addWidget(self.stats_table)
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

        # 运动台（晶圆台→ws_move_repeat / 对准台→as_move_repeat）
        layout.addWidget(create_label("运动台"), cur_row_index, 0)
        self.cb_stage = create_combobox(
            ["晶圆台 (WS_LS_X/Y)", "对准台 (AS_X/Y)"])
        layout.addWidget(self.cb_stage, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 相机（晶圆台恒下相机锁定；对准台可选 上相机/下相机，缺省 up）
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        self.cb_camera = create_combobox(["上相机 (up)", "下相机 (down)"])
        layout.addWidget(self.cb_camera, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称（必填，须已在视觉页注册）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit(
            "", "必填：模板须在视觉页已注册")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(3, 1000, 0)
        self.repeat_times.setValue(10)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 60000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 起始位置 (x, y)
        layout.addWidget(create_label("起始位置 (x, y)"), cur_row_index, 0)
        self.start_x = create_spinbox(-250, 250, 3)
        self.start_y = create_spinbox(-250, 250, 3)
        self.start_x.setValue(-5.0)
        self.start_y.setValue(-5.0)
        layout.addWidget(self.start_x, cur_row_index, 1, 1, 1)
        layout.addWidget(self.start_y, cur_row_index, 2, 1, 1)
        cur_row_index += 1
        # 起始位置取当前位（GET /api/axes 按运动台映射轴组回填）
        self.btn_snap_start = self._make_snap_button()
        layout.addWidget(self.btn_snap_start, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 拍照位置 (x, y)
        layout.addWidget(create_label("拍照位置 (x, y)"), cur_row_index, 0)
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
