# -*- coding: utf-8 -*-
"""
Filename: WsRotateCenterCalib_ui.py
Description:
    硅片台旋转中心标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    迭代 32 新测校项：后端 routine ws_rotate_center（cam=down）——示教
    点位上 WS_Rz 按 step_rad×n_angles 步进拍照，圆拟合实际旋转中心
    → measured.rz_center_offset_mm=[cx,cy]（mm）+ 残差 RMS/3σ +
    中心 repeat 3σ(µm)；写回 fields=["rz_center_offset_mm"]。
    迭代 38：结果区新增"标定机位 ref_xy"只读行（measured
    .rz_center_ref_xy_mm=[标定时 WS_LS_X, WS_LS_Y]，缺省 —；消费侧
    pivot=吸盘心+offset+(当前台位−ref) 平移复用）。

    参数区：示教点表格（taught_points_table 共用行布局组件，2~10 点；
    迭代 35 起每点 8 列 旋前/旋后 双位姿 + →旋前/→旋后两个取位按钮，
    旋后位用于防止大角度旋转后标记出视场，不填=旋前位）
    + step_rad（默认 0.01）+ n_angles（默认 3）+ repeat（默认 10）+
    wait。结果区：中心坐标大数字显示（mm 与 µm）+ 逐 repeat 中心散点/
    趋势 + 残差统计表（含标定机位 ref_xy 只读行）+ "写回
    rz_center_offset_mm"按钮（作业 done 后可用，POST
    /api/calibration/apply）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_title_label, create_log_group, create_result_group,
    create_run_button, create_stop_button, create_stats_table,
    apply_global_style)
from taught_points_table import TaughtPointsTable
from WsRotateRepeat_ui import _make_pose_hint

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1400, 900)
        Widget.setWindowTitle("Wafer Stage Rotation Center Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Wafer Stage Rotation Center Calibration"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        # 迭代 35：结果组 min-width 550→470，给 8 列示教表让宽
        result_group, self.res_display = create_result_group(470)
        # 中心坐标大数字显示（图上方；作业完成后由逻辑层填值）
        self.lbl_center = QLabel("旋转中心：—（作业完成后显示）")
        self.lbl_center.setAlignment(Qt.AlignCenter)
        self.lbl_center.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #2c3e50; "
            "padding: 4px;")
        result_group.layout().insertWidget(0, self.lbl_center)
        # 残差统计表（cx/cy + 残差 RMS/3σ + 中心 repeat 3σ + 标定机位
        # ref_xy 只读行，迭代 38 可见化 measured.rz_center_ref_xy_mm）
        self.stats_table = create_stats_table(
            ["统计量", "数值"], rows=6, height=228)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回 rz_center_offset_mm")
        self.btn_apply.setEnabled(False)
        self.btn_apply.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 18px;
                font-weight: bold;
            }
            QPushButton:hover { background: #357abd; }
            QPushButton:disabled { background: #b0c4de; }
        """)
        self.lbl_apply = QLabel("（作业完成后可写回机器常数）")
        self.lbl_apply.setStyleSheet("color: #666;")
        apply_row.addWidget(self.btn_apply)
        apply_row.addWidget(self.lbl_apply, 1)
        result_group.layout().addLayout(apply_row)
        top_layout.addWidget(result_group)
        main_layout.addLayout(top_layout)

        # 日志区域
        log_group, self.textEdit = create_log_group(160)
        main_layout.addWidget(log_group)

        Widget.setLayout(main_layout)

    @property
    def point_groups(self):
        """示教点行（迭代 35：tpl/spins 8 键 + btn_snap_before/after）。"""
        return self.pts_table.point_groups

    def rebuild_points(self, n):
        """示教点行数调整为 n（已填值按下标保留）。"""
        self.pts_table.rebuild(n)

    def create_input_group(self):
        group = create_groupbox("Input Parameters", min_width=500)
        # 迭代 35：Expanding 策略——窗口加宽时输入组（8 列示教表）优先吃
        # 富余宽度，避免横向滚动条；窗口窄时回退为示教表内 h-scroll
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout = QGridLayout()
        layout.setVerticalSpacing(10)
        layout.setHorizontalSpacing(8)

        cur_row_index = 0

        # 示教点数（2~10，整数，默认 2）
        layout.addWidget(create_label("示教点数"), cur_row_index, 0)
        self.n_points = create_spinbox(2, 10, 0)
        self.n_points.setValue(2)
        layout.addWidget(self.n_points, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 示教点表格（与旋转重复性共用行布局组件）
        self.pts_table = TaughtPointsTable()
        layout.addWidget(self.pts_table, cur_row_index, 0, 1, 3)
        cur_row_index += 1
        self.rebuild_points(2)

        # 页面说明（迭代 35）：旋前/旋后双位姿流程与缺省
        layout.addWidget(_make_pose_hint(), cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 步进角 step_rad（rad，默认 0.01）
        layout.addWidget(create_label("步进角 (rad)"), cur_row_index, 0)
        self.step_rad = create_spinbox(0.0001, 0.6, 4)
        self.step_rad.setValue(0.01)
        layout.addWidget(self.step_rad, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("rad"), cur_row_index, 2)
        cur_row_index += 1

        # 角度数 n_angles（默认 3）
        layout.addWidget(create_label("角度数"), cur_row_index, 0)
        self.n_angles = create_spinbox(2, 10, 0)
        self.n_angles.setValue(3)
        layout.addWidget(self.n_angles, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 重复次数（默认 10）
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


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
