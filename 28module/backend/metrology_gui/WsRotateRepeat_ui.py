# -*- coding: utf-8 -*-
"""
Filename: WsRotateRepeat_ui.py
Description:
    硅片台旋转重复性 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    迭代 32 行布局化（用户反馈旧 2×2 紧凑示教组在 330px 滚动区里被
    压扁）：改表格式行布局——点数 spinbox + 表头（# | 模板名 | 旋前
    AS_X AS_Y WS_X WS_Y | 旋后 AS_X AS_Y WS_X WS_Y）+ 每点一行（行内
    控件紧凑等宽，组标题"旋前"/"旋后"视觉分组，组区宽度自适应内容，
    QScrollArea 只管超长列表纵向滚动）；行编辑组件与
    WsRotateCenterCalib/WsRotateGainCalib 共用（taught_points_table.
    TaughtPointsTable，迭代 35 起每点 8 列 + →旋前/→旋后两个取位
    按钮）。底部参数区（rot_rad/repeat/wait）不动。

    迭代 35 页面说明 label：旋前走到位测 P_k，旋后走到位测 Q_k，
    后端按 find_rotation_center 口径拟合旋转角/中心；旋后位用于防止
    大角度旋转后标记出视场，不填=旋前位（小角度）。

    结果区（迭代 32 直接测角语义）：统计表末三行=角度误差 e 的
    mean/std/3σ（µrad，stats.angle_err）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_title_label, create_log_group, create_result_group,
    create_run_button, create_stop_button, create_stats_table,
    apply_global_style)
from taught_points_table import TaughtPointsTable

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QLabel
)
from PyQt5 import QtWidgets

# 页面说明（迭代 35）：每点两个位姿的用途与缺省
POSE_HINT = ("流程：旋前各点走到位测 P_k → 旋转 → 旋后各点走到位测 Q_k，"
             "后端按 find_rotation_center 口径拟合旋转角/中心。"
             "旋后位用于防止大角度旋转后标记出视场；"
             "不填=旋前位（小角度）。")


def _make_pose_hint():
    lab = QLabel(POSE_HINT)
    lab.setWordWrap(True)
    lab.setStyleSheet("color: #888; font-size: 11px;")
    lab.setAlignment(Qt.AlignLeft)
    return lab


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1400, 900)
        Widget.setWindowTitle("Wafer Stage Rotation Repeatability")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Wafer Stage Rotation Repeatability"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        # 迭代 35：结果组 min-width 550→470，给 8 列示教表让宽
        result_group, self.res_display = create_result_group(470)
        # 统计表（N 个示教点行 + 3 行角度误差 e 统计，行数随点数动态）
        self.stats_table = create_stats_table(
            ["示教点", "col 3σ (nm)", "row 3σ (nm)", "e (µrad)"],
            rows=3, height=116)
        result_group.layout().addWidget(self.stats_table)
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

        # 示教点表格（行布局组件：组标题表头 + 每点一行，超高才滚动）
        self.pts_table = TaughtPointsTable()
        layout.addWidget(self.pts_table, cur_row_index, 0, 1, 3)
        cur_row_index += 1
        self.rebuild_points(2)

        # 页面说明（迭代 35）：旋前/旋后双位姿流程与缺省
        layout.addWidget(_make_pose_hint(), cur_row_index, 0, 1, 3)
        cur_row_index += 1

        # 旋转指令角（rad；后端首选 rot_rad）
        layout.addWidget(create_label("旋转角 (rad)"), cur_row_index, 0)
        self.rot_rad = create_spinbox(0.0001, 0.6, 4)
        self.rot_rad.setValue(0.0175)
        layout.addWidget(self.rot_rad, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("rad（0.0175≈1°）"),
                         cur_row_index, 2)
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
