# -*- coding: utf-8 -*-
"""
Filename: AsHeadCenterCalib_ui.py
Description:
    上吸盘位置标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    迭代 42 新测校项：后端 routine as_head_center（cam 恒 up）——
    前置走位（AS_X/Y→锚点、AS_Z→下焦面基准、BS_Arm_Z→上焦面基准）
    后每轮 XY ±1mm 动态再逼近回锚点 identify，solve_pixel 转世界偏移
    δ=标记−相机 → 新锚点=当前 AS 位+δ → measured.as_anchor_xy_mm /
    as_head_center_xy_mm（双键同值联动，上/下相机同轴）；写回
    fields=["as_anchor_xy_mm","as_head_center_xy_mm"]。

    参数区：视觉模板名称（缺省=机器常数 vision_marks.tpl_head_chuck，
    留空=后端缺省）+ 重复次数（默认 10）+ 等待时间。
    结果区：锚点坐标大数字显示（mm 与 µm）+ 逐 repeat 锚点散点（µm，
    相对均值）+ 统计表（mean/3σ）+ "写回锚点"按钮（作业 done 后可
    用，POST /api/calibration/apply）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 800)
        Widget.setWindowTitle("Head Chuck Center Calibration (Up Camera)")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Head Chuck Center Calibration (Up Camera)"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # 锚点坐标大数字显示（图上方；作业完成后由逻辑层填值）
        self.lbl_anchor = QLabel("锚点：—（作业完成后显示）")
        self.lbl_anchor.setAlignment(Qt.AlignCenter)
        self.lbl_anchor.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #2c3e50; "
            "padding: 4px;")
        result_group.layout().insertWidget(0, self.lbl_anchor)
        # 统计表（mean/3σ：mm 与 nm 双单位）
        self.stats_table = create_stats_table(
            ["统计量", "数值"], rows=5, height=190)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回锚点")
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
        self.btn_apply.setToolTip(
            "POST /api/calibration/apply：fields=[as_anchor_xy_mm, "
            "as_head_center_xy_mm] 双键同值联动写回（落盘+热生效）")
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

    def create_input_group(self):
        group = create_groupbox("Input Parameters")
        layout = QGridLayout()
        layout.setVerticalSpacing(12)
        layout.setHorizontalSpacing(10)

        cur_row_index = 0

        # 相机（恒 up，只读提示）
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        lbl_cam = create_label("上精对准 (up)——认键合头吸盘中心")
        layout.addWidget(lbl_cam, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称（留空=后端缺省 vision_marks.tpl_head_chuck）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit(
            "", "留空=后端缺省 tpl_head_chuck（须已注册）")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 重复次数（默认 10）
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(2, 100, 0)
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

        # 页面说明
        hint = QLabel(
            "流程：前置走位（AS_X/Y→锚点现值、AS_Z→下焦面基准、"
            "BS_Arm_Z→上焦面基准）→ 每轮 XY ±1mm 动态再逼近回锚点后 "
            "identify，solve_pixel 转世界偏移 δ=标记−相机 → 新锚点="
            "当前 AS 位+δ（相机随台动，写回后复测 δ≈0）。measured "
            "双键同值联动：as_anchor_xy_mm（wafer_ega 组）/"
            "as_head_center_xy_mm（focus_level 组）。")
        hint.setStyleSheet("color: #888; font-size: 9pt;")
        hint.setWordWrap(True)
        layout.addWidget(hint, cur_row_index, 0, 1, 3)
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
