# -*- coding: utf-8 -*-
"""
Filename: BestFocusCalib_ui.py
Description:
    相机最佳焦平面标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备；参数形态
    对照参考框架 OptimalFocalCal：相机 + z_start/z_end/step + 搜索轮
    数 + 每点帧数 + 等待时间——z 三格为文本框，留空=后端缺省）。

    迭代 42 新测校项：后端 routine best_focus（cam=up/down）——区间
    收缩法：每轮 z_start→z_end 按 step 扫 z-score 曲线、argmax 后区
    间缩到峰点左右各一采样点、step 减半，末轮峰处三点抛物线拟合得
    亚步距 z*；down 扫 AS_Z（写回 as_z_chuck_focus_mm）/ up 前置走
    位后扫 BS_Arm_Z（写回 head_z_upfocus_mm）。

    参数区：相机下拉（up/down）+ 视觉模板名称（留空=后端缺省按相
    机取 vision_marks）+ z_start/z_end/step（文本框，留空=后端缺省
    焦面常数 ±0.3mm/步 0.02）+ 搜索轮数（默认 2）+ 每点帧数（默认
    3）+ 等待时间。结果区：z-score 曲线（各轮不同颜色 + 峰值竖线）
    + best_z 大数字 + "写回焦面"按钮（作业 done 后可用，POST
    /api/calibration/apply）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
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
        Widget.setWindowTitle("Camera Best Focus Plane Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Camera Best Focus Plane Calibration"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # best_z 大数字显示（图上方；作业完成后由逻辑层填值）
        self.lbl_best_z = QLabel("最佳焦面：—（作业完成后显示）")
        self.lbl_best_z.setAlignment(Qt.AlignCenter)
        self.lbl_best_z.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #2c3e50; "
            "padding: 4px;")
        result_group.layout().insertWidget(0, self.lbl_best_z)
        # 统计表（best_z/拟合方式/扫描轴/置信度）
        self.stats_table = create_stats_table(
            ["统计量", "数值"], rows=4, height=152)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回焦面")
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
            "POST /api/calibration/apply：按相机写回 "
            "as_z_chuck_focus_mm（down）/ head_z_upfocus_mm（up）"
            "（落盘+热生效）")
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

        # 相机（cam 契约值：up/down；down 扫 AS_Z / up 扫 BS_Arm_Z）
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        self.cb_camera = create_combobox(
            ["下精对准 (down)——扫 AS_Z", "上精对准 (up)——扫 BS_Arm_Z"])
        layout.addWidget(self.cb_camera, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称（留空=后端缺省按相机取 vision_marks）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit(
            "", "留空=后端缺省（up=tpl_head_chuck / down=cv_quad）")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # z_start / z_end（文本框，留空=后端缺省焦面常数 ±0.3mm）
        layout.addWidget(create_label("z_start (mm)"), cur_row_index, 0)
        self.z_start = create_lineedit("", "留空=焦面常数−0.3")
        layout.addWidget(self.z_start, cur_row_index, 1, 1, 2)
        cur_row_index += 1
        layout.addWidget(create_label("z_end (mm)"), cur_row_index, 0)
        self.z_end = create_lineedit("", "留空=焦面常数+0.3")
        layout.addWidget(self.z_end, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # step_mm（文本框，留空=0.02）
        layout.addWidget(create_label("步长 step (mm)"), cur_row_index, 0)
        self.step_mm = create_lineedit("", "留空=0.02")
        layout.addWidget(self.step_mm, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 搜索轮数（默认 2，[1,5]）
        layout.addWidget(create_label("搜索轮数"), cur_row_index, 0)
        self.rounds = create_spinbox(1, 5, 0)
        self.rounds.setValue(2)
        layout.addWidget(self.rounds, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 每点帧数（默认 3，[1,10]）
        layout.addWidget(create_label("每点帧数"), cur_row_index, 0)
        self.frames_per_point = create_spinbox(1, 10, 0)
        self.frames_per_point.setValue(3)
        layout.addWidget(self.frames_per_point, cur_row_index, 1, 1, 1)
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
            "区间收缩法（参考 OptimalFocalCal，清晰度=score 代理）：每轮 "
            "z_start→z_end 按 step 扫 z-score 曲线，argmax 后区间缩到峰"
            "点左右各一采样点、step 减半；末轮峰处三点抛物线拟合得亚步"
            "距 z*。down 扫 AS_Z 写回 as_z_chuck_focus_mm（当前位须视野"
            "有标记）；up 先前置走位（AS→锚点/焦面基准、BS_Arm_Z→上焦"
            "面附近）再扫 BS_Arm_Z 写回 head_z_upfocus_mm。")
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
