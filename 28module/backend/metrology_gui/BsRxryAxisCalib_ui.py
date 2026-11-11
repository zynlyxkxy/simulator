# -*- coding: utf-8 -*-
"""
Filename: BsRxryAxisCalib_ui.py
Description:
    键合头Rx/Ry旋转轴标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    后端 routine bs_rxry_axis（cam 恒 up）：BS_Rx/BS_Ry 各按
    ±angle_mrad 摆动（neg/zero/pos 三位），上相机追踪 4 个标记的
    位移矢量，拟合旋转轴线（轴点 rx/ry_axis_point_mm + 方向
    rx/ry_axis_dir，3 元向量）+ 拟合残差 rx/ry_rmse_mm。

    参数区：摆动角度 angle_mrad（默认 1.5）+ 重复次数 repeat（默认 3）
    + 等待时间 wait_ms（默认 10）。
    结果区：Rx/Ry 双 subplot（4 标记 neg→pos 位移矢量 quiver，XY 投影）
    + 统计表（轴点/轴方向/RMSE，measured vs current）+ "写回旋转轴"
    按钮（作业 done 后可用，POST /api/calibration/apply fields=
    [rx_axis_point_mm, rx_axis_dir, ry_axis_point_mm, ry_axis_dir]）。
"""
from qt_custom_ui import (
    create_spinbox, create_label, create_groupbox,
    create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 800)
        Widget.setWindowTitle("Bond Head Rx/Ry Rotation Axis Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Bond Head Rx/Ry Rotation Axis Calibration"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # 统计表（轴点/轴方向/RMSE：measured vs current）
        self.stats_table = create_stats_table(
            ["统计量", "measured", "current"], rows=6, height=225)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回旋转轴")
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
            "POST /api/calibration/apply：fields=[rx_axis_point_mm, "
            "rx_axis_dir, ry_axis_point_mm, ry_axis_dir]（落盘+热生效）")
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
        lbl_cam = create_label("上精对准 (up)——追踪键合头 4 标记")
        layout.addWidget(lbl_cam, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 摆动角度 angle_mrad（默认 1.5；范围对齐后端校验 0.2~1.7）
        layout.addWidget(create_label("摆动角度"), cur_row_index, 0)
        self.angle_mrad = create_spinbox(0.2, 1.7, 2)
        self.angle_mrad.setValue(1.5)
        layout.addWidget(self.angle_mrad, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("mrad"), cur_row_index, 2)
        cur_row_index += 1

        # 重复次数 repeat（默认 3）
        layout.addWidget(create_label("重复次数"), cur_row_index, 0)
        self.repeat_times = create_spinbox(1, 20, 0)
        self.repeat_times.setValue(3)
        layout.addWidget(self.repeat_times, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 等待时间 wait_ms（默认 10）
        layout.addWidget(create_label("等待时间"), cur_row_index, 0)
        self.wait_time = create_spinbox(0, 60000, 0)
        self.wait_time.setValue(10)
        layout.addWidget(self.wait_time, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 页面说明
        hint = QLabel(
            "流程：BS_Rx/BS_Ry 各按 ±angle_mrad 摆动（neg/zero/pos 三位"
            "置），上相机追踪 4 个标记的位移矢量，三点圆弧拟合旋转轴线"
            "——输出轴点 rx/ry_axis_point_mm（3 元 mm）+ 轴方向 "
            "rx/ry_axis_dir（单位向量）+ 拟合残差 rx/ry_rmse_mm。结果图"
            "为 4 标记 neg→pos 位移矢量的 XY 投影（mm）。")
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
