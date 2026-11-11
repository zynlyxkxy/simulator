# -*- coding: utf-8 -*-
"""
Filename: BondTempCalib_ui.py
Description:
    键合温度校准 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备）。

    后端 routine bond_temp_calib（cam 恒 up）：按 setpoints 逐点控温，
    每点采 samples×sample_interval_ms 温度序列求 mean/3σ，线性拟合
    gain/offset——heater=up 写回 head_temp_gain/head_temp_offset_c，
    heater=wafer 写回 wafer_temp_gain/wafer_temp_offset_c。

    参数区：加热器下拉（up=键合头脉冲加热 / wafer=硅片台加热，默认
    up；切换时 setpoints 文本在两份默认间联动——up 默认
    "100,200,300"、wafer 默认 "50,80,110"，仅在文本为空或等于另一
    份默认时替换，不覆盖用户手改值）+ setpoints_c 文本 + samples
    （默认 20）+ sample_interval_ms（默认 100）。
    结果区：mean_c vs setpoint_c 散点 + 拟合直线（标注 gain/offset）
    + 统计表（每 setpoint 一行 mean/3σ + gain/offset_c，measured vs
    current）+ "写回温度校准"按钮（作业 done 后可用，POST
    /api/calibration/apply）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton, QLabel
)
from PyQt5 import QtWidgets

# 各加热器默认温度点（切换下拉时的联动缺省）
DEFAULT_SETPOINTS = {"up": "100,200,300", "wafer": "50,80,110"}


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1100, 800)
        Widget.setWindowTitle("Bond Temperature Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Bond Temperature Calibration"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(520)
        # 统计表（每 setpoint 一行 mean/3σ + gain/offset_c；预留 8 点）
        self.stats_table = create_stats_table(
            ["统计量", "measured", "current"], rows=10, height=320)
        result_group.layout().addWidget(self.stats_table)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回温度校准")
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
            "POST /api/calibration/apply：heater=up 写回 "
            "head_temp_gain/head_temp_offset_c，wafer 写回 "
            "wafer_temp_gain/wafer_temp_offset_c（落盘+热生效）")
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
        lbl_cam = create_label("上精对准 (up)")
        layout.addWidget(lbl_cam, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 加热器下拉（up=键合头脉冲加热 / wafer=硅片台加热，默认 up）
        layout.addWidget(create_label("加热器"), cur_row_index, 0)
        self.cb_heater = create_combobox(
            ["up (键合头脉冲加热)", "wafer (硅片台加热)"])
        self.cb_heater.currentIndexChanged.connect(
            self._on_heater_changed)
        layout.addWidget(self.cb_heater, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 温度点 setpoints_c（文本，逗号分隔；默认随加热器联动）
        layout.addWidget(create_label("温度点 setpoints"), cur_row_index, 0)
        self.setpoints = create_lineedit(
            DEFAULT_SETPOINTS["up"], "逗号分隔，如 100,200,300")
        layout.addWidget(self.setpoints, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("℃"), cur_row_index, 2)
        cur_row_index += 1

        # 每点采样数 samples（默认 20）
        layout.addWidget(create_label("每点采样数"), cur_row_index, 0)
        self.samples = create_spinbox(5, 500, 0)
        self.samples.setValue(20)
        layout.addWidget(self.samples, cur_row_index, 1, 1, 1)
        cur_row_index += 1

        # 采样间隔 sample_interval_ms（默认 100）
        layout.addWidget(create_label("采样间隔"), cur_row_index, 0)
        self.sample_interval = create_spinbox(10, 60000, 0)
        self.sample_interval.setValue(100)
        layout.addWidget(self.sample_interval, cur_row_index, 1, 1, 1)
        layout.addWidget(create_label("ms"), cur_row_index, 2)
        cur_row_index += 1

        # 页面说明（耗时提示）
        hint = QLabel(
            "测温升降温耗时较长（硅片台约0.73℃/s），可随时中断。\n"
            "流程：按 setpoints 逐点控温，每点采 samples×间隔温度序列求"
            " mean/3σ，线性拟合 gain/offset（实测 = gain×设定 + "
            "offset）。")
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

    def heater(self):
        """当前加热器契约值：up / wafer。"""
        return "wafer" if "wafer" in self.cb_heater.currentText() else "up"

    def _on_heater_changed(self, _i):
        """切换加热器时 setpoints 在两份默认间联动（文本为空或等于另一
        份默认才替换，不覆盖用户手改值）。"""
        cur = self.setpoints.text().strip()
        other = {k: v for k, v in DEFAULT_SETPOINTS.items()
                 if k != self.heater()}
        if not cur or cur in other.values():
            self.setpoints.setText(DEFAULT_SETPOINTS[self.heater()])


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    ui = Ui_Widget()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())
