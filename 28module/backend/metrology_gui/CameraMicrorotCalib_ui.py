# -*- coding: utf-8 -*-
"""
Filename: CameraMicrorotCalib_ui.py
Description:
    相机微小旋转角标定 界面（布局/QSS 复刻参考框架三段式：蓝标题条 +
    Input Parameters / Result Display(matplotlib 嵌入+统计表) /
    Operation Log；开始/中断按钮与进度条为本软件无宿主自备；参数形态
    对照参考框架 CameraTiltAngleCalibration：相机 + 模板名 + 两个
    拍照位 + 重复次数）。

    迭代 31：后端 routine camera_microrot（cam=up/down/coarse）。
    迭代 34：模板名必填；拍照位1/2 四输入框常使能（删"指定拍照位"
    复选框，photo_pos1/2 总是下发）；新增运动台下拉（仅 down 相机
    可选：对准台 align 默认 / 晶圆台 wafer；上相机恒对准台禁用、
    粗对准相机走翻转模块不参与）；各配"取当前位"按钮（GET
    /api/axes 按相机+运动台映射运动组回填）。
    结果区下方带"写回 rot_mrad"按钮（作业 done 后可用，POST
    /api/calibration/apply fields=["rot_mrad"]）。
    迭代 38（任务 6，真值/认知分离）：结果区新增"视场旋转建议"块——
    当前 view_rot_deg 显示（cameras.<cam>.view_rot_deg=物理旋转）+
    "物理转正"按钮（view_rot 调为 cur−measured×0.0572958°，POST
    /api/vision/camera/config，限 ±5° 超限钳位；与①写回 rot_mrad
    二选一，都做=双重补偿）。
    迭代 42：删"当前视场旋转 view_rot_deg"常驻显示行（真机读不到
    view_rot_deg 物理真值，GET /api/vision/cameras 无此接口）——
    "物理转正"按钮保留，view_rot 改为点击按钮时才惰性读取，读取
    不可用仅落日志区（不抛异常不弹窗）。
    迭代 39：统计表下方新增"当前 rot_mrad（机器常数·控制器认知）"
    与"差值 = 实测−当前"只读行（结果 current.rot_mrad，缺省显示 —）；
    建议块文案明确二选一语义（写回=反算链生效图像不变／物理转正=
    直接旋转视场图像去旋，都做=双重补偿）。
"""
from qt_custom_ui import (
    create_combobox, create_spinbox, create_label, create_groupbox,
    create_lineedit, create_title_label, create_log_group,
    create_result_group, create_run_button, create_stop_button,
    create_stats_table, apply_global_style)

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QProgressBar, QSpacerItem, QSizePolicy, QPushButton,
    QLabel
)
from PyQt5 import QtWidgets


class Ui_Widget:
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.resize(1050, 800)
        Widget.setWindowTitle("Camera Micro-Rotation Calibration")
        apply_global_style(Widget)

        main_layout = QtWidgets.QVBoxLayout(Widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # 标题区域
        main_layout.addWidget(
            create_title_label("Camera Micro-Rotation Calibration"))

        # 顶部布局：输入和结果
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        top_layout.addWidget(self.create_input_group())
        result_group, self.res_display = create_result_group(550)
        # 统计表（mean/std/3σ：µrad 与 mrad 双单位）
        self.stats_table = create_stats_table(
            ["N", "mean (mrad)", "mean (µrad)", "std (µrad)",
             "3σ (µrad)"], rows=1, height=80)
        result_group.layout().addWidget(self.stats_table)
        # 当前 rot_mrad / 差值 只读行（迭代 39：current=机器常数·控制器
        # 认知（结果 current.rot_mrad），差值=实测−当前，mrad 3 位小数；
        # 缺省 None 显示 —）
        self.lbl_rot_current = QLabel(
            "当前 rot_mrad（机器常数·控制器认知）：—")
        self.lbl_rot_current.setStyleSheet("color: #2c3e50;")
        result_group.layout().addWidget(self.lbl_rot_current)
        self.lbl_rot_diff = QLabel("差值 = 实测−当前：—")
        self.lbl_rot_diff.setStyleSheet("color: #2c3e50;")
        result_group.layout().addWidget(self.lbl_rot_diff)
        # 写回行（作业 done 后可用）
        apply_row = QHBoxLayout()
        self.btn_apply = QPushButton("写回 rot_mrad")
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
        # 视场旋转建议块（迭代 38 任务 6：真值/认知分离——view_rot_deg=
        # 相机物理旋转角真值（逆时针正，图像内容反向转），rot_mrad=
        # 控制器认知（机器常数），两者换算相等=补偿到位；
        # 迭代 42 起不再常驻显示当前 view_rot_deg——真机无此接口，
        # 点击"物理转正"时才惰性读取，读取结果落日志区）
        derot_row = QHBoxLayout()
        self.btn_derot = QPushButton(
            "物理转正（view_rot 调为 cur−measured×0.0572958°）")
        self.btn_derot.setEnabled(False)
        self.btn_derot.setStyleSheet("""
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
        self.btn_derot.setToolTip(
            "建议②：POST /api/vision/camera/config，把本相机的视场旋转"
            "（物理旋转）调为 当前值−实测 rot_mrad×0.0572958°（限 ±5°，"
            "超限钳位）；图像去旋，机器常数 rot_mrad 须置 0")
        lbl_derot_hint = QLabel(
            "二选一：①写回 rot_mrad=带入计算修正世界坐标（反算链生效，"
            "图像不变）／②物理转正=直接旋转视场（图像去旋，rot_mrad "
            "置 0）——都做=双重补偿")
        lbl_derot_hint.setStyleSheet("color: #666;")
        lbl_derot_hint.setWordWrap(True)
        derot_row.addWidget(self.btn_derot)
        derot_row.addWidget(lbl_derot_hint, 1)
        result_group.layout().addLayout(derot_row)
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

        # 相机（cam 契约值：up/down/coarse）
        layout.addWidget(create_label("相机"), cur_row_index, 0)
        self.cb_camera = create_combobox(
            ["下精对准 (down)", "上精对准 (up)", "粗对准 (coarse)"])
        layout.addWidget(self.cb_camera, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 运动台（仅下相机可选：对准台 align / 晶圆台 wafer，缺省 align；
        # 上相机恒对准台禁用；粗对准相机走翻转模块 FA_X 不参与本选择）
        layout.addWidget(create_label("运动台"), cur_row_index, 0)
        self.cb_move_stage = create_combobox(
            ["对准台 (align)", "晶圆台 (wafer)"])
        layout.addWidget(self.cb_move_stage, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 视觉模板名称（必填，须已在视觉页注册）
        layout.addWidget(create_label("视觉模板名称"), cur_row_index, 0)
        self.cb_vision_template = create_lineedit(
            "", "必填：模板须在视觉页已注册")
        layout.addWidget(self.cb_vision_template, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 拍照位1/2 必填（常使能；纯 X 间距口径由后端校验）

        # 拍照位1 (x, y) + 取当前位
        layout.addWidget(create_label("拍照位置1 (x,y)"), cur_row_index, 0)
        self.get_pic_pos_x_1 = create_spinbox(-250, 250, 4)
        self.get_pic_pos_y_1 = create_spinbox(-250, 250, 4)
        layout.addWidget(self.get_pic_pos_x_1, cur_row_index, 1, 1, 1)
        layout.addWidget(self.get_pic_pos_y_1, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1
        self.btn_snap_pos1 = self._make_snap_button()
        layout.addWidget(self.btn_snap_pos1, cur_row_index, 1, 1, 2)
        cur_row_index += 1

        # 拍照位2 (x, y) + 取当前位
        layout.addWidget(create_label("拍照位置2 (x,y)"), cur_row_index, 0)
        self.get_pic_pos_x_2 = create_spinbox(-250, 250, 4)
        self.get_pic_pos_y_2 = create_spinbox(-250, 250, 4)
        layout.addWidget(self.get_pic_pos_x_2, cur_row_index, 1, 1, 1)
        layout.addWidget(self.get_pic_pos_y_2, cur_row_index, 2, 1, 1)
        layout.addWidget(create_label("mm"), cur_row_index, 3)
        cur_row_index += 1
        self.btn_snap_pos2 = self._make_snap_button()
        layout.addWidget(self.btn_snap_pos2, cur_row_index, 1, 1, 2)
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
