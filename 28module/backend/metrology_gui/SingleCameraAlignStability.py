# -*- coding: utf-8 -*-
"""
Filename: SingleCameraAlignStability.py
Description:
    相机对准重复性·静态（对照参考框架 SingleCameraAlignStability.py）。
    流程：循环{等待 wait_ms → take_photo → 记 row/col} → CSV → 双曲线
    （row/col 子图，标题 3σ nm，np.std ddof=0 × 3 × nm_per_px）。
    模板名必填（须在视觉页已注册），默认 cv_cross_0824。

    与参考框架的刻意偏差：
    - take_photo 失败抛 TakePhotoError 时经 sig_error 中止作业，
      绝不把 0 值 append 进统计（参考语义超时返回 [0,0] 会被照单
      全收，正是 3σ 爆表的根因）；
    - 迭代 42：删 warm-up 首拍（迭代 27 曾"正式循环前先打 1 拍结果
      丢弃"）——后端 _warmup 启动已预热，GUI 侧 warm-up 多余且是
      拍照链卡等重灾区；首拍即测量拍。
"""
import os
import threading
import time

import numpy as np
import pandas as pd
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from metrology_config import MetrologyConfig
from script_widget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from mqtt_receive_worker import MqttReceiveWorker
import device_actuator
import SingleCameraAlignStability_ui as ui


class SingleCameraAlignStability(ScriptWidget):
    TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.config = MetrologyConfig()
        # 启动时刷新 nm_per_px（后端不在线则保留初值）
        refreshed = self.config.refresh_nm_per_px()
        self.ui.add_camera_dict(self.config.camera_dict)

        # MQTT（后端不在线时允许界面照常打开，运行时才报错）
        self.paho_client = PahoMqttClient(log_cb=self.log)
        try:
            self.paho_client.connect(str(self.config.MQTTHOST),
                                     int(self.config.MQTTPORT))
            self.paho_client.subscribe(self.TOPICS)
        except Exception as e:
            self.ui.textEdit.append("[warn] MQTT broker 未连接（%s），"
                                    "开始测校前请先启动后端仿真" % e)
        # log_cb：陈旧帧/camID 不符丢弃落 Operation Log（迭代 42）
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client,
                                                     self.event,
                                                     log_cb=self.log)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        # 运行控制
        self.ui.btn_run.clicked.connect(self._on_run_clicked)
        self.ui.btn_interrupt.clicked.connect(self.stop_)
        self.sig_progress.connect(self._on_progress)
        self.ui.textEdit.append(
            "nm_per_px: %s（%s）" % (
                {k: v["nm_per_px"] for k, v in self.config.camera_info.items()},
                "已从 machine_constants 刷新" if refreshed else "初值，后端未在线"))

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        INPUT = {
            'camera_ID': self.ui.camera_ID.currentText(),
            'vision_template': self.ui.cb_vision_template.text(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': int(self.ui.wait_time.value()),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentText(param_dict['camera_ID'])
        self.ui.cb_vision_template.setText(param_dict['vision_template'])
        self.ui.repeat_times.setValue(param_dict['repeat_time'])
        self.ui.wait_time.setValue(param_dict['wait_time'])

    # ---------------- 运行（worker 线程；UI 更新只走信号） ----------------
    def run(self, dict_input, dict_mc):
        if not self.paho_client.get_connect_statue():
            raise RuntimeError("MQTT broker 未连接，请先启动后端仿真")
        cam_id = self.config.camera_dict[dict_input['camera_ID']]
        vision_template = dict_input['vision_template'].strip()
        if not vision_template:
            raise ValueError("视觉模板名称必填（模板须在视觉页已注册）")
        repeat_time = int(dict_input['repeat_time'])
        wait_time = float(dict_input['wait_time'])
        k_nm = self.config.nm_per_px(cam_id)

        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()
        row_list, col_list = [], []
        self.log("test start: camera=%s template=%s repeat=%d wait_ms=%s"
                 % (cam_id, vision_template, repeat_time, wait_time))
        try:
            for i in range(repeat_time):
                if not self.running:
                    break
                self.event.wait(wait_time / 1000.0)
                # 拍照（失败抛 TakePhotoError → sig_error 中止，
                # 绝不把 0 值 append 进统计）
                row, col = device_actuator.take_photo(
                    cam_id, vision_template, self.paho_client,
                    self.mqtt_receive_worker)
                row_list.append(row)
                col_list.append(col)
                self.log("detect result: row: %.2f, col: %.2f" % (row, col))
                self.log("cycle time: %d / %d" % (i + 1, repeat_time))
                self.set_progress(i + 1, repeat_time)
        finally:
            self.mqtt_receive_worker.stop()

        if not row_list:
            self.log("no data, test aborted")
            return {}

        # save data
        saved_columns_name = ['index', 'col', 'row']
        save_file_name = "single_camera_stability_%s.csv" \
                         % time.strftime('%m%d_%H_%M_%S')
        save_data = np.array([list(range(len(row_list))), col_list,
                              row_list]).T
        os.makedirs(self.config.DATA_SAVE_ROOT, exist_ok=True)
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(
            self.config.DATA_SAVE_ROOT + save_file_name, index=False)
        self.log("data saved: %s" % save_file_name)

        # display result（经信号回 GUI 线程绘图）
        self.set_realtime_data({'row': row_list, 'col': col_list,
                                'nm_per_px': k_nm})
        return {'csv': save_file_name, 'n': len(row_list)}

    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    # ---------------- GUI 线程槽 ----------------
    def plot_realtime_data(self, data):
        row_list, col_list = data['row'], data['col']
        k_nm = data['nm_per_px']
        self.res_ax_row.clear()
        self.res_ax_col.clear()
        self.res_ax_row.plot(list(range(len(row_list))), row_list)
        self.res_ax_row.set_title("row: %.3f nm (3σ)"
                                  % (np.std(row_list) * 3 * k_nm))
        self.res_ax_row.set_ylabel("pixel")
        self.res_ax_col.plot(list(range(len(col_list))), col_list)
        self.res_ax_col.set_title("col: %.3f nm (3σ)"
                                  % (np.std(col_list) * 3 * k_nm))
        self.res_ax_col.set_ylabel("pixel")
        self.res_fig.tight_layout()
        self.res_canvas.draw()

    def _on_run_clicked(self):
        self.ui.btn_run.setEnabled(False)
        self.ui.btn_interrupt.setEnabled(True)
        self.ui.progress_bar.setValue(0)
        self.begin_()

    def _on_progress(self, cur, total):
        self.ui.progress_bar.setMaximum(max(total, 1))
        self.ui.progress_bar.setValue(cur)

    def _on_finish(self, output):
        super()._on_finish(output)
        self.ui.btn_run.setEnabled(True)
        self.ui.btn_interrupt.setEnabled(False)

    def _on_error(self, err_msg):
        super()._on_error(err_msg)
        self.ui.btn_run.setEnabled(True)
        self.ui.btn_interrupt.setEnabled(False)


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = SingleCameraAlignStability()
    test.show()
    sys.exit(myapp.exec_())
