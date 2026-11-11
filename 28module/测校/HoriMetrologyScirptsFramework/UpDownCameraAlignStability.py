# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic.py
Date: 2026/03/11
Description:
    上下相机对准重复性
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import UpDownCameraAlignStability_ui as ui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
from PyQt5.QtGui import QTextCursor
import pandas as pd
import numpy as np
import threading
import random
import json
import sys

# 继承ScriptWidget类
class UpDownCameraAlignStability(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        TOPICS = ["VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.paho_client.subscribe(TOPICS)
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_row_2 = self.res_ax_row.twinx()

        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_ax_col_2 = self.res_ax_col.twinx()

        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.set_realtime_data([])

        self.ui.add_camera_dict(self.config.camera_dict)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'camera_ID': int(self.ui.camera_ID.currentIndex()),
            'vision_template': self.ui.led_vision_template.text(),
            'snap_wait_time': self.ui.snap_wait_time.value(),
            'camera_2_ID': int(self.ui.camera_2_ID.currentIndex()),
            'cam_2_vision_template': self.ui.led_cam_2_vision_template.text(),
            'cycle_wait_time': self.ui.cycle_wait_time.value(),
            'repeat_time': int(self.ui.repeat_times.value()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentIndex(param_dict.get('camera_ID', 0))
        self.ui.led_vision_template.setText(param_dict.get('vision_template', "Enter template name"))

        self.ui.snap_wait_time.setValue(param_dict.get('snap_wait_time', 0))
        self.ui.cycle_wait_time.setValue(param_dict.get('cycle_wait_time', 0))

        self.ui.camera_2_ID.setCurrentIndex(param_dict.get('camera_2_ID', 1))
        self.ui.led_cam_2_vision_template.setText(param_dict.get('cam_2_vision_template', "Enter template name"))

        self.ui.repeat_times.setValue(param_dict.get('repeat_time', 1))

    def plot_realtime_data(self, data):
        self.res_ax_row.clear()
        self.res_ax_col.clear()

        color1 = '#1f77b4'  # 蓝色
        self.res_ax_row.set_xlabel('index', fontsize=8)
        self.res_ax_row.set_ylabel('cam 1 row (pixel)', color=color1, fontsize=8)
        self.res_ax_row.tick_params(axis='y', labelcolor=color1)

        color2 = '#ff7f0e'  # 橙色
        self.res_ax_row_2.set_ylabel('cam 1 row (pixel)', color=color2, fontsize=8)
        self.res_ax_row_2.tick_params(axis='y', labelcolor=color2)

        color1 = '#1f77b4'  # 蓝色
        self.res_ax_col.set_xlabel('index', fontsize=8)
        self.res_ax_col.set_ylabel('cam 1 col (pixel)', color=color1, fontsize=8)
        self.res_ax_col.tick_params(axis='y', labelcolor=color1)

        color2 = "#f07408"  # 橙色
        self.res_ax_col_2.set_ylabel('cam 1 col (pixel)', color=color2, fontsize=8)
        self.res_ax_col_2.tick_params(axis='y', labelcolor=color2)


        if data is not None and len(data) > 0:
            line_row_1 = self.res_ax_row.plot(data[:, 0], data[:, 1],
                                              color=color1, linewidth=2.5, alpha=0.7, label='cam 1')
            line_row_2 = self.res_ax_row_2.plot(data[:, 0], data[:, 2],
                                                color=color2, linewidth=2.5, alpha=0.7, label='cam 2')
            line_col_1 = self.res_ax_col.plot(data[:, 0], data[:, 3],
                                              color=color1, linewidth=2.5, alpha=0.7, label='cam 1')
            line_col_2 = self.res_ax_col_2.plot(data[:, 0], data[:, 4],
                                                color=color2, linewidth=2.5, alpha=0.7,  label='cam 2')
            # 合并图例（关键技巧：避免图例重叠）
            lines_row = line_row_1 + line_row_2
            labels = [l.get_label() for l in lines_row]
            self.res_ax_row.legend(lines_row, labels, loc='upper left', frameon=True, fancybox=True, shadow=True)
            lines_col = line_col_1 + line_col_2
            labels = [l.get_label() for l in lines_col]
            self.res_ax_col.legend(lines_col, labels, loc='upper left', frameon=True, fancybox=True, shadow=True)

        self.res_fig.tight_layout()

        self.res_canvas.draw()

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()

        camera_ID = int(self.config.camera_dict[self.ui.camera_ID.currentText()])
        vision_template = self.ui.led_vision_template.text()
        snap_wait_time = self.ui.snap_wait_time.value()
        cycle_wait_time = self.ui.cycle_wait_time.value()
        camera_2_ID = int(self.config.camera_dict[self.ui.camera_2_ID.currentText()])
        cam_2_vision_template = self.ui.led_cam_2_vision_template.text()
        repeat_time = int(self.ui.repeat_times.value())

        vision_detect_res_list = []
        self.append_to_text_edit(f"{'-'}*10 start")
        for i in range(repeat_time):
            if not self.running:
                break
            # 相机1拍照
            row_1, col_1 = self.take_photo(camera_ID, vision_template)

            self.event.wait(snap_wait_time)

            # 相机2拍照
            row_2, col_2 = self.take_photo(camera_2_ID, cam_2_vision_template)

            vision_detect_res_list.append([i, row_1, col_1, row_2, col_2])

            self.event.wait(cycle_wait_time)
            self.append_to_text_edit(f"cycle time: {i + 1} / {repeat_time}")

        # save data
        saved_columns_name = ['index', 'cam 1 col', 'cam 1 row', 'cam 2 col', 'cam 2 row']
        save_data = np.array(vision_detect_res_list)
        save_file_name = f"up_and_down_camera_stability_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'cam_stability/' + save_file_name
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(full_save_path)

        self.set_realtime_data(save_data)

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    def take_photo(self, camera_id, template_name):
        # self.mqtt_receive_worker.reset_image_process_status()
        # self.paho_client.publish("VSC_Com",
        #                     mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0,
        #                                                           is_image_send=True),
        #                     0)
        # self.plc_client.camera_trigger(camera_id)
        # arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return 0, 0

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(message)
        self.ui.textEdit.moveCursor(QTextCursor.End)
        self.ui.textEdit.ensureCursorVisible()

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = UpDownCameraAlignStability()
    # test.MqttClient.loop_start()
    test.show()
    test.set_saved_param({}, {})
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
