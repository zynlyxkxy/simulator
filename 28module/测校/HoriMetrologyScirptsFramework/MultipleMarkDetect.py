# -*- coding: utf-8 -*-
"""
Filename: MultipleMarkDetect.py
Date: 2025/9/1
Description:
    多标记对准重复性
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import  MultipleMarkDetect_ui as ui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pandas as pd
import numpy as np
import threading
import random
import time
import json
import sys


# 继承ScriptWidget类
class MultipleMarkDetect(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        detect_point_info = self.ui.detect_point_info_table_widget.get_all_data()
        INPUT = {
            'detect_point_info': detect_point_info,
            'repeat_times': int(self.ui.repeat_times.value()),
            'cycle_wait_time': self.ui.cycle_wait_time.value(),
            'motion_wait_time': self.ui.motion_wait_time.value(),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.detect_point_info_table_widget.set_table_data(param_dict.get('detect_point_info', []))
        self.ui.repeat_times.setValue(param_dict.get('repeat_times', 1))
        self.ui.motion_wait_time.setValue(param_dict.get('motion_wait_time', 0.5))
        self.ui.cycle_wait_time.setValue(param_dict.get('cycle_wait_time', 1))

    def plot_realtime_data(self, data):
        pass

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()
        self.paho_client.subscribe(self.TOPICS)

        repeat_times = int(self.ui.repeat_times.value())
        cycle_wait_time = self.ui.cycle_wait_time.value()
        motion_wait_time = self.ui.motion_wait_time.value()
        detect_point_info = self.ui.detect_point_info_table_widget.get_all_data()
        print(detect_point_info)

        for cycle_index in range(repeat_times):
            if not self.running:
                return {}, {}
            for cur_detect_point_info in detect_point_info:
                if not self.running:
                    return {}, {}

            self.event.wait(cycle_wait_time)

        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)
        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)

    def take_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                                 mcCamera.VisionCmdJson(id=camera_id, template=template_name, exposure=0),
                                 0)
        arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return arr_row, arr_col

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    def draw_res(self, row_res_list, col_res_list):
        self.res_fig.clear()
        self.res_ax_row.plot()
        self.res_fig.tight_layout()


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = MultipleMarkDetect()
    # test.MqttClient.loop_start()
    test.set_saved_param({}, {})
    test.get_saved_param()
    test.show()
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
