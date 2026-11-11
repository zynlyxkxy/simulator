# -*- coding: utf-8 -*-
"""
Filename: BondingHeadHorizontalOffset.py
Date: 2025/9/1
Description:
    键合头水平偏心标定
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
import BondingHeadHorizontalOffset_ui as ui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pandas as pd
import numpy as np
import threading
import random
import json
import sys


# 继承ScriptWidget类
class BondingHeadHorizontalOffset(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.thread_event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion']
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.thread_event)

        self.res_fig = Figure()
        self.bonding_head_offset_x = self.res_fig.add_subplot(211)
        self.bonding_head_offset_y = self.res_fig.add_subplot(212)
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_display_layout = QVBoxLayout()
        self.res_display_layout.addWidget(self.res_canvas)
        self.ui.socre_display.setLayout(self.res_display_layout)
        self.res_plot_update()

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            "bonding_start_position": self.ui.bonding_start_position.value(),
            "bonding_end_position": self.ui.bonding_end_position.value(),
            "repeat_times": int(self.ui.repeat_times.value()),
            "bonding_head_index": self.ui.cb_bonding_head_index.currentIndex(),
            "movement_wait_time": self.ui.movement_wait_time.value()
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.bonding_start_position.setValue(param_dict.get("bonding_start_position", 0))
        self.ui.bonding_end_position.setValue(param_dict.get("bonding_end_position", 5))
        self.ui.repeat_times.setValue(param_dict.get("repeat_times", 10))
        bonding_head_index = param_dict.get("bonding_head_index", 0)
        if bonding_head_index >= 0:
            self.ui.cb_bonding_head_index.setCurrentIndex(bonding_head_index)
        self.ui.movement_wait_time.setValue(param_dict.get("movement_wait_time", 0))

    def plot_realtime_data(self, data):
        pass

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.thread_event.clear()
        self.mqtt_receive_worker.start()
        self.paho_client.subscribe(self.TOPICS)

        bonding_start_position = self.ui.bonding_start_position.value()
        bonding_end_position = self.ui.bonding_end_position.value()
        repeat_times = int(self.ui.repeat_times.value())
        bonding_head_index = self.ui.cb_bonding_head_index.currentIndex()
        movement_wait_time = self.ui.movement_wait_time.value()
        if bonding_head_index == 0:
            bonding_head_name = "BondHead_YNXN"
        elif bonding_head_index == 1:
            bonding_head_name = 'BondHead_YNXP'
        elif bonding_head_index == 2:
            bonding_head_name = "BondHead_YPXN"
        else:
            bonding_head_name = 'BondHead_YPXP'

        timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')
        current_axis = bonding_head_name
        self.paho_client.publish("GUI_DR_ACS_Motion",
                                 mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                     pos=bonding_start_position,
                                     stage_name=LST_AXIS_NAME[current_axis]["name"],
                                     stage_index=LST_AXIS_NAME[current_axis]["index"],
                                     axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                 ),
                                 0)
        self.mqtt_receive_worker.wait_stage_stop()
        self.thread_event.wait(movement_wait_time)
        self.append_to_text_edit(
            f"test start: bonding head={bonding_head_name}, start={bonding_start_position} mm, "
            f"end={bonding_end_position} mm, repeat={repeat_times}")

        res = []
        for _ in range(repeat_times):
            if not self.running:
                return {}, {}

            current_axis = bonding_head_name
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=bonding_start_position,
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
            self.thread_event.wait(movement_wait_time)
            start_x_capacitor_value, start_y_capacitor_value = self.mqtt_receive_worker.get_bonding_head_capacitor_value(bonding_head_index)

            current_axis = bonding_head_name
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=bonding_end_position,
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
            self.thread_event.wait(movement_wait_time)
            end_x_capacitor_value, end_y_capacitor_value = self.mqtt_receive_worker.get_bonding_head_capacitor_value(bonding_head_index)
            res.append([start_x_capacitor_value, start_y_capacitor_value,
                        end_x_capacitor_value, end_y_capacitor_value,
                        end_x_capacitor_value- start_x_capacitor_value, end_y_capacitor_value - start_y_capacitor_value])
            self.append_to_text_edit(
                f"cycle {len(res)}: start=({start_x_capacitor_value:.3f}, {start_y_capacitor_value:.3f}), "
                f"end=({end_x_capacitor_value:.3f}, {end_y_capacitor_value:.3f}), "
                f"offset=({end_x_capacitor_value - start_x_capacitor_value:.3f}, "
                f"{end_y_capacitor_value - start_y_capacitor_value:.3f}) um")
            if len(res) % 20 == 0:
                self.res_plot_update(np.array(res))
                self.save_data_to_file(np.array(res), timestamp_str)

        res = np.array(res)
        print(res)
        self.res_plot_update(res)
        self.save_data_to_file(res, timestamp_str)
        self.append_to_text_edit(
            f"test finished, data saved: BondingHeadHorizontalOffset_{timestamp_str}.csv")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.thread_event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(message)


    def res_plot_update(self, res_data:np.ndarray=None):
        self.bonding_head_offset_x.clear()
        self.bonding_head_offset_y.clear()

        x_mean, y_mean = 0, 0
        x_3sigma, y_3sigma = 0, 0
        if res_data is not None and res_data.shape[1] >= 6:
            self.bonding_head_offset_x.plot(res_data[:, 4])
            self.bonding_head_offset_y.plot(res_data[:, 5])
            mean = np.mean(res_data, axis=1)
            x_mean, y_mean = mean[4], mean[5]
            std3 = np.std(res_data, axis=1) * 3
            x_3sigma, y_3sigma = std3[4], std3[5]

        self.bonding_head_offset_x.set_title(f"x offset mean: {x_mean:.3f} um, 3sigma: {x_3sigma:.3f} um")
        self.bonding_head_offset_x.set_ylabel("offset / um")
        self.bonding_head_offset_x.set_xlabel("index")
        self.bonding_head_offset_y.set_title(f"y offset mean: {y_mean:.3f} um, 3sigma: {y_3sigma:.3f} um")
        self.bonding_head_offset_y.set_ylabel("offset / um")
        self.bonding_head_offset_y.set_xlabel("index")
        self.res_fig.tight_layout()
        self.res_canvas.draw()

    def save_data_to_file(self, res_data:np.ndarray, timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')):
        columns_name_list = ["start x capacitor value", "start y capacitor value",
                             "end x capacitor value", "end y capacitor value", 
                             "x offset", "y offset"]
        save_file_name = f"BondingHeadHorizontalOffset_{timestamp_str}.csv"
        pd.DataFrame(res_data, columns=columns_name_list).to_csv(self.config.DATA_SAVE_ROOT + save_file_name)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = BondingHeadHorizontalOffset()
    # test.MqttClient.loop_start()
    test.show()
    # test.begin_()
    test.set_saved_param({}, {})
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
