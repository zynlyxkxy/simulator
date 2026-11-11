# -*- coding: utf-8 -*-
"""
Filename: interferometerDynamicStabilityHorizontal.py
Date: 2026/3/15
Description:
    干涉仪重复性，水平向
"""
import time
from cProfile import label

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from MqttReceiveWorker import MqttReceiveWorker
import device_actuator as actuator
import interferometerDynamicStabilityVertical_ui as ui

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
from PyQt5.QtCore import pyqtSignal
import pandas as pd
import numpy as np
import threading
import random
import json
import sys
import os


# 继承ScriptWidget类
class interferometerStability(TestScriptWidget):
    __sig_update_realtime_log = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion']
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.interferometer_data_number = 11

        self.res_fig = Figure()
        self.ax_interferometer_data_x = self.res_fig.add_subplot(211)
        self.ax_interferometer_data_y = self.res_fig.add_subplot(212)
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_widget_1.setLayout(self.res_fig_layout)

        self.res_fig_2 = Figure()
        self.ax_interferometer_data_rx = self.res_fig_2.add_subplot(211)
        self.ax_interferometer_data_ry = self.res_fig_2.add_subplot(212)
        self.res_canvas_2 = FigureCanvas(self.res_fig_2)
        self.res_fig_layout_2 = QVBoxLayout()
        self.res_fig_layout_2.addWidget(self.res_canvas_2)
        self.ui.res_widget_2.setLayout(self.res_fig_layout_2)

        self.draw_test_data(np.array([[]]))

        self.__sig_update_realtime_log.connect(self.append_to_text_edit)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        detect_point_info = self.ui.detect_point_info_table_widget.get_all_data()
        INPUT = {
            'detect_point_info': detect_point_info,
            'repeat_times': int(self.ui.repeat_times.value()),
            "test_interval": self.ui.test_interval.value(),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.detect_point_info_table_widget.set_table_data(param_dict.get('detect_point_info', [[0], [1]]))
        self.ui.repeat_times.setValue(param_dict.get('repeat_times', 1))
        self.ui.test_interval.setValue(param_dict.get("test_interval", 0.5))

    def plot_realtime_data(self, data):
        self.draw_test_data(data)

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.event.clear()

        detect_point_info = self.ui.detect_point_info_table_widget.get_all_data()
        repeat_times = int(self.ui.repeat_times.value())
        test_interval = self.ui.test_interval.value()

        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        interferometer_data_list = []
        current_move_number = 0
        save_file_name = f"interferometerHorizontalStability_{time.strftime('%m%d-%H%M%S')}.csv"

        self.__sig_update_realtime_log.emit(f"{'-'*5}> test start")
        while current_move_number < repeat_times and self.running:
            cycle_data = []
            for current_point in detect_point_info:
                if not self.running:
                    return {}, {}

                actuator.move_vertical_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                                   target_rx=0, target_ry=0, target_z=current_point[0])
                self.event.wait(0.1)
                interferometer_data = self.mqtt_receive_worker.get_interferometer_data()
                cycle_data.extend(current_point)
                cycle_data.extend(interferometer_data)
                if (len(interferometer_data) + 1) % 10 == 0:
                    self.set_realtime_data(np.array(interferometer_data_list))
                    self.save_data_to_file(np.array(interferometer_data_list), save_file_name)
                self.event.wait(test_interval)
            interferometer_data_list.append(np.array(cycle_data))
            current_move_number += 1
            self.__sig_update_realtime_log.emit(f"{'-' * 5}> cycle {current_move_number} end")

        self.set_realtime_data(np.array(interferometer_data_list))
        self.save_data_to_file(np.array(interferometer_data_list), save_file_name)
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(time.strftime("%H:%M:%S ") + message)

    def draw_test_data(self, interferometer_data):
        if interferometer_data is None or interferometer_data.shape[1] < self.interferometer_data_number + 2:
            return

        self.ax_interferometer_data_x.clear()
        self.ax_interferometer_data_y.clear()
        single_point_data_number = self.interferometer_data_number + 2
        for i in range(interferometer_data.shape[1] // single_point_data_number):
            self.ax_interferometer_data_x.plot(interferometer_data[:, 2 + i * single_point_data_number],
                                               alpha=0.8,
                                               label=f"point {i}")
            self.ax_interferometer_data_y.plot(interferometer_data[:, 3 + i * single_point_data_number],
                                               alpha=0.8,
                                               label=f"point {i}")

        self.ax_interferometer_data_rx.clear()
        self.ax_interferometer_data_ry.clear()
        for i in range(interferometer_data.shape[1] // single_point_data_number):
            self.ax_interferometer_data_rx.plot(interferometer_data[:, 4 + i * single_point_data_number],
                                               alpha=0.8,
                                               label=f"point {i}")
            self.ax_interferometer_data_ry.plot(interferometer_data[:, 5 + i * single_point_data_number],
                                               alpha=0.8,
                                               label=f"point {i}")

        self.ax_interferometer_data_x.set_title(f"x")
        self.ax_interferometer_data_x.set_xlabel("index")
        self.ax_interferometer_data_x.set_ylabel("position / mm")
        self.ax_interferometer_data_x.legend()
        self.ax_interferometer_data_y.set_title(f"y")
        self.ax_interferometer_data_y.set_xlabel("index")
        self.ax_interferometer_data_y.set_ylabel("position / mm")
        self.ax_interferometer_data_y.legend()
        self.res_fig.tight_layout()

        self.ax_interferometer_data_rx.set_title(f"rx")
        self.ax_interferometer_data_rx.set_xlabel("index")
        self.ax_interferometer_data_rx.set_ylabel("rotation / urad")
        self.ax_interferometer_data_rx.legend()
        self.ax_interferometer_data_ry.set_title(f"ry")
        self.ax_interferometer_data_ry.set_xlabel("index")
        self.ax_interferometer_data_ry.set_ylabel("rotation / urad")
        self.ax_interferometer_data_ry.legend()
        self.res_fig_2.tight_layout()

        self.res_canvas.draw()
        self.res_canvas_2.draw()

    def save_data_to_file(self, data_to_save, save_file_name):
        save_file_dir = self.config.DATA_SAVE_ROOT + '/ifm'
        save_data_column_name_list = []
        for index in range(data_to_save.shape[1] // (self.interferometer_data_number + 2)):
            save_data_column_name_list.extend([
                f"point {index} nominal z",
                f"point {index} IFM x/nm", f"point {index} IFM y/nm",
                f"point {index} IFM rx/ urad", f"point {index} IFM ry/ urad", f"point {index} IFM rz/ urad",
                f"point {index} IFM hx1", f"point {index} IFM hx2", f"point {index} IFM hx3",
                f"point {index} IFM hy1", f"point {index} IFM hy2", f"point {index} IFM hy3"
            ])

        data = np.vstack(data_to_save)
        data = np.round(data, 8)
        pd.DataFrame(data, columns=save_data_column_name_list).to_csv(save_file_dir +'/' + save_file_name)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = interferometerStability()
    test.set_saved_param({}, {})
    test.show()
    sys.exit(myapp.exec_())
