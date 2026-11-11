# -*- coding: utf-8 -*-
"""
Filename: interferometerStability.py
Date: 2025/9/15
Description:
    干涉仪重复性
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from MqttReceiveWorker import MqttReceiveWorker
import interferometerStability_ui as ui

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
class interferometerStability(ScriptWidget):
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

        self.draw_test_data([])

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            "test_duration": self.ui.test_duration.value(),
            "test_interval": self.ui.test_interval.value(),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.test_duration.setValue(param_dict.get("test_duration", 100))
        self.ui.test_interval.setValue(param_dict.get("test_interval", 1))

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
        test_duration = self.ui.test_duration.value()
        test_interval = self.ui.test_interval.value()

        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()
        b_time = time.time()
        interferometer_data_list = []
        self.append_to_text_edit(f"test start: duration={test_duration} s, interval={test_interval} s")
        while (time.time() - b_time < test_duration) and self.running:
            self.event.wait(test_interval)
            interferometer_data = self.mqtt_receive_worker.get_interferometer_data()
            interferometer_data_list.append(
                [len(interferometer_data_list) + 1, time.time()] + list(interferometer_data))

            if len(interferometer_data_list) % 20 == 0:
                self.draw_test_data(np.array(interferometer_data_list))
                self.save_data_to_file(interferometer_data_list)
                self.append_to_text_edit(f"collected {len(interferometer_data_list)} samples")

        save_file_name = self.save_data_to_file(interferometer_data_list)
        self.draw_test_data(np.array(interferometer_data_list))
        self.append_to_text_edit(
            f"test finished, {len(interferometer_data_list)} samples, data saved: {save_file_name}")
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
        self.ui.textEdit.append(message)

    def draw_test_data(self, interferometer_data):
        if interferometer_data is not None and len(interferometer_data) > 4:
            # 数据列：0 序号、1 时间戳、2 x、3 y、4 rx、5 ry、6 rz、7~12 通道原始值
            self.ax_interferometer_data_x.clear()
            self.ax_interferometer_data_y.clear()
            self.ax_interferometer_data_x.plot(interferometer_data[:, 2])
            self.ax_interferometer_data_y.plot(interferometer_data[:, 3])

            self.ax_interferometer_data_rx.clear()
            self.ax_interferometer_data_ry.clear()
            self.ax_interferometer_data_rx.plot(interferometer_data[:, 4])
            self.ax_interferometer_data_ry.plot(interferometer_data[:, 5])

        self.ax_interferometer_data_x.set_title(f"x")
        self.ax_interferometer_data_x.set_xlabel("index")
        self.ax_interferometer_data_x.set_ylabel("position / mm")
        self.ax_interferometer_data_y.set_title(f"y")
        self.ax_interferometer_data_y.set_xlabel("index")
        self.ax_interferometer_data_y.set_ylabel("position / mm")
        self.res_fig.tight_layout()

        self.ax_interferometer_data_rx.set_title(f"rx")
        self.ax_interferometer_data_rx.set_xlabel("index")
        self.ax_interferometer_data_rx.set_ylabel("rotation / urad")
        self.ax_interferometer_data_ry.set_title(f"ry")
        self.ax_interferometer_data_ry.set_xlabel("index")
        self.ax_interferometer_data_ry.set_ylabel("rotation / urad")
        self.res_fig_2.tight_layout()
        self.res_canvas.draw()
        self.res_canvas_2.draw()

    def save_data_to_file(self, data_to_save):
        if len(data_to_save) == 0:
            return None
        save_file_dir = self.config.DATA_SAVE_ROOT
        save_file_name = f"interferometerStability_{time.strftime('%m%d-%H%M%S')}.csv"

        save_data_column_name_list = [
            "index", "time stamp",
            "IFM x/mm", "IFM y/mm", "IFM rx/ urad", "IFM ry/ urad", "IFM rz/ urad",
            "IFM hx1", "IFM hx2", "IFM hx3", "IFM hy1", "IFM hy2", "IFM hy3"
        ]
        data = np.vstack(data_to_save)
        data = np.round(data, 8)
        pd.DataFrame(data, columns=save_data_column_name_list).to_csv(save_file_dir + save_file_name)
        return save_file_name

    def save_screenshot_grab(self):
        """使用 grab() 方法截图"""
        pixmap = self.grab()  # 截取整个窗口
        pixmap.save("screenshot.png", "PNG")

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = interferometerStability()
    test.show()
    test.save_screenshot_grab()
    sys.exit(myapp.exec_())
