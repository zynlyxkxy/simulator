# -*- coding: utf-8 -*-
"""
Filename: interferometerStability.py
Date: 2025/9/15
Description:
    光谱共焦重复性
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from MqttReceiveWorker import MqttReceiveWorker
import SpectralConfocalStability_ui as ui
import cmdjson.MC_FL_cmd as mcFls

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
class SpectralConfocalStability(TestScriptWidget):
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
        self.ax_fls_data = self.res_fig.add_subplot(111)
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        self.draw_test_data([])

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'sensor_ID': int(self.ui.sensor_ID.value()),
            "test_duration": self.ui.test_duration.value(),
            "test_interval": self.ui.test_interval.value(),
            "average_number": self.ui.average_number.value()
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.sensor_ID.setValue(param_dict["sensor_ID"])
        self.ui.test_duration.setValue(param_dict["test_duration"])
        self.ui.test_interval.setValue(param_dict["test_interval"])
        self.ui.average_number.setValue(param_dict["average_number"])

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
        self.paho_client.subscribe(self.TOPICS)
        sensor_ID = self.ui.sensor_ID.value()
        test_duration = self.ui.test_duration.value()
        test_interval = self.ui.test_interval.value()
        average_number = self.ui.average_number.value()

        self.mqtt_receive_worker.start()
        b_time = time.time()
        start_time_str = time.strftime('%m%d-%H%M%S')
        fls_data = []
        self.append_to_text_edit("test start")
        while (time.time() - b_time < test_duration) and self.running:
            if not self.running:
                return {}, {}
            fls_value = self.fls_detect(sensor_ID, collection_quantity=average_number)
            fls_data.append(fls_value)
            self.append_to_text_edit(f"cycle {len(fls_data)}: fls value = {fls_value}")
            self.event.wait(test_interval)
            if len(fls_data) % 20 == 0:
                self.draw_test_data(np.array(fls_data))
                self.save_data_to_file(fls_data, start_time_str)

        fls_data = np.array(fls_data)
        self.draw_test_data(np.array(fls_data))
        self.save_data_to_file(fls_data, start_time_str)
        self.append_to_text_edit(f"test finished, data saved: fls_stability_{start_time_str}.csv")
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()
        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def fls_detect(self, fls_sensor_ID, collection_quantity=10):
        self.mqtt_receive_worker.reset_fls_process_status()
        self.paho_client.publish("CTL_WM_CF_ControlManager",
                            mcFls.MCFL_CMD_SnapOnce(sensor_id=fls_sensor_ID, collection_quantity=collection_quantity),
                            0)
        fls_value = self.mqtt_receive_worker.wait_fls_process()
        return fls_value

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(message)

        scrollBar = self.ui.textEdit.verticalScrollBar()
        scrollBar.setValue(scrollBar.maximum())

    def draw_test_data(self, fls_data):
        if len(fls_data) > 0:
            self.ax_fls_data.clear()
            self.ax_fls_data.plot(fls_data)
            self.ax_fls_data.set_title(f"3*sigma: {np.std(fls_data) * 3 : .3f} um")
        else:
            self.ax_fls_data.set_title(f"3*sigma: {0} um")
        self.ax_fls_data.set_xlabel("index")
        self.ax_fls_data.set_ylabel("height / um")
        self.res_fig.tight_layout()
        self.res_canvas.draw()

    def save_data_to_file(self, data_to_save, timestamp_suffix=time.strftime('%m%d-%H%M%S')):
        save_file_dir = self.config.DATA_SAVE_ROOT
        save_file_name = f"fls_stability_{timestamp_suffix}.csv"

        save_data_column_name_list = [
            "fls_data"
        ]
        data = np.round(data_to_save, 8)
        pd.DataFrame(data, columns=save_data_column_name_list).to_csv(save_file_dir + save_file_name)

        # 或者使用 grab() 方法（更简洁）
    def save_screenshot_grab(self):
        """使用 grab() 方法截图"""
        pixmap = self.grab()  # 截取整个窗口
        pixmap.save("screenshot.png", "PNG")


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = SpectralConfocalStability()
    test.show()
    test.save_screenshot_grab()
    sys.exit(myapp.exec_())
