# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic.py
Date: 2025/9/1
Description:
    相机对准重复性
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
import SingleCameraAlignStability_ui as ui

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
class SingleCameraAlignStability(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()
        self.ui.add_camera_dict(self.config.camera_dict)

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.paho_client.subscribe(TOPICS)
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
        INPUT = {
            'camera_ID': self.ui.camera_ID.currentText(),
            'vision_template': self.ui.cb_vision_template.text(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': int(self.ui.wait_time.value()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentText(param_dict['camera_ID'])
        self.ui.cb_vision_template.setText(param_dict['vision_template'])
        self.ui.repeat_times.setValue(param_dict['repeat_time'])
        self.ui.wait_time.setValue(param_dict['wait_time'])

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
        camera_ID = self.config.camera_dict[self.ui.camera_ID.currentText()]
        vision_template = self.ui.cb_vision_template.text()
        repeat_time = int(self.ui.repeat_times.value())
        wait_time = float(self.ui.wait_time.value())
        row_list, col_list = [], []
        self.append_to_text_edit(f"test start")
        for i in range(repeat_time):
            if not self.running:
                break
            self.event.wait(wait_time / 1000.0)
            # 拍照
            row, col = self.take_photo(camera_ID, vision_template)
            row_list.append(row)
            col_list.append(col)
            self.append_to_text_edit(f"cycle time: {i + 1} / {repeat_time}")

        # save data
        saved_columns_name = ['index', 'col', 'row']
        save_file_name = f"single_camera_stability_{time.strftime('%m%d_%H_%M_%S')}.csv"
        save_data = np.array([[i for i in range(len(row_list))], col_list, row_list]).T
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(
            self.config.DATA_SAVE_ROOT + save_file_name, index=False)

        # display result
        self.res_ax_row.plot([i for i in range(len(row_list))], row_list)
        self.res_ax_row.set_title(f"row: {np.std(row_list) * 3 * 0.00274 / 20 * 1e6:.3f} nm")
        self.res_ax_row.set_ylabel("pixel")
        self.res_ax_col.plot([i for i in range(len(col_list))], col_list)
        self.res_ax_col.set_title(f"col: {np.std(col_list) * 3 * 0.00274 / 20 * 1e6:.3f} nm")
        self.res_ax_col.set_ylabel("pixel")
        self.res_canvas.draw()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    def take_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                            mcCamera.VisionCmdJson(id=camera_id, template=template_name, exposure=0),
                            0)
        arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return arr_row, arr_col

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(message)
        scrollBar = self.ui.textEdit.verticalScrollBar()
        scrollBar.setValue(scrollBar.maximum())

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = SingleCameraAlignStability()
    # test.MqttClient.loop_start()
    test.show()
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
