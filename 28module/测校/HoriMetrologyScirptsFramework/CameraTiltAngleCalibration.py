# -*- coding: utf-8 -*-
"""
Filename: MicroStageRotationCenter.py
Date: 2025/9/1
Description:
    水平向微动台旋转中心标定
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import cmdjson.MC_IO_cmd as mcIO
import CameraTiltAngleCalibration_ui as ui
from RotationCalculator import RotationCalculator
import device_actuator as actuator

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
class MicroStageRotationCenter(TestScriptWidget):
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
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        # 输入相机参数
        self.pixel_size = 2.74e-3
        self.magnification = 20
        self.image_center_x = 2256
        self.image_center_y = 2256

        self.rz_threshold = 1

        self.ui.add_camera_dict(self.config.camera_dict)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'camera_ID': int(self.ui.camera_ID.currentIndex()),
            'vision_template': self.ui.cb_vision_template.text(),
            'get_pic_pos_x_1': float(self.ui.get_pic_pos_x_1.value()),
            'get_pic_pos_y_1': float(self.ui.get_pic_pos_y_1.value()),
            'get_pic_pos_x_2': float(self.ui.get_pic_pos_x_2.value()),
            'get_pic_pos_y_2': float(self.ui.get_pic_pos_y_2.value()),
            'repeat_time': int(self.ui.repeat_times.value()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentIndex(param_dict.get('camera_ID', 0))
        self.ui.cb_vision_template.setText(param_dict.get('vision_template', 'test'))
        self.ui.get_pic_pos_x_1.setValue(param_dict.get('get_pic_pos_x_1', 0))
        self.ui.get_pic_pos_y_1.setValue(param_dict.get('get_pic_pos_y_1', 0))
        self.ui.get_pic_pos_x_2.setValue(param_dict.get('get_pic_pos_x_2', 0))
        self.ui.get_pic_pos_y_2.setValue(param_dict.get('get_pic_pos_y_2', 0))
        self.ui.repeat_times.setValue(param_dict.get('repeat_time', 1))

    def plot_realtime_data(self, data):
        pass

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    def take_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                                 mcCamera.VisionCmdJson(id=camera_id, template=template_name, exposure=0),
                                 0)
        arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return arr_row, arr_col

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        camera_ID = self.config.camera_dict[self.ui.camera_ID.currentText()]
        template = self.ui.cb_vision_template.text()
        get_pic_pos_x_0 = float(self.ui.get_pic_pos_x_1.value())
        get_pic_pos_y_0 = float(self.ui.get_pic_pos_y_1.value())
        get_pic_pos_x_1 = float(self.ui.get_pic_pos_x_2.value())
        get_pic_pos_y_1 = float(self.ui.get_pic_pos_y_2.value())
        repeat_time = int(self.ui.repeat_times.value())

        start_pos = [get_pic_pos_x_0, get_pic_pos_y_0]
        end_pos = [get_pic_pos_x_1, get_pic_pos_y_1]
        self.ui.table_widget.clear_all_data()
        res = [[0, 0]] * repeat_time
        for i in range(repeat_time):
            if not self.running:
                return {}, {}
            self.append_to_text_edit(f"{'-' * 10}> move to start position")
            stop_pos = actuator.move_coarse_stage(start_pos,
                                                  self.paho_client, self.mqtt_receive_worker)
            self.append_to_text_edit(f"{'-' * 10}> stage move end to {stop_pos}")
            if not self.running:
                return {}, {}

            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            start_pos_detect_row, start_pos_detect_col = self.take_photo(camera_ID, template)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end")
            if not self.running:
                return {}, {}

            self.append_to_text_edit(f"{'-' * 10}> move to end position")
            stop_pos = actuator.move_coarse_stage(end_pos,
                                                  self.paho_client, self.mqtt_receive_worker)
            self.append_to_text_edit(f"{'-' * 10}> stage move end to {stop_pos}")
            if not self.running:
                return {}, {}

            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            end_pos_detect_row, end_pos_detect_col = self.take_photo(camera_ID, template)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end")
            if not self.running:
                return {}, {}

            # 旋转前线段的两个端点所处的对准台坐标：
            alignment_pixel_A_before_rotation = [start_pos_detect_row, start_pos_detect_col]
            alignment_pixel_B_before_rotation = [end_pos_detect_row, end_pos_detect_col]

            # 旋转后线段的两个端点所处的对准台坐标：
            alignment_pixel_A_after_rotation = [0, 0]
            alignment_pixel_B_after_rotation = [1000, 0]

            # 计算夹角（弧度）
            angle_radians = RotationCalculator.angle_between_segments(alignment_pixel_A_before_rotation,
                                                                      alignment_pixel_B_before_rotation,
                                                                      alignment_pixel_A_after_rotation,
                                                                      alignment_pixel_B_after_rotation)
            self.append_to_text_edit(
                f"两条线段之间的夹角（弧度）为:{-(np.deg2rad(90) - angle_radians)};两条线段之间的夹角（角度）为:{-(90 - np.rad2deg(angle_radians))}")
            res[i] = [-(np.deg2rad(90) - angle_radians), -(90 - np.rad2deg(angle_radians))]
            self.ui.table_widget.add_data_row([i, -(np.deg2rad(90) - angle_radians), -(90 - np.rad2deg(angle_radians))])
            self.event.wait(1)
        res = np.array(res)
        self.append_to_text_edit(
            f"两条线段之间的平均夹角（弧度）为:{np.mean(res[:, 0])};两条线段之间的平均夹角（角度）为:{np.mean(res[:, 1])}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = MicroStageRotationCenter()
    # test.MqttClient.loop_start()
    test.show()
    test.set_saved_param({}, {})
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
