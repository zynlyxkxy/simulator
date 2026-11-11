# -*- coding: utf-8 -*-
"""
Filename: AlignCameraRzCalibration.py
Date: 2025/9/1
Description:
    对准相机倾斜角度标定
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import cmdjson.MC_IO_cmd as mcIO
import AlignCameraRzCalibration_ui as ui
from RotationCalculator import RotationCalculator
import cv2

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
class AlignCameraRzCalibration(ScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(config.MQTTHOST), int(config.MQTTPORT))
        self.paho_client.subscribe(TOPICS)
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.rz_threshold = 1

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(111)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'camera_ID': int(self.ui.camera_ID.value()),
            'vision_template': self.ui.cb_vision_template.text(),
            'coarse_stage_get_pic_pos_x_1': float(self.ui.coarse_stage_get_pic_pos_x_1.value()),
            'coarse_stage_get_pic_pos_y_1': float(self.ui.coarse_stage_get_pic_pos_y_1.value()),
            'coarse_stage_get_pic_pos_x_2': float(self.ui.coarse_stage_get_pic_pos_x_2.value()),
            'coarse_stage_get_pic_pos_y_2': float(self.ui.coarse_stage_get_pic_pos_y_2.value()),
            # 'pixel_size': float(self.ui.pixel_size.value()),
            # 'lens_magnification': float(self.ui.lens_magnification.value()),
            # 'image_center_col': int(self.ui.image_center_col.value()),
            # 'image_center_row': int(self.ui.image_center_row.value()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setValue(param_dict['camera_ID'])
        self.ui.cb_vision_template.setText(param_dict['vision_template'])
        self.ui.coarse_stage_get_pic_pos_x_1.setValue(param_dict['coarse_stage_get_pic_pos_x_1'])
        self.ui.coarse_stage_get_pic_pos_y_1.setValue(param_dict['coarse_stage_get_pic_pos_y_1'])
        self.ui.coarse_stage_get_pic_pos_x_2.setValue(param_dict['coarse_stage_get_pic_pos_x_2'])
        self.ui.coarse_stage_get_pic_pos_y_2.setValue(param_dict['coarse_stage_get_pic_pos_y_2'])
        # self.ui.pixel_size.setValue(param_dict['pixel_size'])
        # self.ui.lens_magnification.setValue(param_dict['lens_magnification'])
        # self.ui.image_center_col.setValue(param_dict['image_center_col'])
        # self.ui.image_center_row.setValue(param_dict['image_center_row'])

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
        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()

        camera_ID = self.ui.camera_ID.value()
        vision_template = self.ui.cb_vision_template.text()
        coarse_stage_get_pic_pos_x_0 = float(self.ui.coarse_stage_get_pic_pos_x_1.value()),
        coarse_stage_get_pic_pos_y_0 = float(self.ui.coarse_stage_get_pic_pos_y_1.value()),
        coarse_stage_get_pic_pos_x_1 = float(self.ui.coarse_stage_get_pic_pos_x_2.value()),
        coarse_stage_get_pic_pos_y_1 = float(self.ui.coarse_stage_get_pic_pos_y_2.value()),

        repeat_times = 10
        res = [[0, 0]] * repeat_times
        for i in range(repeat_times):
            self.append_to_text_edit(f"{'-' * 10}> move to start position")
            current_axis = 'COARSE_X'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_stage_get_pic_pos_x_0,
                                                                            stage_name=LST_AXIS_NAME[current_axis][
                                                                                "name"],
                                                                            stage_index=LST_AXIS_NAME[current_axis][
                                                                                "index"],
                                                                            axis_name=LST_AXIS_NAME[current_axis][
                                                                                "axisName"]),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()

            current_axis = 'COARSE_Y'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_stage_get_pic_pos_y_0,
                                                                            stage_name=LST_AXIS_NAME[current_axis][
                                                                                "name"],
                                                                            stage_index=LST_AXIS_NAME[current_axis][
                                                                                "index"],
                                                                            axis_name=LST_AXIS_NAME[current_axis][
                                                                                "axisName"]),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
            self.append_to_text_edit(f"{'-' * 10}> stage move end")

            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            start_pos_detect_row, start_pos_detect_col = self.take_photo(camera_ID, vision_template)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end")

            self.append_to_text_edit(f"{'-' * 10}> move to end position")
            current_axis = 'COARSE_X'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_stage_get_pic_pos_x_1,
                                                                            stage_name=LST_AXIS_NAME[current_axis][
                                                                                "name"],
                                                                            stage_index=LST_AXIS_NAME[current_axis][
                                                                                "index"],
                                                                            axis_name=LST_AXIS_NAME[current_axis][
                                                                                "axisName"]),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()

            current_axis = 'COARSE_Y'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_stage_get_pic_pos_y_1,
                                                                            stage_name=LST_AXIS_NAME[current_axis][
                                                                                "name"],
                                                                            stage_index=LST_AXIS_NAME[current_axis][
                                                                                "index"],
                                                                            axis_name=LST_AXIS_NAME[current_axis][
                                                                                "axisName"]),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
            self.append_to_text_edit(f"{'-' * 10}> stage move end")
            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            end_pos_detect_row, end_pos_detect_col = self.take_photo(camera_ID, vision_template)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end")

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
        res = np.array(res)
        self.append_to_text_edit(
            f"两条线段之间的平均夹角（弧度）为:{np.mean(res[:, 0])};两条线段之间的平均夹角（角度）为:{np.mean(res[:, 1])}")

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    @staticmethod
    def calculate_rect_corner_point(image):
        _, binary = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        rect_points = approx.reshape(-1, 2)
        return rect_points


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = AlignCameraRzCalibration()
    test.show()
    sys.exit(myapp.exec_())
