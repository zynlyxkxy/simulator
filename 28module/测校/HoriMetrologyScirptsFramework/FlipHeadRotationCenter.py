# -*- coding: utf-8 -*-
"""
Filename: FlipHeadRotationCenter.py
Date: 2025/9/1
Description:
    水平向微动台旋转中心标定
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
from plc_grpc_client import PlcClient
from RotationCalculator import RotationCalculator
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import FlipHeadRotationCenter_ui as ui

from PyQt5.QtWidgets import QApplication, QVBoxLayout
from PyQt5.QtCore import pyqtSignal
import numpy as np
import pandas as pd
import cv2

import threading
import sys


# 继承ScriptWidget类
class FlipHeadRotationCenter(TestScriptWidget):
    __sig_update_realtime_log = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))

        self.paho_client.subscribe(TOPICS)
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)
        self.plc_client = PlcClient()

        self.__sig_update_realtime_log.connect(self.append_to_text_edit)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'flip_rz_1': self.ui.flip_rz_1.value(),
            'flip_rz_2': self.ui.flip_rz_2.value(),
            'repeat_times': self.ui.repeat_times.value()
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.flip_rz_1.setValue(param_dict.get('flip_rz_1', 0))
        self.ui.flip_rz_2.setValue(param_dict.get('flip_rz_2', 0))
        self.ui.repeat_times.setValue(param_dict.get('repeat_times', 1))

    def plot_realtime_data(self, data):
        self.ui.table_widget.add_data_row(data)

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        self.ui.textEdit.clear()

    def get_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.update_realtime_log(f"{camera_id}")
        self.paho_client.publish("VSC_Com",
                                 mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0,
                                                                       is_image_send=True),
                                 0)
        self.plc_client.camera_trigger(camera_id)
        self.mqtt_receive_worker.wait_get_raw_image()
        raw_image = self.mqtt_receive_worker.get_raw_image()
        return raw_image

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()

        camera_ID = self.config.camera_dict['粗对准']
        repeat_times = int(self.ui.repeat_times.value())
        flip_stage_start_rz = self.ui.flip_rz_1.value()
        flip_stage_end_rz = self.ui.flip_rz_2.value()
        self.update_realtime_log("test start")

        save_data_list = []
        for i in range(repeat_times):
            save_data_temp = [i]
            current_axis = 'FlipStage_RZ'
            self.update_realtime_log(f"{'-' * 10}> flip stage to rz position 1: {flip_stage_start_rz}")
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=flip_stage_start_rz,
                                                                       stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                       stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                       axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                                0)
            self.mqtt_receive_worker.wait_stage_stop()
            if not self.running:
                return {}, {}

            self.update_realtime_log(f"{'-' * 10}> take_photo start")
            raw_image = self.get_photo(camera_ID, '')
            self.update_realtime_log(f"{'-' * 10}> take_photo end")
            rect_points_before_rotation = self.calculate_rect_corner_point(raw_image[:, :])
            if len(rect_points_before_rotation) < 4:
                self.update_realtime_log(f"cycle {i}: rect corner detect failed before rotation, skipped")
                continue

            if not self.running:
                return {}, {}

            self.update_realtime_log(f"{'-' * 10}> flip stage to rz position 2: {flip_stage_end_rz}")
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=flip_stage_end_rz,
                                                                       stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                       stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                       axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                                0)
            self.mqtt_receive_worker.wait_stage_stop()

            self.update_realtime_log(f"{'-' * 10}> take_photo start")
            raw_image = self.get_photo(camera_ID, '')
            self.update_realtime_log(f"{'-' * 10}> take_photo end")
            rect_points_after_rotation = self.calculate_rect_corner_point(raw_image[:, :])
            if len(rect_points_after_rotation) < 4:
                self.update_realtime_log(f"cycle {i}: rect corner detect failed after rotation, skipped")
                continue

            if not self.running:
                return {}, {}

            # 旋转前线段的两个端点所处的对准台坐标：
            alignment_pixel_A_before_rotation = [rect_points_before_rotation[0][0], rect_points_before_rotation[0][1]]
            alignment_pixel_B_before_rotation = [rect_points_before_rotation[2][0], rect_points_before_rotation[2][1]]
            save_data_temp.extend(alignment_pixel_A_before_rotation)
            save_data_temp.extend(alignment_pixel_B_before_rotation)

            # 旋转后线段的两个端点所处的对准台坐标：
            alignment_pixel_A_after_rotation = [rect_points_after_rotation[0][0], rect_points_after_rotation[0][1]]
            alignment_pixel_B_after_rotation = [rect_points_after_rotation[2][0], rect_points_after_rotation[2][1]]
            save_data_temp.extend(alignment_pixel_A_after_rotation)
            save_data_temp.extend(alignment_pixel_B_after_rotation)

            # 计算夹角（弧度）
            angle_radians = RotationCalculator.angle_between_segments(alignment_pixel_A_before_rotation,
                                                                      alignment_pixel_B_before_rotation,
                                                                      alignment_pixel_A_after_rotation,
                                                                      alignment_pixel_B_after_rotation)
            self.update_realtime_log(f"两条线段之间的夹角（弧度）为:{angle_radians};两条线段之间的夹角（角度）为:{np.rad2deg(angle_radians)}")
            save_data_temp.extend([np.rad2deg(angle_radians)])

            rotation_center = RotationCalculator.find_rotation_center(alignment_pixel_A_before_rotation,
                                                                      alignment_pixel_A_after_rotation,
                                                                      alignment_pixel_B_before_rotation,
                                                                      alignment_pixel_B_after_rotation)
            self.update_realtime_log(f"旋转中心坐标为:{rotation_center}")
            save_data_temp.extend([rotation_center[0], rotation_center[1]])
            self.set_realtime_data(np.array([int(i), rotation_center[0], rotation_center[1]]))
            save_data_list.append(save_data_temp)

        # save data
        saved_columns_name = ['index', 'line 1 point 1 x', 'line 1 point 1 y', 'line 1 point 2 x', 'line 1 point 2 y',
                              'line 2 point 1 x', 'line 2 point 1 y', 'line 2 point 2 x', 'line 2 point 2 y',
                              'angle_degree', 'rotation_center x', 'rotation_center y']
        save_data = np.array(save_data_list)
        save_file_name = f"flip_head_rotation_center_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'coordinate_metrology/' + save_file_name
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(full_save_path)
        self.update_realtime_log(f"test finished, data saved: {full_save_path}")

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    def update_realtime_log(self, message):
        self.__sig_update_realtime_log.emit(message)

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
    test = FlipHeadRotationCenter()
    test.show()
    test.set_saved_param({}, {})
    sys.exit(myapp.exec_())
