# -*- coding: utf-8 -*-
"""
Filename: MicroStageRotationCenter.py
Date: 2025/9/1
Description:
    水平向微动台旋转中心标定
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import cmdjson.MC_IO_cmd as mcIO
import MicroStageRotationCenter_ui as ui
from RotationCalculator import RotationCalculator

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
class MicroStageRotationCenter(ScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(config.MQTTHOST), int(config.MQTTPORT))
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        # 输入相机参数
        self.pixel_size = 2.74e-3
        self.magnification = 20
        self.image_center_x = 2256
        self.image_center_y = 2256

        self.rz_threshold = 1

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'camera_ID': int(self.ui.camera_ID.value()),
            'vision_template': self.ui.cb_vision_template.text(),
            'coarse_stage_get_pic_pos_x_1': float(self.ui.coarse_stage_get_pic_pos_x_1.value()),
            'coarse_stage_get_pic_pos_y_1': float(self.ui.coarse_stage_get_pic_pos_y_1.value()),
            'coarse_stage_get_pic_pos_x_2': float(self.ui.coarse_stage_get_pic_pos_x_2.value()),
            'coarse_stage_get_pic_pos_y_2': float(self.ui.coarse_stage_get_pic_pos_y_2.value()),
            'horizontal_micro_stage_rz_1': float(self.ui.horizontal_micro_stage_rz_1.value()),
            'horizontal_micro_stage_rz_2': float(self.ui.horizontal_micro_stage_rz_2.value()),
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
        self.ui.horizontal_micro_stage_rz_1.setValue(param_dict['horizontal_micro_stage_rz_1'])
        self.ui.horizontal_micro_stage_rz_2.setValue(param_dict['horizontal_micro_stage_rz_2'])

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

    def get_two_mark_position(self, camera_ID, vision_template, start_pos, end_pos):
        self.append_to_text_edit(f"{'-' * 10}> move to start position")

        current_axis = 'COARSE_X'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=start_pos[0],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()
        current_axis = 'COARSE_Y'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=start_pos[1],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()
        self.append_to_text_edit(f"{'-' * 10}> stage move end")

        self.append_to_text_edit(f"{'-' * 10}> take_photo start")
        start_pos_detect_row, start_pos_detect_col = self.take_photo(camera_ID, vision_template)
        self.append_to_text_edit(f"{'-' * 10}> take_photo end")
        coarse_stage_pos = self.mqtt_receive_worker.get_coarse_stage_pos()
        coarse_stage_pos[0] = -coarse_stage_pos[0]
        coarse_stage_pos[1] = -coarse_stage_pos[1]
        start_world_position = RotationCalculator.pixel_to_world([start_pos_detect_col, start_pos_detect_row],
                                                                 coarse_stage_pos,
                                                                 self.pixel_size,
                                                                 self.magnification,
                                                                 self.image_center_x,
                                                                 self.image_center_y)

        self.append_to_text_edit(f"{'-' * 10}> move to end position")
        current_axis = 'COARSE_X'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=end_pos[0],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()
        current_axis = 'COARSE_Y'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=end_pos[1],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()

        self.append_to_text_edit(f"{'-' * 10}> take_photo start")
        end_pos_detect_row, end_pos_detect_col = self.take_photo(camera_ID, vision_template)
        self.append_to_text_edit(f"{'-' * 10}> take_photo end")
        coarse_stage_pos = self.mqtt_receive_worker.get_coarse_stage_pos()
        coarse_stage_pos[0] = -coarse_stage_pos[0]
        coarse_stage_pos[1] = -coarse_stage_pos[1]
        end_world_position = RotationCalculator.pixel_to_world([end_pos_detect_col, end_pos_detect_row],
                                                               coarse_stage_pos,
                                                               self.pixel_size,
                                                               self.magnification,
                                                               self.image_center_x,
                                                               self.image_center_y)
        return start_world_position, end_world_position

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        camera_ID = self.ui.camera_ID.value()
        template = self.ui.cb_vision_template.text()
        coarse_stage_get_pic_pos_x_0 = float(self.ui.coarse_stage_get_pic_pos_x_1.value()),
        coarse_stage_get_pic_pos_y_0 = float(self.ui.coarse_stage_get_pic_pos_y_1.value()),
        coarse_stage_get_pic_pos_x_1 = float(self.ui.coarse_stage_get_pic_pos_x_2.value()),
        coarse_stage_get_pic_pos_y_1 = float(self.ui.coarse_stage_get_pic_pos_y_2.value()),
        horizontal_micro_stage_rz_0 = float(self.ui.horizontal_micro_stage_rz_1.value()),
        horizontal_micro_stage_rz_1 = float(self.ui.horizontal_micro_stage_rz_2.value()),

        self.paho_client.publish("GUI_DR_PLC_IO",
                            mcIO.MCIO_CMD_SetAo(Ao_index=2006, target_value=horizontal_micro_stage_rz_0),
                            0)
        while abs(self.mqtt_receive_worker.get_horizon_micro_stage_pos()[2] - horizontal_micro_stage_rz_0) > self.rz_threshold:
            self.event.wait(0.2)
        start_world_position1, end_world_position1 = self.get_two_mark_position(camera_ID, template,
                                                                                [coarse_stage_get_pic_pos_x_0, coarse_stage_get_pic_pos_y_0],
                                                                                [coarse_stage_get_pic_pos_x_1, coarse_stage_get_pic_pos_y_1])

        self.paho_client.publish("GUI_DR_PLC_IO",
                            mcIO.MCIO_CMD_SetAo(Ao_index=2006, target_value=horizontal_micro_stage_rz_1),
                            0)
        while abs(self.mqtt_receive_worker.get_horizon_micro_stage_pos()[2] - horizontal_micro_stage_rz_1) > self.rz_threshold:
            self.event.wait(0.2)
        start_world_position2, end_world_position2 = self.get_two_mark_position(camera_ID, template,
                                                                                [coarse_stage_get_pic_pos_x_0, coarse_stage_get_pic_pos_y_0],
                                                                                [coarse_stage_get_pic_pos_x_1, coarse_stage_get_pic_pos_y_1])

        # 计算夹角（弧度）
        angle_radians = RotationCalculator.angle_between_segments(start_world_position1,
                                                                  end_world_position1,
                                                                  start_world_position2,
                                                                  end_world_position2)
        print(
            f"两条线段之间的夹角（弧度）为:{angle_radians * 1e6:.3f}urad;两条线段之间的夹角（角度）为:{np.rad2deg(angle_radians):.6f}°")

        rotation_center = RotationCalculator.find_rotation_center(start_world_position1,
                                                                  end_world_position1,
                                                                  start_world_position2,
                                                                  end_world_position2)
        print("旋转中心坐标为:", rotation_center)
        self.ui.table_widget.add_data_row(['0', rotation_center[0], rotation_center[1]])

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

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = MicroStageRotationCenter()
    # test.MqttClient.loop_start()
    test.show()
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
