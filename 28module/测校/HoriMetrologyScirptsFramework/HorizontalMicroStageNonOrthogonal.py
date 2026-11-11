# -*- coding: utf-8 -*-
"""
Filename: HorizontalMicroStageNonOrthogonal.py
Date: 2025/9/1
Description:
    水平向微动台非正交标定
    测量原理：分别命令微动台X轴、Y轴单独运动，用相机记录标记点在两个方向上的
    运动轨迹，两条轨迹的夹角与90°的偏差即为X/Y轴的非正交角。
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
import cmdjson.MC_IO_cmd as mcIO
import HorizontalMicroStageNonOrthogonal_ui as ui
from RotationCalculator import RotationCalculator
from plc_grpc_client import PlcClient
import device_actuator as actuator

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


# 继承TestScriptWidget类
class HorizontalMicroStageNonOrthogonal(TestScriptWidget):
    __sig_update_realtime_log = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.thread_event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.thread_event)
        self.plc_client = PlcClient()

        self.__sig_update_realtime_log.connect(self.update_realtime_log)

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
        self.ui.camera_ID.setValue(param_dict.get('camera_ID', 0))
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
        self.paho_client.subscribe(self.TOPICS)

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.thread_event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        camera_ID = self.ui.camera_ID.value()
        template = self.ui.cb_vision_template.text()
        pos_x_1 = float(self.ui.get_pic_pos_x_1.value())
        pos_y_1 = float(self.ui.get_pic_pos_y_1.value())
        pos_x_2 = float(self.ui.get_pic_pos_x_2.value())
        pos_y_2 = float(self.ui.get_pic_pos_y_2.value())
        repeat_time = int(self.ui.repeat_times.value())

        if repeat_time < 1:
            self.append_to_text_edit("重复次数需 >= 1，测试未执行")
            return {}, {}

        save_data_list = []
        for i in range(repeat_time):
            if not self.running:
                return {}, {}

            # ---------- 起点（微动台位置1） ----------
            self.append_to_text_edit(f"{'-' * 10}> micro stage to ({pos_x_1}, {pos_y_1})")
            actuator.move_horizontal_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                                 target_x=pos_x_1, target_y=pos_y_1)
            if not self.running:
                return {}, {}
            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            row_1, col_1 = actuator.take_photo(camera_ID, template, self.paho_client,
                                               self.mqtt_receive_worker, self.plc_client)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end, mark pixel: ({col_1:.2f}, {row_1:.2f})")
            if not self.running:
                return {}, {}

            # ---------- X轴单独运动（位置1 -> 位置2的X） ----------
            self.append_to_text_edit(f"{'-' * 10}> micro stage X to {pos_x_2} (Y keep {pos_y_1})")
            actuator.move_horizontal_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                                 target_x=pos_x_2)
            if not self.running:
                return {}, {}
            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            row_2, col_2 = actuator.take_photo(camera_ID, template, self.paho_client,
                                               self.mqtt_receive_worker, self.plc_client)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end, mark pixel: ({col_2:.2f}, {row_2:.2f})")
            if not self.running:
                return {}, {}

            # ---------- Y轴单独运动（位置1的Y -> 位置2的Y） ----------
            self.append_to_text_edit(f"{'-' * 10}> micro stage Y to {pos_y_2} (X keep {pos_x_2})")
            actuator.move_horizontal_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                                 target_y=pos_y_2)
            if not self.running:
                return {}, {}
            self.append_to_text_edit(f"{'-' * 10}> take_photo start")
            row_3, col_3 = actuator.take_photo(camera_ID, template, self.paho_client,
                                               self.mqtt_receive_worker, self.plc_client)
            self.append_to_text_edit(f"{'-' * 10}> take_photo end, mark pixel: ({col_3:.2f}, {row_3:.2f})")
            if not self.running:
                return {}, {}

            # 像素坐标下的两条运动轨迹（x取列，y取行并取反，与图像坐标系一致）
            p_start = [col_1, -row_1]
            p_x_end = [col_2, -row_2]
            p_y_end = [col_3, -row_3]

            # 微动台X轴与相机X轴(u轴)的夹角（X向轨迹统一取指向+u的方向，带符号倾角）
            vec_x = [p_x_end[0] - p_start[0], p_x_end[1] - p_start[1]]
            if vec_x[0] < 0:
                vec_x = [-vec_x[0], -vec_x[1]]
            x_to_u_rad = float(np.arctan2(vec_x[1], vec_x[0]))

            # X向轨迹向量与Y向轨迹向量的夹角
            angle_radians = RotationCalculator.angle_between_segments(p_start, p_x_end,
                                                                      p_x_end, p_y_end)
            # 非正交角 = 实际夹角 - 90°
            non_ortho_rad = angle_radians - np.pi / 2
            non_ortho_deg = np.rad2deg(non_ortho_rad)
            self.append_to_text_edit(
                f"微动台X轴与相机u轴夹角为:{x_to_u_rad * 1e6:.3f}urad; "
                f"X/Y轨迹夹角为:{np.rad2deg(angle_radians):.6f}°; "
                f"非正交角为:{non_ortho_rad * 1e6:.3f}urad ({non_ortho_deg:.6f}°)")
            self.ui.table_widget.add_data_row([i, x_to_u_rad * 1e6, non_ortho_rad * 1e6, non_ortho_deg])
            save_data_list.append([i, col_1, row_1, col_2, row_2, col_3, row_3,
                                   x_to_u_rad * 1e6, np.rad2deg(angle_radians),
                                   non_ortho_rad * 1e6, non_ortho_deg])
            self.thread_event.wait(1)

        if not save_data_list:
            return {}, {}

        res = np.array(save_data_list)
        self.append_to_text_edit(
            f"微动台X轴与相机u轴平均夹角为:{np.mean(res[:, 7]):.3f}urad; "
            f"平均非正交角为:{np.mean(res[:, 9]):.3f}urad ({np.mean(res[:, 10]):.6f}°); "
            f"非正交角重复性(3sigma)为:{3 * np.std(res[:, 9]):.3f}urad")

        # 保存测试数据
        saved_columns_name = ['repeat',
                              'start point col', 'start point row',
                              'x traj end col', 'x traj end row',
                              'y traj end col', 'y traj end row',
                              'x_to_camera_u_urad', 'angle_degree', 'non_ortho_urad', 'non_ortho_degree']
        save_file_name = f"horizontal_micro_stage_non_orthogonal_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'coordinate_metrology/' + save_file_name
        os.makedirs(os.path.dirname(full_save_path), exist_ok=True)
        pd.DataFrame(np.array(save_data_list), columns=saved_columns_name).to_csv(full_save_path)
        self.append_to_text_edit(f"test finished, data saved: {full_save_path}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.thread_event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self, message):
        self.__sig_update_realtime_log.emit(message)

    def update_realtime_log(self, message):
        self.ui.textEdit.append(message)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = HorizontalMicroStageNonOrthogonal()
    # test.MqttClient.loop_start()
    test.show()
    test.set_saved_param({}, {})
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
