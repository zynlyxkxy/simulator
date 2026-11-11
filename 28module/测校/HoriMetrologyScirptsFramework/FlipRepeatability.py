# -*- coding: utf-8 -*-
"""
Filename: FlipRepeatability.py
Date: 2026/8/19
Description:
    翻转重复性测试
    测量原理：翻转模块拾取头吸取芯片并置于粗对准相机视场内，翻转模块 RY 轴在
    "避让角度"与"拍照角度"之间来回旋转，每轮回到拍照角度后硬触发拍照，
    识别芯片标记得到芯片在相机视野中的 X、Y 位置与 Rz 角度，同时记录翻转模块
    自身 Y、Z、Rz、RY 轴读数；多轮统计芯片 X/Y/Rz 的 3σ 重复性。
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
from plc_grpc_client import PlcClient
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import FlipRepeatability_ui as ui

from PyQt5.QtWidgets import QApplication, QVBoxLayout
from PyQt5.QtCore import pyqtSignal
import numpy as np
import pandas as pd
import cv2

import threading
import sys
import os


# 继承TestScriptWidget类
class FlipRepeatability(TestScriptWidget):
    __sig_update_realtime_log = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.__sig_update_realtime_log.connect(self.append_to_text_edit)

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))

        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)
        self.plc_client = PlcClient()

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'flip_ry_avoid': float(self.ui.flip_ry_avoid.value()),
            'flip_ry_photo': float(self.ui.flip_ry_photo.value()),
            'repeat_times': int(self.ui.repeat_times.value()),
            'wait_time': int(self.ui.wait_time.value()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.flip_ry_avoid.setValue(param_dict.get('flip_ry_avoid', 0))
        self.ui.flip_ry_photo.setValue(param_dict.get('flip_ry_photo', 0))
        self.ui.repeat_times.setValue(param_dict.get('repeat_times', 1))
        self.ui.wait_time.setValue(param_dict.get('wait_time', 0))

    def plot_realtime_data(self, data):
        self.ui.table_widget.add_data_row(data)

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        self.paho_client.subscribe(self.TOPICS)
        self.ui.textEdit.clear()

    def get_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                                 mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0,
                                                                       is_image_send=True),
                                 0)
        self.plc_client.camera_trigger(camera_id)
        self.mqtt_receive_worker.wait_get_raw_image()
        raw_image = self.mqtt_receive_worker.get_raw_image()
        return raw_image

    def move_flip_ry(self, ry_pos):
        """翻转模块 RY 轴绝对运动并等待到位"""
        current_axis = 'FlipStage_RY'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                                 mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                     pos=ry_pos,
                                     stage_name=LST_AXIS_NAME[current_axis]["name"],
                                     stage_index=LST_AXIS_NAME[current_axis]["index"],
                                     axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                                 0)
        self.mqtt_receive_worker.wait_stage_stop()

    @staticmethod
    def detect_chip_pose(image):
        """识别图像中最大矩形轮廓（芯片标记），返回中心(x, y)pixel与旋转角(deg)；失败返回None"""
        _, binary = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 100:
            return None
        rect = cv2.minAreaRect(largest_contour)
        (cx, cy), (w, h), angle = rect
        # 归一化到 [-45, 45)，消除长宽边互换引起的90°跳变
        if w < h:
            angle += 90
        angle = ((angle + 45) % 90) - 45
        return cx, cy, angle

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        camera_ID = self.config.camera_dict['粗对准']
        repeat_times = int(self.ui.repeat_times.value())
        wait_time = int(self.ui.wait_time.value())
        flip_ry_avoid = float(self.ui.flip_ry_avoid.value())
        flip_ry_photo = float(self.ui.flip_ry_photo.value())
        cam_setting = self.config.camera_setting.get(camera_ID, self.config.camera_setting[0])
        world_pixel_size = cam_setting['pixel_size'] / cam_setting['magnification']  # mm/pixel

        if repeat_times < 1:
            self.update_realtime_log("重复次数需 >= 1，测试未执行")
            return {}, {}

        self.update_realtime_log(
            f"test start: camera=粗对准, RY avoid angle={flip_ry_avoid}°, photo angle={flip_ry_photo}°, "
            f"repeat={repeat_times}, wait={wait_time}ms")

        save_data_list = []
        for i in range(repeat_times):
            if not self.running:
                return {}, {}

            self.update_realtime_log(f"{'-' * 10}> flip RY to avoid angle: {flip_ry_avoid}°")
            self.move_flip_ry(flip_ry_avoid)
            if not self.running:
                return {}, {}

            self.update_realtime_log(f"{'-' * 10}> flip RY to photo angle: {flip_ry_photo}°")
            self.move_flip_ry(flip_ry_photo)
            if not self.running:
                return {}, {}

            # 到位后稳定等待
            self.event.wait(wait_time / 1000.0)
            if not self.running:
                return {}, {}

            self.update_realtime_log(f"{'-' * 10}> take_photo start")
            raw_image = self.get_photo(camera_ID, "")
            self.update_realtime_log(f"{'-' * 10}> take_photo end")
            if not self.running:
                return {}, {}
            if raw_image is None:
                self.update_realtime_log(f"第{i + 1}轮取图失败，本轮跳过")
                continue

            pose = self.detect_chip_pose(raw_image[:, :])
            if pose is None:
                self.update_realtime_log(f"第{i + 1}轮芯片标记识别失败，本轮跳过")
                continue
            chip_x_pixel, chip_y_pixel, chip_rz_deg = pose
            chip_x_um = chip_x_pixel * world_pixel_size * 1000
            chip_y_um = chip_y_pixel * world_pixel_size * 1000

            # 翻转模块自身 Y/Z/Rz/RY 轴读数
            flip_pos = list(self.mqtt_receive_worker.get_flip_pos())

            self.update_realtime_log(
                f"第{i + 1}轮: chip pixel=({chip_x_pixel:.2f}, {chip_y_pixel:.2f}), Rz={chip_rz_deg:.4f}°, "
                f"flip pos [Y, Z, Rz, RY]={[round(p, 5) for p in flip_pos]}")

            row = [i, *flip_pos, chip_x_um, chip_y_um, chip_rz_deg]
            self.set_realtime_data(row)
            save_data_list.append(row)

        if not save_data_list:
            self.update_realtime_log("无有效轮次，测试结束")
            return {}, {}

        res = np.array([[r[5], r[6], r[7]] for r in save_data_list])
        std_3 = 3 * np.std(res, axis=0)
        self.update_realtime_log(
            f"统计结果: 芯片 X 3sigma = {std_3[0]:.3f} um, Y 3sigma = {std_3[1]:.3f} um, "
            f"Rz 3sigma = {std_3[2]:.4f} deg")

        # 保存测试数据
        saved_columns_name = ['index', 'flip_y', 'flip_z', 'flip_rz', 'flip_ry',
                              'chip_x_um', 'chip_y_um', 'chip_rz_deg']
        save_file_name = f"flip_repeatability_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'coordinate_metrology/' + save_file_name
        os.makedirs(os.path.dirname(full_save_path), exist_ok=True)
        pd.DataFrame(np.array(save_data_list), columns=saved_columns_name).to_csv(full_save_path, index=False)
        self.update_realtime_log(f"test finished, data saved: {full_save_path}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)

    def update_realtime_log(self, message):
        self.__sig_update_realtime_log.emit(message)

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = FlipRepeatability()
    test.show()
    test.set_saved_param({}, {})
    sys.exit(myapp.exec_())
