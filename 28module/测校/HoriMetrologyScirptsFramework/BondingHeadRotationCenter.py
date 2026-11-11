# -*- coding: utf-8 -*-
"""
Filename: BondingHeadRotationCenter.py
Date: 2026/8/19
Description:
    键合头旋转中心标定
    测量原理：将键合头移动至下精对准相机视野内（人工完成），工具依次提示操作员
    将键合头 Rz 旋转至检测点表格中的各个角度，每个角度拍照并识别标记矩形角点，
    取标记中心为一个圆周点；全部角度的圆周点最小二乘拟合圆，圆心即旋转中心，
    半径即标记相对旋转中心的偏心量。
    注：框架内无键合头 Rz 轴的运动指令通道，旋转由操作员在设备软件中手动完成。
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
import BondingHeadRotationCenter_ui as ui

from PyQt5.QtWidgets import QApplication, QVBoxLayout, QMessageBox
from PyQt5.QtCore import pyqtSignal
import numpy as np
import pandas as pd
import cv2

import threading
import sys
import os


# 继承TestScriptWidget类
class BondingHeadRotationCenter(TestScriptWidget):
    __sig_update_realtime_log = pyqtSignal(str)
    __sig_operator_confirm = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()
        self._confirm_event = threading.Event()

        self.__sig_update_realtime_log.connect(self.append_to_text_edit)
        self.__sig_operator_confirm.connect(self._show_operator_confirm)

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))

        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)
        self.plc_client = PlcClient()

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        detect_point_info = self.ui.detect_point_info_table_widget.get_all_data()
        INPUT = {
            'detect_point_info': detect_point_info,
            'repeat_times': int(self.ui.repeat_times.value()),
            'camera_index': int(self.ui.cb_camera.currentIndex()),
            'bonding_head_index': int(self.ui.cb_bonding_head.currentIndex()),
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.detect_point_info_table_widget.set_table_data(param_dict.get('detect_point_info', []))
        self.ui.repeat_times.setValue(param_dict.get('repeat_times', 1))
        camera_index = param_dict.get('camera_index', 0)
        if camera_index >= 0:
            self.ui.cb_camera.setCurrentIndex(camera_index)
        bonding_head_index = param_dict.get('bonding_head_index', 0)
        if bonding_head_index >= 0:
            self.ui.cb_bonding_head.setCurrentIndex(bonding_head_index)

    def plot_realtime_data(self, data):
        self.ui.table_widget.add_data_row(data)

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        self.paho_client.subscribe(self.TOPICS)
        self.ui.textEdit.clear()

    # ---------- 操作员人工旋转确认（GUI线程弹窗，工作线程等待） ----------
    def _show_operator_confirm(self, message):
        QMessageBox.information(self, "人工操作", message)
        self._confirm_event.set()

    def _wait_operator_confirm(self, message):
        self._confirm_event.clear()
        self.__sig_operator_confirm.emit(message)
        while not self._confirm_event.wait(0.2):
            if not self.running:
                return False
        return self.running

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

    @staticmethod
    def fit_circle(points):
        """最小二乘圆拟合，points: Nx2 [x, y]，返回圆心(cx, cy)与半径R"""
        points = np.array(points, dtype=float)
        x = points[:, 0]
        y = points[:, 1]
        A = np.c_[x, y, np.ones(len(x))]
        b = -(x ** 2 + y ** 2)
        D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
        cx, cy = -D / 2, -E / 2
        R = float(np.sqrt(max((D * D + E * E) / 4 - F, 0)))
        return cx, cy, R

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.ui.table_widget.clear_all_data()
        self.running = True
        self.event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        camera_ID = self.config.camera_dict[self.ui.cb_camera.currentText()]
        bonding_head_name = self.ui.cb_bonding_head.currentText()
        repeat_times = int(self.ui.repeat_times.value())
        cam_setting = self.config.camera_setting.get(camera_ID, self.config.camera_setting[0])
        world_pixel_size = cam_setting['pixel_size'] / cam_setting['magnification']  # mm/pixel

        # 检测点角度列表
        angle_list = []
        for row in self.ui.detect_point_info_table_widget.get_all_data():
            try:
                angle_list.append(float(row[0]))
            except (TypeError, ValueError, IndexError):
                continue
        if len(angle_list) < 3:
            self.update_realtime_log("检测点角度数量不足（至少3个），测试未执行")
            return {}, {}
        if repeat_times < 1:
            self.update_realtime_log("重复次数需 >= 1，测试未执行")
            return {}, {}

        self.update_realtime_log(
            f"test start: camera={self.ui.cb_camera.currentText()}, bonding head={bonding_head_name}, "
            f"angles={angle_list}, repeat={repeat_times}")

        save_data_list = []
        for i in range(repeat_times):
            circle_points = []
            angle_value_list = []
            for ang in angle_list:
                if not self.running:
                    return {}, {}
                if not self._wait_operator_confirm(
                        f"第{i + 1}轮：请使用设备软件将键合头 Rz 旋转至 {ang}°，"
                        f"确认标记在视野内后点击确定"):
                    return {}, {}

                self.update_realtime_log(f"{'-' * 10}> take_photo start (angle={ang}°)")
                raw_image = self.get_photo(camera_ID, "")
                self.update_realtime_log(f"{'-' * 10}> take_photo end")
                if not self.running:
                    return {}, {}
                if raw_image is None:
                    self.update_realtime_log(f"{'-' * 10}> 取图失败，跳过角度 {ang}°")
                    continue

                rect_points = self.calculate_rect_corner_point(raw_image[:, :])
                if rect_points is None or len(rect_points) < 4:
                    self.update_realtime_log(f"{'-' * 10}> 标记角点识别失败，跳过角度 {ang}°")
                    continue

                # 标记中心 = 矩形四角点均值（x=列, y=行）
                mark_center = rect_points.mean(axis=0)
                circle_points.append(mark_center)
                angle_value_list.append(ang)
                self.update_realtime_log(
                    f"{'-' * 10}> mark center pixel: ({mark_center[0]:.2f}, {mark_center[1]:.2f})")

            if len(circle_points) < 3:
                self.update_realtime_log(f"第{i + 1}轮有效角度不足3个，本轮跳过")
                continue

            cx, cy, radius_pixel = self.fit_circle(circle_points)
            radius_um = radius_pixel * world_pixel_size * 1000
            self.update_realtime_log(
                f"第{i + 1}轮: 旋转中心 = ({cx:.3f}, {cy:.3f}) pixel, "
                f"偏心半径 = {radius_pixel:.2f} pixel = {radius_um:.2f} um")
            self.set_realtime_data([i, cx, cy, radius_um])
            save_data_list.append([i, *angle_value_list,
                                   *[round(v, 4) for p in circle_points for v in p],
                                   cx, cy, radius_pixel, radius_um])
            self.event.wait(1)

        if not save_data_list:
            self.update_realtime_log("无有效轮次，测试结束")
            return {}, {}

        res_center = np.array([[r[-4], r[-3]] for r in save_data_list])
        res_radius = np.array([r[-1] for r in save_data_list])
        self.update_realtime_log(
            f"统计结果: 平均旋转中心 = ({np.mean(res_center[:, 0]):.3f}, {np.mean(res_center[:, 1]):.3f}) pixel, "
            f"中心重复性(3sigma) = ({3 * np.std(res_center[:, 0]):.3f}, {3 * np.std(res_center[:, 1]):.3f}) pixel; "
            f"平均偏心半径 = {np.mean(res_radius):.2f} um, 半径重复性(3sigma) = {3 * np.std(res_radius):.2f} um")

        # 保存测试数据（跳过因识别失败导致列数不一致的轮次）
        full_row_len = 5 + 3 * len(angle_list)
        valid_rows = [r for r in save_data_list if len(r) == full_row_len]
        if len(valid_rows) < len(save_data_list):
            self.update_realtime_log(
                f"{len(save_data_list) - len(valid_rows)} 轮因部分角度识别失败，数据列不全，未写入CSV")
        n_angles = len(angle_list)
        saved_columns_name = (['index']
                              + [f'angle {k}' for k in range(n_angles)]
                              + [f'point {k} {axis}' for k in range(n_angles) for axis in ('x', 'y')]
                              + ['rotation_center x', 'rotation_center y', 'radius_pixel', 'radius_um'])
        save_file_name = f"bonding_head_rotation_center_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'coordinate_metrology/' + save_file_name
        os.makedirs(os.path.dirname(full_save_path), exist_ok=True)
        pd.DataFrame(np.array(valid_rows), columns=saved_columns_name).to_csv(full_save_path)
        self.update_realtime_log(f"test finished, data saved: {full_save_path}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self._confirm_event.set()
        self.running = False
        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)

    def update_realtime_log(self, message):
        self.__sig_update_realtime_log.emit(message)

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    @staticmethod
    def calculate_rect_corner_point(image):
        """识别图像中最大矩形轮廓的角点；识别失败返回None"""
        _, binary = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        if len(approx) < 4:
            return None
        rect_points = approx.reshape(-1, 2)
        return rect_points

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = BondingHeadRotationCenter()
    test.show()
    test.set_saved_param({}, {})
    sys.exit(myapp.exec_())
