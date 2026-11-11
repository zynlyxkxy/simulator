# -*- coding: utf-8 -*-
"""
Filename: CoarseStageRxryRotationCalibration.py
Date: 2025/9/1
Description:
 
"""
import copy

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
from plc_grpc_client import PlcClient
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import CoarseStageRxryRotationCalibration_ui as ui
from RotationCalculator import RotationCalculator
from rotation_axis_calibration import calibrate_rotation_axis
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
class CoarseStageRxryRotationCalibration(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()
        self.config = MetrologyConfig()

        # 相机参数
        camera_id = 2
        self.pixel_width = self.config.camera_setting[camera_id]['image_center_x']
        self.pixel_height = self.config.camera_setting[camera_id]['image_center_y']
        self.pixel_size = self.config.camera_setting[camera_id]['pixel_size']
        self.camera_scale = self.config.camera_setting[camera_id]['magnification']

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.plc_client = PlcClient()

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            "wafer_radius": float(self.ui.wafer_radius.value()),
            "chip_width": float(self.ui.chip_width.value()),
            "chip_height": float(self.ui.chip_height.value()),
            "chip_spacing": float(self.ui.chip_spacing.value()),
            "offset_x": float(self.ui.offset_x.value()),
            "offset_y": float(self.ui.offset_y.value()),
            "ega_param": [float(self.ui.ega_param_1.value()), float(self.ui.ega_param_2.value()),
                          float(self.ui.ega_param_3.value()), float(self.ui.ega_param_4.value()),
                          float(self.ui.ega_param_5.value()), float(self.ui.ega_param_6.value())],
            "repeat_time": int(self.ui.repeat_time.value()),
            "camera_id": int(self.ui.camera_ID.value()),
            "vision_template": self.ui.cb_vision_template.text(),
            "fls_to_camera_offset_x": float(self.ui.fls_to_camera_offset_x.value()),
            "fls_to_camera_offset_y": float(self.ui.fls_to_camera_offset_y.value()),
            "rotation_angle": float(self.ui.rotation_angle.value())
        }
        return INPUT, {}

    # # 重写该函数，将所有需要保存的结果整理为list返回，运算得到的机器常数以json文件格式返回
    #     def get_result(self):
    #         result = self.ui.led_answer.text()
    #         mc = self.ui.ted_MC.toPlainText()
    #         return [result], mc

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.wafer_radius.setValue(param_dict.get("wafer_radius", 12 * 25.4 / 2 - 3))
        self.ui.chip_width.setValue(param_dict.get("chip_width", 10))
        self.ui.chip_height.setValue(param_dict.get("chip_height", 10))
        self.ui.chip_spacing.setValue(param_dict.get("chip_spacing", 0.08))
        self.ui.offset_x.setValue(param_dict.get("offset_x", 3.8))
        self.ui.offset_y.setValue(param_dict.get("offset_y", 0))
        ega_param = param_dict.get("ega_param", [-54.2204, -186.045, 0.999983, 1.00003, 0.00280008, 0.002721])
        self.ui.ega_param_1.setValue(ega_param[0])
        self.ui.ega_param_2.setValue(ega_param[1])
        self.ui.ega_param_3.setValue(ega_param[2])
        self.ui.ega_param_4.setValue(ega_param[3])
        self.ui.ega_param_5.setValue(ega_param[4])
        self.ui.ega_param_6.setValue(ega_param[5])
        self.ui.camera_ID.setValue(param_dict.get("camera_id", 0))
        self.ui.repeat_time.setValue(param_dict.get("repeat_time", 0))
        self.ui.cb_vision_template.setText(param_dict.get("vision_template", "test"))
        self.ui.fls_to_camera_offset_x.setValue(param_dict.get("fls_to_camera_offset_x", 0))
        self.ui.fls_to_camera_offset_y.setValue(param_dict.get("fls_to_camera_offset_y", 0))
        self.ui.rotation_angle.setValue(param_dict.get("rotation_angle", 0))

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
        self.mqtt_receive_worker.start()
        self.event.wait(1)
        # 参数设置
        wafer_radius = float(self.ui.wafer_radius.value())
        chip_width = float(self.ui.chip_width.value())
        chip_height = float(self.ui.chip_height.value())
        spacing = float(self.ui.chip_spacing.value())
        offset_x = float(self.ui.offset_x.value())
        offset_y = float(self.ui.offset_y.value())
        ega_params = np.array([
            float(self.ui.ega_param_1.value()), float(self.ui.ega_param_2.value()), float(self.ui.ega_param_3.value()),
            float(self.ui.ega_param_4.value()), float(self.ui.ega_param_5.value()), float(self.ui.ega_param_6.value())
        ])
        camera_id = int(self.ui.camera_ID.value())
        repeat_time = int(self.ui.repeat_time.value())
        vision_template = self.ui.cb_vision_template.text()
        fls_to_camera_offset_x = float(self.ui.fls_to_camera_offset_x.value())
        fls_to_camera_offset_y = float(self.ui.fls_to_camera_offset_y.value())
        rotation_angle = float(self.ui.rotation_angle.value())

        cells_layout = self.create_chip_position(wafer_radius, chip_width, chip_height, spacing)
        marks_layout = self.compute_mark_positions(cells_layout, offset_x, offset_y)
        # select_position_index = random.sample([i for i in range(marks_layout.shape[0])], 30)
        # select_point = marks_layout[select_position_index]
        select_point = self.select_fitting_points(marks_layout, 6, wafer_radius)
        real_position = self.nominal_to_real(select_point, ega_params)

        # 旋转到零位
        actuator.move_vertical_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                           target_rx=0, target_ry=0)
        fls_value_list_before = self.get_multip_mark_fls_value(real_position, camera_id, vision_template,
                                                               2,
                                                               [fls_to_camera_offset_x,fls_to_camera_offset_y])

        # 旋转到指定值
        actuator.move_vertical_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                           target_rx=rotation_angle, target_ry=0)
        fls_value_list_after_rx = self.get_multip_mark_fls_value(real_position, camera_id, vision_template,
                                                                 2,
                                                                 [fls_to_camera_offset_x,fls_to_camera_offset_y])
        if not self.running:
            return {}, {}

        # 旋转到指定值
        actuator.move_vertical_micro_stage(self.paho_client, self.mqtt_receive_worker,
                                           target_rx=0, target_ry=rotation_angle)
        fls_value_list_after_ry = self.get_multip_mark_fls_value(real_position, camera_id, vision_template,
                                                                 2,
                                                                 [fls_to_camera_offset_x,fls_to_camera_offset_y])
        if not self.running:
            return {}, {}

        # 计算旋转轴
        axis_point_rx, axis_dir_rx, stats = calibrate_rotation_axis(
            fls_value_list_before, fls_value_list_after_rx, method='nonlinear', verbose=True
        )

        self.ui.res_text.append("Rx")
        self.display_calibration_result(axis_dir_rx, axis_dir_rx, stats)
        self.ui.res_text.append("\n\n")

        self.ui.res_text.append("Ry")
        axis_point_ry, axis_dir_ry, stats = calibrate_rotation_axis(
            fls_value_list_before, fls_value_list_after_ry, method='nonlinear', verbose=True
        )
        self.display_calibration_result(axis_dir_ry, axis_dir_ry, stats)

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)

    @staticmethod
    def create_chip_position(wafer_radius, chip_width, chip_height, spacing):
        """创建晶圆上的芯片位置"""
        step_x = chip_width + spacing
        step_y = chip_height + spacing

        # 生成x坐标网格
        x_coords = np.arange(-wafer_radius + step_x / 2, wafer_radius, step_x)
        if len(x_coords[x_coords >= 0]) > 0:
            x_coords = x_coords - np.min(x_coords[x_coords >= 0])

        # 生成y坐标网格
        y_coords = np.arange(-wafer_radius + step_y / 2, wafer_radius, step_y)
        if len(y_coords[y_coords >= 0]) > 0:
            y_coords = y_coords - np.min(y_coords[y_coords >= 0])

        xv, yv = np.meshgrid(x_coords, y_coords)
        grid_positions = np.column_stack((xv.ravel(), yv.ravel()))

        # 计算芯片角点到中心的距离（用于边界检查）
        corner_distance = np.sqrt((chip_width / 2) ** 2 + (chip_height / 2) ** 2)

        # 过滤超出晶圆的芯片
        distances = np.sqrt(grid_positions[:, 0] ** 2 + grid_positions[:, 1] ** 2)
        nominal_positions = grid_positions[distances + corner_distance < wafer_radius]

        return nominal_positions

    @staticmethod
    def compute_mark_positions(centers, a, b):
        """计算每个芯片的标记点位置"""
        offsets = np.array([
            [a, b],
            [a, -b],
            [-a, b],
            [-a, -b]
        ])
        marks = centers[:, None, :] + offsets[None, :, :]
        return marks.reshape(-1, 2)

    @staticmethod
    def nominal_to_real(nominal, params):
        real_positions = []
        for (u, v) in nominal:
            x = params[0] + params[2] * u - params[4] * v
            y = params[1] + params[3] * v + params[5] * u
            real_positions.append((x, y))
        return np.array(real_positions)

    @staticmethod
    def select_fitting_points(nominal_positions, points_number, wafer_radius):
        target_radius = wafer_radius * 0.6

        angles = np.linspace(0, 2 * np.pi, points_number, endpoint=False)
        fit_points = []
        for angle in angles:
            target_x = target_radius * np.cos(angle)
            target_y = target_radius * np.sin(angle)
            distances = np.sqrt(
                (nominal_positions[:, 0] - target_x) ** 2 +
                (nominal_positions[:, 1] - target_y) ** 2
            )
            closest_point_index = np.argmin(distances)
            fit_points.append(nominal_positions[closest_point_index])
        return np.array(fit_points)


    def get_mark_position(self, detect_pos, detect_camera_id, detect_vision_template,
                          pixel_size=0.00274, magnification=20, image_center_x=2256, image_center_y=2256):
        detect_res = [0, 0]
        # 到标记位置
        self.append_to_text_edit(f"{'-' * 10}> stage to {detect_pos[0]}, {detect_pos[1]}")
        stay_position = actuator.move_coarse_stage(detect_pos, self.paho_client, self.mqtt_receive_worker)
        print(f"{'-' * 10}> stage move end ({stay_position[0]}, {stay_position[1]})")
        if not self.running:
            return detect_res

        # 图像检测
        print(f"{'-' * 10}> take_photo start")
        row, col = actuator.take_photo(detect_camera_id, detect_vision_template,
                                       self.paho_client, self.mqtt_receive_worker, self.plc_client)
        print(f"{'-' * 10}> take_photo end, detect column and row ({col:.3f}, {row:.3f})")
        if not self.running:
            return detect_res

        coarse_stage_pos = self.mqtt_receive_worker.get_coarse_stage_pos()
        coarse_stage_pos[0] = -coarse_stage_pos[0]
        coarse_stage_pos[1] = -coarse_stage_pos[1]
        detect_res = RotationCalculator.pixel_to_world([col, row],
                                                       coarse_stage_pos,
                                                       pixel_size, magnification, image_center_x, image_center_y)

        return detect_res

    def get_mark_fls_value(self, detect_pos, detect_fls_id):
        fls_value = 0
        print(f"{'-' * 10}> stage to {detect_pos[0]}, {detect_pos[1]}")
        stay_position = actuator.move_coarse_stage(detect_pos, self.paho_client, self.mqtt_receive_worker)
        print(f"{'-' * 10}> stage move end ({stay_position[0]}, {stay_position[1]})")
        if not self.running:
            return fls_value

        print(f"{'-' * 10}> get fls value start")
        fls_value = actuator.fls_detect(detect_fls_id, self.paho_client, self.mqtt_receive_worker)
        print(f"{'-' * 10}> get fls value end")

        return fls_value

    def get_multip_mark_fls_value(self, real_position, camera_id, vision_template, detect_fls_id,
                                  fls_to_camera_offset):
        detect_pos_list = []
        fls_value_list = []
        for i, cur_position in enumerate(real_position):
            if not self.running:
                return fls_value_list
            detect_res = self.get_mark_position(cur_position, camera_id, vision_template,
                                                self.pixel_size, self.camera_scale,
                                                self.pixel_width / 2, self.pixel_height / 2)

            detect_pos_list.append(copy.deepcopy(detect_res))
            self.event.wait(0.3)

        for i, cur_position in enumerate(detect_pos_list):
            if not self.running:
                return fls_value_list
            target_position = [cur_position[0] + fls_to_camera_offset[0],
                               cur_position[1] + fls_to_camera_offset[1]]
            detect_res = self.get_mark_fls_value(target_position, detect_fls_id)
            coarse_coordinate = self.mqtt_receive_worker.get_coarse_stage_pos()

            fls_value_list.append([coarse_coordinate[0], coarse_coordinate[1], detect_res])
            self.event.wait(0.3)

        return fls_value_list

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    def display_calibration_result(self, axis_dir, axis_point, stats):
        self.ui.res_text.clear()
        self.ui.res_text.append(f"\n标定结果:")
        self.ui.res_text.append(f"  轴方向向量: [{axis_dir[0]:.8f}, {axis_dir[1]:.8f}, {axis_dir[2]:.8f}]")
        self.ui.res_text.append(f"  轴上一点:   [{axis_point[0]:.6f}, {axis_point[1]:.6f}, {axis_point[2]:.6f}]")

        if 'residual_mean' in stats:
            self.ui.res_text.append(f"\n拟合质量:")
            self.ui.res_text.append(f"  平均残差: {stats['residual_mean']:.6f}")
            self.ui.res_text.append(f"  残差标准差: {stats.get('residual_std', 'N/A')}")

        if 'angle_degrees' in stats:
            self.ui.res_text.append(f"  旋转角度: {stats['angle_degrees']:.4f}°")



if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = CoarseStageRxryRotationCalibration()
    # test.MqttClient.loop_start()
    test.show()
    test.set_saved_param({}, {})
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
