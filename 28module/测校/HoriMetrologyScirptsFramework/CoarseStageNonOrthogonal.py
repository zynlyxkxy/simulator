# -*- coding: utf-8 -*-
"""
Filename: CoarseStageNonOrthogonal.py
Date: 2025/9/1
Description:
    粗动台非正交校准
    测量原理：分别控制粗动台沿X向、Y向运动到两个拍照位置，相机拍摄测校版上的
    标记并换算世界坐标；X向两位置连线倾角为运动轨迹相对相机u轴的倾角，Y向同理，
    两者之差即为Y轴相对X轴的非正交角；进一步结合X/Y理论间隔计算缩放系数，
    组成2x2补偿矩阵并输出正/逆矩阵。
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import CoarseStageNonOrthogonal_ui as ui
from non_orthogonal_model import non_orthogonal_model
from RotationCalculator import RotationCalculator

from plc_grpc_client import PlcClient
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
import time
import os


# 继承TestScriptWidget类
class CoarseStageNonOrthogonal(TestScriptWidget):
    __sig_update_log = pyqtSignal(str)
    __sig_update_res = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.__sig_update_log.connect(self.update_log)
        self.__sig_update_res.connect(self.update_res)

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]

        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)
        self.plc_client = PlcClient()

        self.pixel_size = self.config.camera_setting[0]['pixel_size']
        self.magnification = self.config.camera_setting[0]['magnification']
        self.image_center_x = self.config.camera_setting[0]['image_center_x']
        self.image_center_y = self.config.camera_setting[0]['image_center_y']

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'x_get_pic_pos_x_1': float(self.ui.x_get_pic_pos_x_1.value()),
            'x_get_pic_pos_y_1': float(self.ui.x_get_pic_pos_y_1.value()),
            'x_get_pic_pos_x_2': float(self.ui.x_get_pic_pos_x_2.value()),
            'x_get_pic_pos_y_2': float(self.ui.x_get_pic_pos_y_2.value()),
            'y_get_pic_pos_x_1': float(self.ui.y_get_pic_pos_x_1.value()),
            'y_get_pic_pos_y_1': float(self.ui.y_get_pic_pos_y_1.value()),
            'y_get_pic_pos_x_2': float(self.ui.y_get_pic_pos_x_2.value()),
            'y_get_pic_pos_y_2': float(self.ui.y_get_pic_pos_y_2.value()),
            'x_interval': float(self.ui.x_interval.value()),
            'y_interval': float(self.ui.y_interval.value()),
            'camera_ID': int(self.ui.camera_ID.value()),
            'vision_template': self.ui.cb_vision_template.text(),
            "repeat_time": int(self.ui.repeat_time.value()),
        }
        return INPUT, {}


    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setValue(param_dict.get("camera_ID", 0))
        self.ui.repeat_time.setValue(param_dict.get("repeat_time", 1))
        self.ui.cb_vision_template.setText(param_dict.get("vision_template", ""))
        self.ui.x_get_pic_pos_x_1.setValue(param_dict.get('x_get_pic_pos_x_1', 0))
        self.ui.x_get_pic_pos_y_1.setValue(param_dict.get('x_get_pic_pos_y_1', 0))
        self.ui.x_get_pic_pos_x_2.setValue(param_dict.get('x_get_pic_pos_x_2', 0))
        self.ui.x_get_pic_pos_y_2.setValue(param_dict.get('x_get_pic_pos_y_2', 0))
        self.ui.y_get_pic_pos_x_1.setValue(param_dict.get('y_get_pic_pos_x_1', 0))
        self.ui.y_get_pic_pos_y_1.setValue(param_dict.get('y_get_pic_pos_y_1', 0))
        self.ui.y_get_pic_pos_x_2.setValue(param_dict.get('y_get_pic_pos_x_2', 0))
        self.ui.y_get_pic_pos_y_2.setValue(param_dict.get('y_get_pic_pos_y_2', 0))
        self.ui.x_interval.setValue(param_dict.get('x_interval', 0))
        self.ui.y_interval.setValue(param_dict.get('y_interval', 0))

    def plot_realtime_data(self, data):
        pass

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        self.paho_client.subscribe(self.TOPICS)
        self.ui.textEdit.clear()
        self.ui.res_textedit.clear()

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()
        self.event.wait(1)

        # 参数设置
        camera_id = int(self.ui.camera_ID.value())
        repeat_time = int(self.ui.repeat_time.value())
        vision_template = self.ui.cb_vision_template.text()
        x_stage_coordinate_list = [[self.ui.x_get_pic_pos_x_1.value(), self.ui.x_get_pic_pos_y_1.value()],
                                   [self.ui.x_get_pic_pos_x_2.value(), self.ui.x_get_pic_pos_y_2.value()]]  # x向坐标列表
        y_stage_coordinate_list = [[self.ui.y_get_pic_pos_x_1.value(), self.ui.y_get_pic_pos_y_1.value()],
                                   [self.ui.y_get_pic_pos_x_2.value(), self.ui.y_get_pic_pos_y_2.value()]]  # y向坐标列表

        X_interval = self.ui.x_interval.value()  # X向理论间隔
        Y_interval = self.ui.y_interval.value()  # Y向理论间隔

        if repeat_time < 1:
            self.append_to_text_edit("重复次数需 >= 1，测试未执行")
            return {}, {}
        if X_interval == 0 or Y_interval == 0:
            self.append_to_text_edit("X/Y理论间隔不能为0，测试未执行")
            return {}, {}

        save_data_list = []
        for i in range(repeat_time):
            if not self.running:
                return {}, {}

            self.append_to_text_edit(f"{'=' * 10}> repeat {i + 1}/{repeat_time}")
            detect_x_stage_position = []
            detect_x_ifm_position = []
            for cur_x_coordinate in x_stage_coordinate_list:
                if not self.running:
                    return {}, {}
                pos_grating, pos_ifm = self.get_mark_stage_position(camera_id, vision_template, cur_x_coordinate)
                detect_x_stage_position.append(pos_grating)
                detect_x_ifm_position.append(pos_ifm)
            detect_x_stage_position = np.array(detect_x_stage_position)
            detect_x_ifm_position = np.array(detect_x_ifm_position)

            detect_y_stage_position = []
            detect_y_ifm_position = []
            for cur_y_coordinate in y_stage_coordinate_list:
                if not self.running:
                    return {}, {}
                pos_grating, pos_ifm = self.get_mark_stage_position(camera_id, vision_template, cur_y_coordinate)
                detect_y_stage_position.append(pos_grating)
                detect_y_ifm_position.append(pos_ifm)
            detect_y_stage_position = np.array(detect_y_stage_position)
            detect_y_ifm_position = np.array(detect_y_ifm_position)

            X_rotation = non_orthogonal_model.X_rotation(coords=detect_x_stage_position)
            Y_rotation = non_orthogonal_model.Y_rotation(coords=detect_y_stage_position)
            non_ortho_rad = Y_rotation - X_rotation
            self.append_to_res_text_edit(
                f"{'=' * 10}> 第{i + 1}次测量\n"
                f"X向运动轨迹倾角（相对相机u轴）= {X_rotation:.6e}rad ({X_rotation * 1e6:.3f}urad)\n"
                f"Y向运动轨迹倾角（相对相机v轴）= {Y_rotation:.6e}rad ({Y_rotation * 1e6:.3f}urad)\n"
                f"Y轴相对X轴的非正交角 = {non_ortho_rad:.6e}rad = {non_ortho_rad * 1e6:.3f}urad "
                f"({np.rad2deg(non_ortho_rad):.6f}°)")

            compensated_X_coords = non_orthogonal_model.Compensated_rotation(detect_x_stage_position, X_rotation)
            compensated_Y_coords = non_orthogonal_model.Compensated_rotation(detect_y_stage_position, X_rotation)

            a = np.mean(np.diff(compensated_X_coords[:, 0]) / X_interval)
            b = -np.tan(non_ortho_rad)
            c = 0
            d = np.mean(np.diff(compensated_Y_coords[:, 1]) / Y_interval)
            matrix = np.array([[a, b], [c, d]])
            matrix_inv = np.linalg.inv(matrix)
            self.append_to_res_text_edit(f'正向matrix：\n{np.round(matrix, 6)}\n'
                                         f'逆向matrix：\n{np.round(matrix_inv, 6)}')

            # 干涉仪模型非正交参数 sx：与光栅尺路径共用拍照流程，
            # 标记世界坐标改用干涉仪读数换算，sx = X向轨迹倾角 - Y向轨迹倾角
            X_rotation_ifm = non_orthogonal_model.X_rotation(coords=detect_x_ifm_position)
            Y_rotation_ifm = non_orthogonal_model.Y_rotation(coords=detect_y_ifm_position)
            sx_rad = X_rotation_ifm - Y_rotation_ifm
            self.append_to_res_text_edit(
                f"干涉仪 X向轨迹倾角 = {X_rotation_ifm:.6e}rad ({X_rotation_ifm * 1e6:.3f}urad)\n"
                f"干涉仪 Y向轨迹倾角 = {Y_rotation_ifm:.6e}rad ({Y_rotation_ifm * 1e6:.3f}urad)\n"
                f"干涉仪模型中的非正交参数 sx = {sx_rad:.6e}rad = {sx_rad * 1e6:.3f}urad "
                f"({np.rad2deg(sx_rad):.6f}°)")

            save_data_list.append([i,
                                   *detect_x_stage_position.flatten().tolist(),
                                   *detect_y_stage_position.flatten().tolist(),
                                   X_rotation, Y_rotation,
                                   non_ortho_rad, non_ortho_rad * 1e6, np.rad2deg(non_ortho_rad),
                                   a, b, d,
                                   *detect_x_ifm_position.flatten().tolist(),
                                   *detect_y_ifm_position.flatten().tolist(),
                                   X_rotation_ifm, Y_rotation_ifm,
                                   sx_rad, sx_rad * 1e6, np.rad2deg(sx_rad)])
            self.event.wait(1)

        if not save_data_list:
            return {}, {}

        res = np.array(save_data_list)
        mean_matrix = np.array([[np.mean(res[:, 14]), np.mean(res[:, 15])],
                                [0, np.mean(res[:, 16])]])
        self.append_to_res_text_edit(
            f"{'=' * 10}> 统计结果（共{len(save_data_list)}次）\n"
            f"平均非正交角 = {np.mean(res[:, 12]):.3f}urad ({np.mean(res[:, 13]):.6f}°)\n"
            f"非正交角重复性(3sigma) = {3 * np.std(res[:, 12]):.3f}urad\n"
            f"平均补偿矩阵 = \n{np.round(mean_matrix, 6)}\n"
            f"平均干涉仪非正交参数 sx = {np.mean(res[:, 28]):.3f}urad ({np.mean(res[:, 29]):.6f}°)\n"
            f"sx 重复性(3sigma) = {3 * np.std(res[:, 28]):.3f}urad\n"
            f"注：sx 需经运动控制或干涉仪负责人确认后，填入 dashboard 干涉仪非正交参数 sxy")

        # 保存测试数据
        saved_columns_name = ['repeat',
                              'x pos1 x', 'x pos1 y', 'x pos2 x', 'x pos2 y',
                              'y pos1 x', 'y pos1 y', 'y pos2 x', 'y pos2 y',
                              'x_rotation_rad', 'y_rotation_rad',
                              'non_ortho_rad', 'non_ortho_urad', 'non_ortho_degree',
                              'matrix_a', 'matrix_b', 'matrix_d',
                              'ifm x pos1 x', 'ifm x pos1 y', 'ifm x pos2 x', 'ifm x pos2 y',
                              'ifm y pos1 x', 'ifm y pos1 y', 'ifm y pos2 x', 'ifm y pos2 y',
                              'x_rotation_ifm_rad', 'y_rotation_ifm_rad',
                              'sx_rad', 'sx_urad', 'sx_degree']
        save_file_name = f"coarse_stage_non_orthogonal_{time.strftime('%m%d_%H%M_%S')}.csv"
        full_save_path = self.config.DATA_SAVE_ROOT + 'coordinate_metrology/' + save_file_name
        os.makedirs(os.path.dirname(full_save_path), exist_ok=True)
        pd.DataFrame(np.array(save_data_list), columns=saved_columns_name).to_csv(full_save_path)
        self.append_to_text_edit(f"test finished, data saved: {full_save_path}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()
        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()
        self.paho_client.unsubscribe(self.TOPICS)

    def take_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                            mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0),
                            0)
        self.plc_client.camera_trigger(camera_id)
        arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return arr_row, arr_col

    def get_mark_stage_position(self, camera_ID, vision_template, coarse_coordinate, inv_matrix=None):
        self.append_to_text_edit(f"{'-' * 10}> move to position ({coarse_coordinate})")

        current_axis = 'COARSE_X'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_coordinate[0],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()
        current_axis = 'COARSE_Y'
        self.paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_coordinate[1],
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                                   axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                            0)
        self.mqtt_receive_worker.wait_stage_stop()
        self.append_to_text_edit(f"{'-' * 10}> stage move end")

        self.append_to_text_edit(f"{'-' * 10}> take_photo start")
        start_pos_detect_row, start_pos_detect_col = self.take_photo(camera_ID, vision_template)
        self.append_to_text_edit(f"{'-' * 10}> take_photo end, mark pixel: "
                                 f"({start_pos_detect_col:.2f}, {start_pos_detect_row:.2f})")

        # 复制光栅尺读数，避免直接修改worker内部列表
        coarse_stage_pos = list(self.mqtt_receive_worker.get_coarse_stage_pos())
        if inv_matrix is not None:
            coarse_stage_pos, _ = non_orthogonal_model.apply_compensation(coarse_stage_pos, inv_matrix)
        coarse_stage_pos[0] = -coarse_stage_pos[0]
        coarse_stage_pos[1] = -coarse_stage_pos[1]
        world_position = RotationCalculator.pixel_to_world([start_pos_detect_col, start_pos_detect_row],
                                                           coarse_stage_pos,
                                                           self.pixel_size, self.magnification,
                                                           self.image_center_x, self.image_center_y)
        self.append_to_text_edit(f"{'-' * 10}> calculate position "
                                 f"({-world_position[0]:.4f}, {-world_position[1]:.4f})")

        # 干涉仪路径：同一标记像素坐标，位置基准换成干涉仪读数（原始读数单位为nm，换算为mm）
        ifm_data = list(self.mqtt_receive_worker.get_interferometer_data())
        ifm_pos = [-ifm_data[0] / 1e6, -ifm_data[1] / 1e6]
        world_position_ifm = RotationCalculator.pixel_to_world([start_pos_detect_col, start_pos_detect_row],
                                                               ifm_pos,
                                                               self.pixel_size, self.magnification,
                                                               self.image_center_x, self.image_center_y)
        self.append_to_text_edit(f"{'-' * 10}> calculate position by interferometer "
                                 f"({-world_position_ifm[0]:.4f}, {-world_position_ifm[1]:.4f})")
        return [-world_position[0], -world_position[1]], [-world_position_ifm[0], -world_position_ifm[1]]

    def append_to_text_edit(self, message):
        self.__sig_update_log.emit(message)

    def append_to_res_text_edit(self, message):
        self.__sig_update_res.emit(message)

    def update_log(self, message):
        self.ui.textEdit.append(message)

    def update_res(self, message):
        self.ui.res_textedit.append(message)


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = CoarseStageNonOrthogonal()
    # test.MqttClient.loop_start()
    test.show()
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
