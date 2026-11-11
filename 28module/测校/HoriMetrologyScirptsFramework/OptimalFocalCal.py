# -*- coding: utf-8 -*-
"""
Filename: OptimalFocalCal.py
Date: 2025/9/1
Description:
    最佳焦面查找
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
from MTF_algorithm import compute_tenengrad, compute_laplacian, compute_brenner_gradient, compute_tenengrad, compute_local_variance, compute_sobel_variance
from plc_grpc_client import PlcClient
# from AlgorithmServer.common_algorithm import ega_nominal_to_real, create_chip_position
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import OptimalFocalCal_ui as ui
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
import copy


# 继承ScriptWidget类
class OptimalFocalCal(ScriptWidget):
    sig_updateScore = pyqtSignal(list, list)

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com", 'CTL_DR_PLC_Motion']
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)
        self.plc_client = PlcClient()

        self.detect_score_fig = Figure()
        self.detect_score_ax = self.detect_score_fig.add_subplot(111)
        self.detect_score_canvas = FigureCanvas(self.detect_score_fig)
        self.detect_score_layout = QVBoxLayout()
        self.detect_score_layout.addWidget(self.detect_score_canvas)
        self.ui.socre_display.setLayout(self.detect_score_layout)
        self.score_plot_update([], [])

        self.sig_updateScore.connect(self.score_plot_update)

        self.ui.sig_move_to_coarse_stage_position.connect(self.move_coarse_stage_to_target_position)


    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'mark_position_x': self.ui.detect_position_X.value(),
            'mark_position_y': self.ui.detect_position_Y.value(),
            'camera_id': self.ui.camera_ID.value(),
            'z_start': float(self.ui.z_start.text()),
            'z_end': float(self.ui.z_end.text()),
            'step': float(self.ui.step.text()),
            'iterations': int(float(self.ui.max_iterations.text())),
            'tolerance': float(self.ui.tolerance.text()),
            "fineAlign": {
                'lst_alignPoints': [

                ]
            }
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.detect_position_X.setValue(param_dict.get('mark_position_x', 0))
        self.ui.detect_position_Y.setValue(param_dict.get('mark_position_y', 0))
        self.ui.camera_ID.setValue(param_dict.get('camera_id', 2))
        self.ui.z_start.setText(str(param_dict.get('z_start', -0.25)))
        self.ui.z_end.setText(str(param_dict.get('z_end', 0.25)))
        self.ui.step.setText(str(param_dict.get('step', 0.1)))
        self.ui.max_iterations.setText(str(param_dict.get('iterations', 5)))
        self.ui.tolerance.setText(str(param_dict.get('tolerance', 0.0025)))

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

        # self.append_to_text_edit("lst_alignPoints" + str(dict_input['a']["fineAlign"]['lst_alignPoints']))
        # return {}, {}

        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()

        mark_position_x = self.ui.detect_position_X.value()
        mark_position_y = self.ui.detect_position_Y.value()
        camera_id = self.ui.camera_ID.value()
        iterations = int(float(self.ui.max_iterations.text()))
        current_iterations = int(float(self.ui.max_iterations.text()))
        current_step = float(self.ui.step.text())
        current_z_start = float(self.ui.z_start.text())
        current_z_end = float(self.ui.z_end.text())
        tolerance = float(self.ui.tolerance.text())

        # MTF_Score = self.GetMTFScores(camera_id)
        # self.append_to_text_edit(f"MTF评价分为:{MTF_Score:.4f}")
        #
        # self.paho_client.unsubscribe(self.TOPICS)
        # self.mqtt_receive_worker.stop()
        # return {}, {}

        # actuator.move_vertical_micro_stage(
        #     self.paho_client, self.mqtt_receive_worker,
        #     target_z=current_z_start
        # )
        # self.paho_client.unsubscribe(self.TOPICS)
        # self.mqtt_receive_worker.stop()
        # return {}, {}

        # actuator.move_coarse_stage([mark_position_x, mark_position_y],
        #                            self.paho_client, self.mqtt_receive_worker)
        best_score = None
        while current_iterations > 0 and current_step >= tolerance and self.running:
            z_values = []
            z = current_z_start
            while z <= current_z_end:
                z_values.append(z)
                z += current_step

            if z_values[-1] != current_z_end:
                z_values.append(current_z_end)

            score_lst = []
            print(f"迭代#{iterations - current_iterations + 1}: 扫描{len(z_values)}个Z位置, "
                                     f"范围[{current_z_start:.4f}, {current_z_end:.4f}], "
                                     f"步长:{current_step:.4f}")

            for current_index, z in enumerate(z_values):
                if not self.running:
                    self.append_to_text_edit(f"test interrupt")
                    return {}, {}

                self.append_to_text_edit(f"{current_index} vertical micro stage move to {z} mm")
                actuator.move_vertical_micro_stage(
                    self.paho_client, self.mqtt_receive_worker,
                    target_z=z
                )

                MTF_Score = self.GetMTFScores(camera_id)
                self.append_to_text_edit(f"焦面{z:.4f}对应的MTF评价分为:{MTF_Score:.4f}")
                score_lst.append(MTF_Score)
            self.sig_updateScore.emit(z_values, score_lst)
            max_val = max(score_lst)
            max_index = score_lst.index(max_val)

            if best_score is None or max_val > best_score:
                best_score = max_val
                best_z = z_values[max_index]

            left_val = score_lst[max_index - 1] if max_index > 0 else None
            right_val = score_lst[max_index + 1] if max_index < len(score_lst) - 1 else None

            if left_val is not None and right_val is not None:
                current_z_start = z_values[max_index - 1]
                current_z_end = z_values[max_index + 1]
            elif left_val is not None:
                current_z_start = z_values[max_index - 1]
            elif right_val is not None:
                current_z_end = z_values[max_index + 1]

            current_step /= 2
            current_iterations -= 1

            self.append_to_text_edit(f"最佳位置: {best_z:.4f}, 分数: {best_score:.4f}, "
                                     f"新范围: [{current_z_start:.4f}, {current_z_end:.4f}], "
                                     f"新步长: {current_step:.4f}")

        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def get_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        print(f"{camera_id}")
        self.paho_client.publish("VSC_Com",
                            mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0,
                                                                  is_image_send=True),
                            0)
        self.plc_client.camera_trigger(camera_id)
        self.mqtt_receive_worker.wait_get_raw_image()
        raw_image = self.mqtt_receive_worker.get_raw_image()
        return raw_image

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    def score_plot_update(self, z_list, score_list):
        self.detect_score_ax.clear()
        self.detect_score_ax.set_title("detect score")
        self.detect_score_ax.set_ylabel("score")
        self.detect_score_ax.set_xlabel("z / mm")
        if len(z_list) != 0 and len(z_list) == len(score_list):
            self.detect_score_ax.plot(z_list, score_list)
        self.detect_score_canvas.draw()

    def GetMTFScores(self, camera_id):
        raw_image = self.get_photo(camera_id, "hik_D2W_W")
        print("get picture done")
        MTF_Score = compute_sobel_variance(raw_image[:, :])
        return MTF_Score

    def closeEvent(self, event):
        self.ui.close_chip_window()

    def move_coarse_stage_to_target_position(self, target_position):
        if self.running:
            if self.ui.child:
                self.ui.child.display_move_error("task running")
            return
        self.paho_client.subscribe(['CTL_DR_ACS_Motion'])
        print(f'move coarse stage to target position: {target_position}')
        actuator.move_coarse_stage(target_position,
                                   self.paho_client, self.mqtt_receive_worker)
        self.paho_client.unsubscribe(['CTL_DR_ACS_Motion'])


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = OptimalFocalCal()
    test.set_saved_param({}, {})
    # test.MqttClient.loop_start()
    test.show()
    # test.run({}, {})
    # test.begin_()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
