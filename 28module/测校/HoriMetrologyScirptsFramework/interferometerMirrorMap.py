# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic.py
Date: 2025/9/1
Description:
    干涉仪mirror map
"""
import time

from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import interferometerMirrorMap_ui as ui
from MirrorTextX import calculate_y_mirror
from MirrorTextY import calculate_x_mirror
from plc_grpc_client import PlcClient

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pandas as pd
import numpy as np
import threading
import random
import json
import sys
import os


# 继承ScriptWidget类
class interferometerMirrorMap(ScriptWidget):
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

        self.res_fig = Figure()
        self.res_ax_x_mirror_TXY = self.res_fig.add_subplot(221)
        self.res_ax_x_mirror_Rz = self.res_fig.add_subplot(222)
        self.res_ax_y_mirror_TXY = self.res_fig.add_subplot(223)
        self.res_ax_y_mirror_Rz = self.res_fig.add_subplot(224)
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)
        self.res_canvas.draw()

        self.show_mirror_map()

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            "start_x": self.ui.start_x.value(),
            "end_x": self.ui.end_x.value(),
            "start_y": self.ui.start_y.value(),
            "end_y": self.ui.end_y.value(),
            "step": self.ui.step.value()
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.start_x.setValue(param_dict.get("start_x", 0))
        self.ui.end_x.setValue(param_dict.get("end_x", 0))
        self.ui.start_y.setValue(param_dict.get("start_y", 0))
        self.ui.end_y.setValue(param_dict.get("end_y", 0))
        self.ui.step.setValue(param_dict.get("step", 0))

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
        self.mqtt_receive_worker.start()
        start_x = self.ui.start_x.value()
        end_x = self.ui.end_x.value()
        start_y = self.ui.start_y.value()
        end_y = self.ui.end_y.value()
        step = self.ui.step.value()
        if step <= 0:
            self.append_to_text_edit("步长必须大于 0，测试终止")
            self.mqtt_receive_worker.stop()
            return {}, {}

        sample_number_in_one_position = 50
        timestamp_str = time.strftime("%m%d%H%M")
        # x运动，保存文件名
        x_file_save_name = (self.config.DATA_SAVE_ROOT + "infer_mirror_SetY0_"
                            + str(sample_number_in_one_position) + "_"
                            + timestamp_str + "_cycle_0.csv")
        # y运动，保存文件名
        y_file_save_name = (self.config.DATA_SAVE_ROOT + "infer_mirror_SetX0_"
                            + str(sample_number_in_one_position) + "_"
                            + timestamp_str + "_cycle_0.csv")
        save_data_column_name = ['hx1', 'hx2', 'hy1', 'hy2', 'nominal x', 'nominal y',
                                 'interferometer x', 'interferometer y', 'encoder x', 'encoder y']

        # 粗动台回零并将干涉仪读数清零
        self.append_to_text_edit("move coarse stage to (0, 0)")
        self._move_one_axis('COARSE_X', 0)
        self._move_one_axis('COARSE_Y', 0)
        self.plc_client.interferometer_set_zero()
        self.append_to_text_edit("interferometer set zero")

        # X向扫描：X轴按步长逐点移动，每点采集 sample_number_in_one_position 次
        x_rows = []
        n_x = int(round((end_x - start_x) / step)) + 1
        for i in range(n_x):
            if not self.running:
                self.mqtt_receive_worker.stop()
                return {}, {}
            pos = start_x + i * step
            self._move_one_axis('COARSE_X', pos)
            for _ in range(sample_number_in_one_position):
                if not self.running:
                    self.mqtt_receive_worker.stop()
                    return {}, {}
                self.event.wait(0.02)
                ifm = self.mqtt_receive_worker.get_interferometer_data()
                enc = self.mqtt_receive_worker.get_coarse_stage_pos()
                x_rows.append([ifm[5], ifm[6], ifm[8], ifm[9], pos, 0,
                               ifm[0], ifm[1], enc[0], enc[1]])
            self.append_to_text_edit(f"x scan point {i + 1}/{n_x} at {pos:.3f} mm done")
        pd.DataFrame(x_rows, columns=save_data_column_name).to_csv(x_file_save_name, index=False)
        self.append_to_text_edit(f"x scan data saved: {x_file_save_name}")

        # Y向扫描：Y轴按步长逐点移动，每点采集 sample_number_in_one_position 次
        y_rows = []
        n_y = int(round((end_y - start_y) / step)) + 1
        for i in range(n_y):
            if not self.running:
                self.mqtt_receive_worker.stop()
                return {}, {}
            pos = start_y + i * step
            self._move_one_axis('COARSE_Y', pos)
            for _ in range(sample_number_in_one_position):
                if not self.running:
                    self.mqtt_receive_worker.stop()
                    return {}, {}
                self.event.wait(0.02)
                ifm = self.mqtt_receive_worker.get_interferometer_data()
                enc = self.mqtt_receive_worker.get_coarse_stage_pos()
                y_rows.append([ifm[5], ifm[6], ifm[8], ifm[9], 0, pos,
                               ifm[0], ifm[1], enc[0], enc[1]])
            self.append_to_text_edit(f"y scan point {i + 1}/{n_y} at {pos:.3f} mm done")
        pd.DataFrame(y_rows, columns=save_data_column_name).to_csv(y_file_save_name, index=False)
        self.append_to_text_edit(f"y scan data saved: {y_file_save_name}")

        self.mqtt_receive_worker.stop()

        # 面型误差计算与绘图
        save_dir = os.path.dirname(x_file_save_name)
        df_x = pd.read_csv(x_file_save_name)
        x_mirror_TXY, x_mirror_Rz = calculate_x_mirror(
            df_x[['hx1', 'hx2', 'hy1', 'hy2']].to_numpy(),
            save_dir, sample_number_in_one_position, timestamp_str)
        df_y = pd.read_csv(y_file_save_name)
        y_mirror_TXY, y_mirror_Rz = calculate_y_mirror(
            df_y[['hx1', 'hx2', 'hy1', 'hy2']].to_numpy(),
            save_dir, sample_number_in_one_position, timestamp_str)

        self.show_mirror_map(x_mirror_TXY, x_mirror_Rz, y_mirror_TXY, y_mirror_Rz)
        self.append_to_text_edit(
            f"test finished, mirror map saved: XMirrorTXY/XMirrorRz/YMirrorTXY/YMirrorRz_{timestamp_str}.csv")

        return {}, {}

    def _move_one_axis(self, axis_key, pos):
        """控制粗动台单轴绝对运动并等待到位"""
        self.paho_client.publish("GUI_DR_ACS_Motion",
                                 mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                     pos=pos,
                                     stage_name=LST_AXIS_NAME[axis_key]["name"],
                                     stage_index=LST_AXIS_NAME[axis_key]["index"],
                                     axis_name=LST_AXIS_NAME[axis_key]["axisName"]
                                 ),
                                 0)
        self.mqtt_receive_worker.wait_stage_stop()

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    def append_to_text_edit(self,message):
        self.ui.textEdit.append(message)

    def show_mirror_map(self,
                        x_mirror_TXY = None, x_mirror_Rz = None,
                        y_mirror_TXY = None, y_mirror_Rz = None):

        self.res_ax_x_mirror_TXY.clear()
        self.res_ax_x_mirror_Rz.clear()
        self.res_ax_y_mirror_TXY.clear()
        self.res_ax_y_mirror_Rz.clear()

        if x_mirror_TXY is not None:
            self.res_ax_x_mirror_TXY.plot(x_mirror_TXY)
        if x_mirror_Rz is not None:
            self.res_ax_x_mirror_Rz.plot(x_mirror_Rz)
        if y_mirror_TXY is not None:
            self.res_ax_y_mirror_TXY.plot(y_mirror_TXY)
        if y_mirror_Rz is not None:
            self.res_ax_y_mirror_Rz.plot(y_mirror_Rz)

        self.res_ax_x_mirror_TXY.set_title("X TXY")
        self.res_ax_x_mirror_Rz.set_title("X Rz")
        self.res_ax_y_mirror_TXY.set_title("Y TXY")
        self.res_ax_y_mirror_Rz.set_title("Y Rz")
        self.res_fig.tight_layout()

        self.res_canvas.draw()
    
        # 或者使用 grab() 方法（更简洁）
    def save_screenshot_grab(self):
        """使用 grab() 方法截图"""
        pixmap = self.grab()  # 截取整个窗口
        pixmap.save("screenshot.png", "PNG")

if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = interferometerMirrorMap()
    test.show()
    test.save_screenshot_grab()
    sys.exit(myapp.exec_())
