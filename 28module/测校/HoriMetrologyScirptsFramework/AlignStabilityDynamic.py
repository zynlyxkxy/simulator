# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic.py
Date: 2025/9/1
Description:
    标记对准重复性（动态）
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import AlignStabilityDynamic_ui as ui

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pandas as pd
import numpy as np
import threading
import random
import time
import json
import sys

# 继承ScriptWidget类
class AlignStabilityDynamic(ScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]
        self.config = MetrologyConfig()

        self.paho_client = PahoMqttClient()
        self.paho_client.connect(str(self.config.MQTTHOST), int(self.config.MQTTPORT))
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.event)

        self.ui.add_camera_dict(self.config.camera_dict)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            'camera_ID': int(self.ui.camera_ID.currentIndex()),
            'vision_template': self.ui.cb_vision_template.text(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': int(self.ui.wait_time.value()),
            'stage_move_type': int(self.ui.cb_stage_move_type.currentIndex()),
            'coarse_stage_start_pos_x': float(self.ui.coarse_stage_start_x.value()),
            'coarse_stage_start_pos_y': float(self.ui.coarse_stage_start_y.value()),
            'coarse_stage_get_pic_pos_x': float(self.ui.coarse_stage_get_pic_pos_x.value()),
            'coarse_stage_get_pic_pos_y': float(self.ui.coarse_stage_get_pic_pos_y.value()),
            'bonding_arm_start_pos_x': float(self.ui.bonding_arm_start_x.value()),
            'bonding_arm_get_pic_pos_x': float(self.ui.bonding_arm_get_pic_pos_x.value()),
            'flip_arm_start_pos_x': float(self.ui.filp_arm_start_x.value()),
            'flip_arm_get_pic_pos_x': float(self.ui.filp_arm_get_pic_pos_x.value())
        }
        return INPUT, {}

    # 重写该函数，其中param_list的内容和get_saved_param中自定义的list内容相同
    # 用于在脚本启动时读入保存的输入参数、机器常数
    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentIndex(param_dict.get('camera_ID', 0))
        self.ui.cb_vision_template.setText(param_dict.get('vision_template', ''))
        self.ui.repeat_times.setValue(param_dict.get('repeat_time', 1))
        self.ui.wait_time.setValue(param_dict.get('wait_time', 10))
        self.ui.cb_stage_move_type.setCurrentIndex(param_dict.get('stage_move_type', 0))
        self.ui.coarse_stage_start_x.setValue(param_dict.get('coarse_stage_start_pos_x', 0))
        self.ui.coarse_stage_start_y.setValue(param_dict.get('coarse_stage_start_pos_y', 0))
        self.ui.coarse_stage_get_pic_pos_x.setValue(param_dict.get('coarse_stage_get_pic_pos_x', 0))
        self.ui.coarse_stage_get_pic_pos_y.setValue(param_dict.get('coarse_stage_get_pic_pos_y', 0))
        self.ui.bonding_arm_start_x.setValue(param_dict.get('bonding_arm_start_pos_x', 0))
        self.ui.bonding_arm_get_pic_pos_x.setValue(param_dict.get('bonding_arm_get_pic_pos_x', 0))
        self.ui.filp_arm_start_x.setValue(param_dict.get('flip_arm_start_pos_x', 0))
        self.ui.filp_arm_get_pic_pos_x.setValue(param_dict.get('flip_arm_get_pic_pos_x', 0))

    def plot_realtime_data(self, data):
        pass

    def plot_log(self, log_msg):
        pass

    def initial_before_run(self):
        pass

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.paho_client.subscribe(self.TOPICS)
        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()
        camera_ID = self.config.camera_dict[self.ui.camera_ID.currentText()]
        vision_template = self.ui.cb_vision_template.text()
        repeat_time = int(self.ui.repeat_times.value())
        wait_time = float(self.ui.wait_time.value())
        stage_move_type = int(self.ui.cb_stage_move_type.currentIndex())
        coarse_stage_start_pos = [float(self.ui.coarse_stage_start_x.value()),
                                  float(self.ui.coarse_stage_start_y.value())]
        coarse_stage_get_pic_pos = [float(self.ui.coarse_stage_get_pic_pos_x.value()),
                                    float(self.ui.coarse_stage_get_pic_pos_y.value())]
        bonding_arm_start_pos = [float(self.ui.bonding_arm_start_x.value())]
        bonding_arm_get_pic_pos = [float(self.ui.bonding_arm_get_pic_pos_x.value())]
        flip_arm_start_pos = [float(self.ui.filp_arm_start_x.value())]
        flip_arm_get_pic_pos = [float(self.ui.filp_arm_get_pic_pos_x.value())]

        row_list, col_list = [], []
        self.append_to_text_edit(f"test start")

        if stage_move_type == 0:
            start_pos = coarse_stage_start_pos
            get_pic_pos = coarse_stage_get_pic_pos
        elif stage_move_type == 1:
            start_pos = bonding_arm_start_pos
            get_pic_pos = bonding_arm_get_pic_pos
        else:
            start_pos = flip_arm_start_pos
            get_pic_pos = flip_arm_get_pic_pos

        for i in range(repeat_time):
            if not self.running:
                break
            self.move_stage(stage_move_type, start_pos)
            self.event.wait(wait_time / 1000.0)
            self.move_stage(stage_move_type, get_pic_pos)
            # 拍照
            row, col = self.take_photo(camera_ID, vision_template)
            row_list.append(row)
            col_list.append(col)
            self.append_to_text_edit(f"detect result: row: {row}, col: {col}")
            self.append_to_text_edit(f"cycle time: {i + 1} / {repeat_time}")

        # save data
        saved_columns_name = ['index', 'col', 'row']
        save_file_name = f"align_stability_dynamic_{time.strftime('%m%d_%H_%M_%S')}.csv"
        save_data = np.array([[i for i in range(len(row_list))], col_list, row_list]).T
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(
            self.config.DATA_SAVE_ROOT + save_file_name, index=False)

        # display result
        self.res_ax_row.plot([i for i in range(len(row_list))], row_list)
        self.res_ax_row.set_title(f"row: {np.std(row_list) * 3 * 0.00274 / 20 * 1e6:.3f} nm")
        self.res_ax_row.set_ylabel("pixel")
        self.res_ax_col.plot([i for i in range(len(col_list))], col_list)
        self.res_ax_col.set_title(f"col: {np.std(col_list) * 3 * 0.00274 / 20 * 1e6:.3f} nm")
        self.res_ax_col.set_ylabel("pixel")
        self.res_canvas.draw()
        self.paho_client.unsubscribe(self.TOPICS)

        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.event.set()
        self.running = False
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()

    def move_stage(self, move_type: int, move_pos: list):
        if move_type == 0:
            current_axis = 'COARSE_X'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=move_pos[0],
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            current_axis = 'COARSE_Y'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=move_pos[1],
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
        elif move_type == 1:
            current_axis = 'BondArm_YN'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=move_pos[0],
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()
        elif move_type == 2:
            current_axis = 'FlipStage_RY'
            self.paho_client.publish("GUI_DR_ACS_Motion",
                                     mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                                         pos=move_pos[0],
                                         stage_name=LST_AXIS_NAME[current_axis]["name"],
                                         stage_index=LST_AXIS_NAME[current_axis]["index"],
                                         axis_name=LST_AXIS_NAME[current_axis]["axisName"]
                                     ),
                                     0)
            self.mqtt_receive_worker.wait_stage_stop()

        else:
            pass

    def take_photo(self, camera_id, template_name):
        self.mqtt_receive_worker.reset_image_process_status()
        self.paho_client.publish("VSC_Com",
                                 mcCamera.VisionCmdJson(id=camera_id, template=template_name, exposure=0),
                                 0)
        arr_row, arr_col = self.mqtt_receive_worker.wait_image_process()
        return arr_row, arr_col

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    def draw_res(self, row_res_list, col_res_list):
        self.res_fig.clear()
        self.res_ax_row.plot()
        self.res_fig.tight_layout()

    def save_screenshot_grab(self):
        """使用 grab() 方法截图"""
        pixmap = self.grab()  # 截取整个窗口
        pixmap.save("screenshot.png", "PNG")


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = AlignStabilityDynamic()
    # test.MqttClient.loop_start()
    test.set_saved_param({}, {})
    test.show()
    # test.begin_()
    test.save_screenshot_grab()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
