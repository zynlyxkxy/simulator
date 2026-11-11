# -*- coding: utf-8 -*-
"""
Filename: EgaPrecisionTest.py
Date: 2025/9/1
Description:
 
"""
from metrology_config import MetrologyConfig
from ScriptWidget import ScriptWidget
from TestScriptWidget import TestScriptWidget
from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
import EgaPrecisionTest_ui as ui
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
class EgaPrecisionTest(TestScriptWidget):
    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.thread_event = threading.Event()
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
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client, self.thread_event)

        self.ui.add_camera_dict(self.config.camera_dict)

        self.res_fig = Figure()
        self.res_ax_1 = self.res_fig.add_subplot(211)
        self.res_ax_2 = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        res_fig_layout = QVBoxLayout()
        res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_widget_1.setLayout(res_fig_layout)

        self.res_fig_2 = Figure()
        self.res_ax_3 = self.res_fig_2.add_subplot(211)
        self.res_ax_4 = self.res_fig_2.add_subplot(212)
        self.res_fig_2.tight_layout()
        self.res_canvas_2 = FigureCanvas(self.res_fig_2)
        res_fig_layout = QVBoxLayout()
        res_fig_layout.addWidget(self.res_canvas_2)
        self.ui.res_widget_2.setLayout(res_fig_layout)

        self.draw_res_data()

        self.ega_param = [-49, 188, 1, 1, 0, 0]
        try:
            with open(r"D:\App_Data\ProcessData\egaResult.json", 'r', encoding='utf-8') as file:
                ega_data = json.load(file)
                self.ega_param = ega_data['egaSixParams']
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            # 如果文件不存在或解析失败，使用默认值
            pass

    # 重写该函数，将所有需要保存的输入参数整理为list返回，输入的机器常数以json文件格式返回
    def get_saved_param(self):
        INPUT = {
            "wafer_radius": float(self.ui.wafer_radius.value()),
            "chip_width": float(self.ui.chip_width.value()),
            "chip_height": float(self.ui.chip_height.value()),
            "chip_spacing": float(self.ui.chip_spacing.value()),
            "offset_x": float(self.ui.offset_x.value()),
            "offset_y": float(self.ui.offset_y.value()),
            "repeat_time": int(self.ui.repeat_time.value()),
            "camera_id": int(self.ui.camera_ID.currentIndex()),
            "vision_template": self.ui.cb_vision_template.text()
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
        self.ui.wafer_radius.setValue(param_dict.get("wafer_radius", 0))
        self.ui.chip_width.setValue(param_dict.get("chip_width", 0))
        self.ui.chip_height.setValue(param_dict.get("chip_height", 0))
        self.ui.chip_spacing.setValue(param_dict.get("chip_spacing", 0))
        self.ui.offset_x.setValue(param_dict.get("offset_x", 0))
        self.ui.offset_y.setValue(param_dict.get("offset_y", 0))
        self.ui.camera_ID.setCurrentIndex(param_dict.get("camera_id", 0))
        self.ui.repeat_time.setValue(param_dict.get("repeat_time", 0))
        self.ui.cb_vision_template.setText(param_dict.get("vision_template", "test"))

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

    # 重写该函数，具体的测试步骤
    def run(self, dict_input, dict_mc):
        self.running = True
        self.thread_event.clear()
        self.paho_client.subscribe(self.TOPICS)
        self.mqtt_receive_worker.start()
        self.thread_event.wait(1)
        dict_input, _ = self.get_saved_param()
        # 参数设置
        wafer_radius = dict_input["wafer_radius"]
        chip_width = dict_input["chip_width"]
        chip_height = dict_input["chip_height"]
        spacing = dict_input["chip_spacing"]
        offset_x = dict_input["offset_x"]
        offset_y = dict_input["offset_y"]
        ega_params = np.array(self.ega_param)
        camera_id = dict_input["camera_id"]
        repeat_time = dict_input["repeat_time"]
        vision_template = dict_input["vision_template"]
        # 按界面选择的相机更新像素换算参数
        camera_setting = self.config.camera_setting[camera_id]
        self.pixel_width = camera_setting['image_center_x']
        self.pixel_height = camera_setting['image_center_y']
        self.pixel_size = camera_setting['pixel_size']
        self.camera_scale = camera_setting['magnification']

        cells_layout = self.create_chip_position(wafer_radius, chip_width, chip_height, spacing)
        marks_layout = self.compute_mark_positions(cells_layout, offset_x, offset_y)
        select_position_index = random.sample([i for i in range(marks_layout.shape[0])], 30)
        # select_point = select_fitting_points(marks_layout, 30, wafer_radius)
        select_point = marks_layout[select_position_index]
        real_position = self.nominal_to_real(select_point, ega_params)

        res = np.zeros((repeat_time, 4))

        for cycle_time in range(repeat_time):
            if not self.running:
                return {}, {}
            row_list, col_list = [], []
            detect_pos_list = []
            for i, cur_position in enumerate(real_position):
                if not self.running:
                    return {}, {}
                # 到标记位置
                self.append_to_text_edit(f"{'-' * 10}> stage to {cur_position[0]}, {cur_position[1]}")
                actuator.move_coarse_stage(cur_position,
                                           self.paho_client, self.mqtt_receive_worker)
                self.append_to_text_edit(f"{'-' * 10}> stage move ({cur_position[0]}, {cur_position[1]}) end")
                if not self.running:
                    return {}, {}

                self.append_to_text_edit(f"{'-' * 10}> take_photo start")
                row, col = self.take_photo(camera_id, vision_template)
                self.append_to_text_edit(f"{'-' * 10}> take_photo end")
                if not self.running:
                    return {}, {}

                if row == 0 and col == 0:
                    self.append_to_text_edit(f"{'-' * 10}> mark detect failed at point {i + 1}, skipped")
                    detect_pos_list.append([np.nan, np.nan])
                    continue
                #
                # row = np.random.random(1)[0]
                # col = np.random.random(1)[0]

                row_list.append(row)
                col_list.append(col)

                image_offset_x = (col - self.pixel_width / 2.0) * self.pixel_size / (self.camera_scale * 1000)
                image_offset_y = -(row - self.pixel_height / 2.0) * self.pixel_size / (self.camera_scale * 1000)
                detect_position = [0, 0]
                detect_position[0] = cur_position[0] + image_offset_x
                detect_position[1] = cur_position[1] + image_offset_y
                detect_pos_list.append(detect_position)

                self.append_to_text_edit(
                    f"{'-' * 10}> detect position ({detect_position[0]}, {detect_position[1]})")
                self.append_to_text_edit(f"{'-' * 10}> cycle time: {i + 1} / {len(real_position)}")
                # self.thread_event.wait(1)

            detect_pos_list = np.array(detect_pos_list)
            ega_error = detect_pos_list - real_position
            mean_xy = np.nanmean(ega_error[:], axis=0)
            std3_xy = np.nanstd(ega_error[:], axis=0) * 3
            self.append_to_text_edit(
                f"cycle {cycle_time + 1}: mean = ({mean_xy[0]:.6f}, {mean_xy[1]:.6f}) mm, "
                f"3*sigma = ({std3_xy[0]:.6f}, {std3_xy[1]:.6f}) mm")
            res[cycle_time, 0:2] = mean_xy
            res[cycle_time, 2:4] = std3_xy
            self.draw_res_data(res[:cycle_time + 1])

        save_data_columns_name = ['mean x', 'mean y', '3*sigma x', '3*sigma y']
        save_path_root = self.config.DATA_SAVE_ROOT
        pd.DataFrame(res, columns=save_data_columns_name).to_csv(f'{save_path_root}/data/ega_error.csv', index=False)
        self.append_to_text_edit(f"test finished, data saved: {save_path_root}/data/ega_error.csv")
        self.paho_client.unsubscribe(self.TOPICS)
        self.mqtt_receive_worker.stop()
        return {}, {}

    # 重写该函数，打断时的操作
    def interrupt(self):
        self.thread_event.set()
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

    def draw_res_data(self, res_data=None):
        if res_data is not None and res_data.shape[1] >= 4:
            # Tab "x"：上方 X 向偏差均值，下方 X 向 3σ
            self.res_ax_1.clear()
            self.res_ax_2.clear()
            self.res_ax_1.plot(res_data[:, 0])
            self.res_ax_2.plot(res_data[:, 2])
            # Tab "y"：上方 Y 向偏差均值，下方 Y 向 3σ
            self.res_ax_3.clear()
            self.res_ax_4.clear()
            self.res_ax_3.plot(res_data[:, 1])
            self.res_ax_4.plot(res_data[:, 3])

        self.res_ax_1.set_title(f"mean")
        self.res_ax_1.set_xlabel("index")
        self.res_ax_1.set_ylabel("mean / mm")
        self.res_ax_2.set_title(f"3*sigma")
        self.res_ax_2.set_xlabel("index")
        self.res_ax_2.set_ylabel("3*sigma / mm")
        self.res_fig.tight_layout()

        self.res_ax_3.set_title(f"mean")
        self.res_ax_3.set_xlabel("index")
        self.res_ax_3.set_ylabel("mean / mm")
        self.res_ax_4.set_title(f"3*sigma")
        self.res_ax_4.set_xlabel("index")
        self.res_ax_4.set_ylabel("3*sigma / mm")
        self.res_fig_2.tight_layout()

        self.res_canvas.draw()
        self.res_canvas_2.draw()

    def append_to_text_edit(self, message):
        self.ui.textEdit.append(message)

    def save_screenshot_grab(self):
        """使用 grab() 方法截图"""
        pixmap = self.grab()  # 截取整个窗口
        pixmap.save("screenshot.png", "PNG")


if __name__ == '__main__':
    myapp = QApplication(sys.argv)
    test = EgaPrecisionTest()
    # test.MqttClient.loop_start()
    test.show()
    test.set_saved_param({}, {})
    # test.begin_()
    test.save_screenshot_grab()
    sys.exit(myapp.exec_())
    # test.MqttClient.loop_stop()
    # test.MqttClient.disconnect()
