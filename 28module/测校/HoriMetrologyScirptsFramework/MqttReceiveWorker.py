# -*- coding: utf-8 -*-
# @Time: 2025/7/1 11:40
# @Author: Jianming.Liu
import time
import json
import threading
import base64
import cv2
import numpy as np
import copy
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME


class MqttReceiveWorker:

    def __init__(self, mqtt_client, run_thread_event):
        self.paho_client = mqtt_client
        self.run_thread_event = run_thread_event
        self.running = False
        self.lock = threading.Lock()

        self._stage_moving = False
        self._plc_stage_moving = False
        self._flip_pos = [0] * 4  # [y, z, rz, ry]
        self._coarse_stage = [0] * 2  # [x, y]
        self._vertical_micro_stage = [0] * 3  # [rx, ry, z]
        self._horizon_micro_stage = [0] * 3  # [x, y, rz]
        self._interferometer_data = [0] * 11  # [x, y, rx, ry, rz, hx1, hx2, hx3, hy1, hy2, hy3]

        self._flip_vacuum_value = 0
        self._bonding_vacuum_value = 0

        self.receive_image_res = False
        self._image_res = [0, 0]
        self._image = None
        self.coarse_alignment_calculate_finish = False

        self._receive_fls_res = False
        self._fls_value = 0

        self._bonding_head_capacitor_value = [[0, 0]] * 4


    def __worker(self):
        """
        mqtt消息监听
        :return: None
        """
        while True:
            if not self.running:
                break
            if not self.paho_client.mqttQueue.empty():
                msg = self.paho_client.mqttQueue.get()
                self.parse_data(msg)
            self.run_thread_event.wait(0.01)
        print("mqtt thread exit")

    def start(self):
        self.running = True
        self.t1 = threading.Thread(target=self.__worker, name="工作线程")
        self.t1.start()
        self.run_thread_event.wait(1)

    def stop(self):
        self.running = False

    def parse_data(self, msg):
        if msg:
            recv_raw = json.loads(msg.payload.decode('utf-8'))
            task = recv_raw['task']

            if msg and msg.topic == 'CTL_DR_PLC_Motion':
                # print(msg)
                recv_raw = json.loads(msg.payload.decode('utf-8'))
                task = recv_raw['task']
                res = recv_raw
                if res and task == 'CMD_Motion_RunTimeInfo':
                    plc_stage_info = json.loads(recv_raw['context'])
                    stage_moving = False
                    with self.lock:
                        for stage_index, stage_info in plc_stage_info['dic_Stage'].items():
                            if not stage_info['inPos'] or stage_info['moving']:
                                stage_moving = True
                            if stage_index == '2005':
                                # Rx
                                self._vertical_micro_stage[0] = stage_info['lst_Axis'][0]['coord']['value']
                                # Ry
                                self._vertical_micro_stage[1] = stage_info['lst_Axis'][1]['coord']['value']
                                # z
                                self._vertical_micro_stage[2] = stage_info['lst_Axis'][2]['coord']['value']
                    self._plc_stage_moving = stage_moving
                    if self._plc_stage_moving:
                        print("plc stage info, is moving", self._plc_stage_moving, "pos: ", self._vertical_micro_stage)

            res = recv_raw
            # stage info
            if res and task == 'CMD_Motion_RunTimeInfo':
                acs_info = json.loads(recv_raw['context'])
                stage_moving = False
                with self.lock:
                    for stage_index, stage_info in acs_info['dic_Stage'].items():
                        if not stage_info['inPos'] or stage_info['moving']:
                            stage_moving = True

                        if stage_index == '1003':
                            # 翻转模块 CoarseMoveStage 整块反馈，lst_Axis 顺序 [Y, Z, Rz, Ry]
                            for index, axis_info in enumerate(stage_info['lst_Axis'][:4]):
                                self._flip_pos[index] = axis_info['coord']['value']

                        if stage_index == str(LST_AXIS_NAME['FlipStage_Y']['index']):
                            # y
                            self._flip_pos[0] = stage_info['lst_Axis'][0]['coord']['value']
                        if stage_index == str(LST_AXIS_NAME['FlipStage_Z']['index']):
                            # z
                            self._flip_pos[1] = stage_info['lst_Axis'][0]['coord']['value']
                        if stage_index == str(LST_AXIS_NAME['FlipStage_RZ']['index']):
                            # rz
                            self._flip_pos[2] = stage_info['lst_Axis'][0]['coord']['value']
                        if stage_index == str(LST_AXIS_NAME['FlipStage_RY']['index']):
                            # ry
                            self._flip_pos[3] = stage_info['lst_Axis'][0]['coord']['value']
                        if stage_index == LST_AXIS_NAME['COARSE_Y']:
                            # coarse stage x
                            self._coarse_stage[1] = stage_info['lst_Axis'][0]['coord']['value']
                        if stage_index == LST_AXIS_NAME['COARSE_X']:
                            # coarse stage y
                            self._coarse_stage[0] = stage_info['lst_Axis'][0]['coord']['value']

                self._stage_moving = stage_moving
                if self._stage_moving:
                    print(time.strftime("%H:%M:%S: stage , coarse stage pos:"), self._coarse_stage)

            # io
            if res and task == 'CMD_IO_AiRuntimeInfo':
                ai_group_info = json.loads(json.loads(msg.payload)['context'])
                self._flip_vacuum_value = ai_group_info['dic_Ai']['2027']['value']
                self._bonding_vacuum_value = ai_group_info['dic_Ai']['2027']['value']

                with self.lock:
                    self._horizon_micro_stage[0] = ai_group_info['dic_Ai']['2051']['value']
                    self._horizon_micro_stage[1] = ai_group_info['dic_Ai']['2052']['value']
                    self._horizon_micro_stage[2] = ai_group_info['dic_Ai']['2053']['value']

                with self.lock:
                    self._interferometer_data[0] = ai_group_info['dic_Ai']['2054']['value']
                    self._interferometer_data[1] = ai_group_info['dic_Ai']['2055']['value']
                    self._interferometer_data[2] = ai_group_info['dic_Ai']['2056']['value']
                    self._interferometer_data[3] = ai_group_info['dic_Ai']['2057']['value']
                    self._interferometer_data[4] = ai_group_info['dic_Ai']['2058']['value']
                    self._interferometer_data[5] = ai_group_info['dic_Ai']['2059']['value']
                    self._interferometer_data[6] = ai_group_info['dic_Ai']['2060']['value']
                    self._interferometer_data[7] = ai_group_info['dic_Ai']['2061']['value']
                    self._interferometer_data[8] = ai_group_info['dic_Ai']['2062']['value']
                    self._interferometer_data[9] = ai_group_info['dic_Ai']['2063']['value']
                    self._interferometer_data[10] = ai_group_info['dic_Ai']['2064']['value']

            # vision
            if msg.topic == "VSS_Com":
                result = json.loads(recv_raw["context"])
                # print(result.get('content'))
                if result.get('image') is not None and result.get('image') != "":
                    self._image = self.base64_to_cv2_image(result.get('image'))
                    self.receive_image_res = True
                if result.get('content') is not None:
                    result = json.loads(result["content"])
                    if len(result["1"]["lstRow"]) > 0 and len(result["1"]["lstCol"]) > 0:
                        self._image_res[0] = result["1"]["lstRow"][0]
                        self._image_res[1] = result["1"]["lstCol"][0]
                        self.receive_image_res = True

            # coarse alignment alg
            if res and task == 'CMD_GetCoarseAlignResult':
                alignment_res = json.loads(recv_raw['context'])
                res_array = alignment_res['result']
                coarse_alignment_calculate_finish = True

        if msg and msg.topic == 'CTL_DR_CF_FLS':
            recv_raw = json.loads(msg.payload.decode('utf-8'))
            task = recv_raw['task']
            res = recv_raw
            if res and task == 'CMD_CF_MeasureOneResult':
                fls_data = json.loads(recv_raw['context'])
                self._receive_fls_res = True
                self._fls_value = np.average(fls_data['data'])
                # print(f'fls detect value: {self._fls_value}')

    def wait_stage_stop(self):
        self.run_thread_event.wait(0.1)
        while True and self.running:
            self.run_thread_event.wait(0.1)
            if not self._stage_moving:
                break

    def wait_vertical_micro_stage_stop(self, index=-1, target_pos=0, threshold=1e-4):
        if index == -1 or index > 3:
            return
        time.sleep(1)
        while True and self.running:
            time.sleep(0.5)
            if abs(target_pos - self._vertical_micro_stage[index]) < threshold:
                break

    def wait_vertical_micro_stage_in_position(self, target_pos, threshold=1e-3):
        if target_pos is None or len(target_pos) < 3:
            return
        while True and self.running:
            time.sleep(0.3)
            if (abs(target_pos[0] - self._vertical_micro_stage[0]) < threshold and
                    abs(target_pos[1] - self._vertical_micro_stage[1]) < threshold and
                    abs(target_pos[2] - self._vertical_micro_stage[2]) < threshold and
                    not self._plc_stage_moving):
                break

    def wait_horizontal_micro_stage_stop(self, target_pos, threshold=1e-3):
        if target_pos is None or len(target_pos) < 3:
            return
        time.sleep(1)
        while True and self.running:
            time.sleep(0.3)
            if (abs(target_pos[0] - self._horizon_micro_stage[0]) < threshold and
                    abs(target_pos[1] - self._horizon_micro_stage[1]) < threshold and
                    abs(target_pos[2] - self._horizon_micro_stage[2]) < threshold):
                break

    def get_interferometer_data(self):
        # [x, y, rx, ry, rz, hx1, hx2, hx3, hy1, hy2, hy3]
        with self.lock:
            return self._interferometer_data

    def get_coarse_stage_pos(self):
        with self.lock:
            return self._coarse_stage

    def get_horizon_micro_stage_pos(self):
        with self.lock:
            return self._horizon_micro_stage

    def get_vertical_micro_stage_pos(self):
        with self.lock:
            return self._vertical_micro_stage

    def get_flip_pos(self):
        with self.lock:
            return self._flip_pos

    def get_bonding_head_capacitor_value(self, bonding_head_index):
        if  0 <= bonding_head_index < 4:
            with self.lock:
                return self._bonding_head_capacitor_value[bonding_head_index]
        else:
            return [0, 0]

    def check_flip_vacuum(self, threshold):
        while True:
            self.run_thread_event.wait(2)
            if self._flip_vacuum_value > threshold:
                return

    def check_bonding_vacuum(self, threshold):
        while True:
            self.run_thread_event.wait(2)
            if self._bonding_vacuum_value > threshold:
                return

    def reset_image_process_status(self):
        self.receive_image_res = False

    def wait_get_raw_image(self, timeout=10):
        """等待硬触发拍照的原始图像返回（VSS_Com 消息携带 image 字段）"""
        b_time = time.time()
        self.run_thread_event.wait(1)
        while True:
            self.run_thread_event.wait(0.5)
            if self.receive_image_res and self._image is not None:
                break
            if time.time() - b_time > timeout or not self.running:
                print("get raw image timeout")
                return None
        return self._image

    def wait_image_process(self, timeout=10):
        b_time = time.time()
        self.run_thread_event.wait(1)
        while True :
            self.run_thread_event.wait(0.5)
            if self.receive_image_res:
                break
            if time.time() - b_time > timeout or not self.running:
                print("image process timeout")
                return [0, 0]

        return self._image_res

    def reset_fls_process_status(self):
        self._receive_fls_res = False

    def wait_fls_process(self, timeout=10):
        b_time = time.time()
        while True:
            time.sleep(0.5)
            if self._receive_fls_res:
                break
            if time.time() - b_time > timeout:
                print("fls process timeout")
                return 0

        return self._fls_value

    @staticmethod
    def base64_to_cv2_image(base64_string):
        """
        将Base64字符串转换为OpenCV图像
        """
        # 解码Base64字符串
        image_data = base64.b64decode(base64_string)

        # 将字节数据转换为numpy数组
        nparr = np.frombuffer(image_data, np.uint8)

        # 使用OpenCV解码图像
        img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)

        return img

    def get_raw_image(self):
        return self._image

    def coarse_alignment_calculate(self):
        pass
