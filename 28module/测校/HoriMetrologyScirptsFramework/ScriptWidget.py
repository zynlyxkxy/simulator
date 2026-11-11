import json
import threading
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal
from abc import abstractmethod
import os
import inspect
import copy
from datetime import datetime


class ScriptWidget(QWidget):
    __sig_realtime = pyqtSignal(object)
    __sig_log = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.__managerClient = mqtt.Client()
        self.__bIsBlur = False
        self.__bIsRunning = False
        self.__strUuid = ""
        self.__nProgress = 0
        self.__MC_type = 0
        self.__recorder = None
        self.ui = 0
        current_class_file = inspect.getfile(self.__class__)
        file_name = os.path.basename(current_class_file)
        self.__strName = os.path.splitext(file_name)[0]
        self.__strTopic = ""
        self.__sig_realtime.connect(self.__plot_realtime_data)
        self.__sig_log.connect(self.__log)

    def initial_(self, client, uuid, topic):
        self.__managerClient = client
        self.__strUuid = uuid
        self.__strTopic = topic

    def set_recorder_(self, recorder):
        self.__recorder = recorder

    def set_blur_(self, able):
        if not able:
            # self.ui.frame.setGraphicsEffect(None)
            self.__bIsBlur = False
        else:
            if not self.__bIsBlur:
                # blur_effect = QGraphicsBlurEffect()
                # blur_effect.setBlurRadius(5)
                # blur_effect.setBlurHints(QGraphicsBlurEffect.PerformanceHint)
                # self.ui.frame.setGraphicsEffect(blur_effect)
                self.__bIsBlur = True

    def display_(self, x, y, w, h):
        try:
            self.setFixedSize(w, h)
            self.move(-w, -h)
            # self.setAttribute(Qt.WA_OpaquePaintEvent)
            self.setWindowFlags(
                self.windowFlags() | Qt.FramelessWindowHint | Qt.WA_DeleteOnClose)
            self.show()
            win_id = int(self.winId())
            json_input = {
                'context': str(win_id),
                'result': True,
                'errMsg': None,
                'task': 'CMD_Display',
                'uuid': self.__strUuid
            }
            data = json.dumps(json_input)
            self.__managerClient.publish(self.__strTopic, data, 0)
        except Exception as e:
            self.exception(str(e))

    def begin_(self):
        try:
            self.initial_before_run()
        except Exception as e:
            self.exception(e)
        self.set_enabled_(True)
        t1 = threading.Thread(target=self.__worker, name="线程1")
        t1.setDaemon(True)
        t1.start()

    def get_param_(self, publish):
        try:
            input_config, input_mc = self.get_saved_param()
            if isinstance(input_mc, dict):
                input_mc = json.dumps(input_mc)

            context = {
                'm_strScriptName': self.__strName,
                'm_byteInput': json.dumps(input_config),
                'm_nUseMC': self.__MC_type,
                'm_byteMC': input_mc,
            }
            json_input = {
                'context': json.dumps(context),
                'result': True,
                'errMsg': None,
                'task': 'CMD_GetParam',
                'uuid': self.__strUuid
            }
            data = json.dumps(json_input)
            if publish:
                self.__managerClient.publish(self.__strTopic, data, 0)
            return context
        except Exception as e:
            self.exception(str(e))
            return -1

    def set_param_(self, data):
        try:
            context = json.loads(data)
            if self.__strName == context['m_strScriptName']:
                input_config = json.loads(context['m_byteInput'])
                self.__MC_type = context['m_nUseMC']
                input_mc = json.loads(context['m_byteMC'])
                self.set_saved_param(input_config, input_mc)
        except Exception as e:
            self.exception(str(e))

    def get_interrupt_(self):
        try:
            self.interrupt()
        except Exception as e:
            self.exception(str(e))
            return
        self.__bIsRunning = False
        context = {
            'm_strScriptName': self.__strName,
            'm_nProgress': self.__nProgress
        }
        json_interrupt = {
            'context': json.dumps(context),
            'result': True,
            'errMsg': None,
            'task': 'CMD_Break',
            'uuid': self.__strUuid
        }
        data = json.dumps(json_interrupt)
        self.__managerClient.publish(self.__strTopic, data, 0)

    def set_enabled_(self, able):
        self.setEnabled(able)

    def __log(self, log_msg):
        if isinstance(log_msg, str):
            try:
                cur_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                log_msg = f"[{cur_time}]{log_msg}"
                self.plot_log(log_msg)
                json_log = {
                    'context': log_msg,
                    'result': True,
                    'errMsg': None,
                    'task': 'CMD_Log',
                    'uuid': self.__strUuid
                }
                data = json.dumps(json_log)
                self.__managerClient.publish(self.__strTopic, data, 0)
            except Exception as e:
                self.exception(str(e))

    def __plot_realtime_data(self, data):
        try:
            self.plot_realtime_data(data)
        except Exception as e:
            self.exception(str(e))

    def __worker(self):
        context = self.get_param_(False)
        if context == -1:
            return
        try:
            json_run = {
                'context': json.dumps(context),
                'result': True,
                'errMsg': None,
                'task': 'CMD_Run',
                'uuid': self.__strUuid
            }
            data = json.dumps(json_run)
            self.__managerClient.publish(self.__strTopic, data, 0)
            self.__nProgress = 0
            self.__bIsRunning = True

            input_config, input_mc = self.get_saved_param()
            running_mc = copy.deepcopy(input_mc)
            if self.__MC_type == 0:
                self.__recorder.update_mc(running_mc)
                # self.set_saved_param(input_config, running_mc)

            output_config, output_mc = self.run(copy.deepcopy(input_config), copy.deepcopy(running_mc))

            if self.__bIsRunning:
                context = {
                    'm_strScriptName': self.__strName,
                    'm_byteInput': json.dumps(input_config),
                    'm_byteInputMC': json.dumps(running_mc),
                    'm_bResult': True,
                    'm_byteOutput': json.dumps(output_config),
                    'm_byteMC': json.dumps(output_mc),
                }
                json_output = {
                    'context': json.dumps(context),
                    'result': True,
                    'errMsg': None,
                    'task': 'CMD_Finish',
                    'uuid': self.__strUuid
                }
                data = json.dumps(json_output)
                self.__managerClient.publish(self.__strTopic, data, 0)
        except Exception as e:
            self.exception(str(e))
            return

    # 调用以设置当前进度
    def set_progress(self, progress):
        if not isinstance(progress, int):
            return
        self.__nProgress = progress
        context = {
            'm_strScriptName': self.__strName,
            'm_nProgress': progress
        }
        json_output = {
            'context': json.dumps(context),
            'result': True,
            'errMsg': None,
            'task': 'CMD_Running',
            'uuid': self.__strUuid
        }
        data = json.dumps(json_output)
        self.__managerClient.publish(self.__strTopic, data, 0)

    # 调用以记录异常情况
    def exception(self, err_msg):
        self.__bIsRunning = False
        context = {
            'm_strScriptName': self.__strName,
            'm_strError': err_msg
        }
        json_error = {
            'context': json.dumps(context),
            'result': True,
            'errMsg': None,
            'task': 'CMD_Error',
            'uuid': self.__strUuid
        }
        data = json.dumps(json_error)
        self.__managerClient.publish(self.__strTopic, data, 0)

    # 调用以记录中间数据
    def set_realtime_data(self, realtime_data):
        self.__sig_realtime.emit(realtime_data)

    # 重写该函数以显示中间参数，输入为中间参数
    @abstractmethod
    def plot_realtime_data(self, realtime_data):
        pass

    # 调用以记录log信息，输入为字符串
    def log(self, log_msg):
        self.__sig_log.emit(log_msg)

    # 重写该函数以显示log信息，输入为字符串
    def plot_log(self, log_msg):
        pass

    # 重写该函数以获得需要保存的输入参数和使用的机器常数
    @abstractmethod
    def get_saved_param(self):
        return {}, {}

    # 重写该函数以设置保存的输入参数和机器常数
    @abstractmethod
    def set_saved_param(self, dict_input, dict_mc):
        pass

    # 重写该函数以实现每次运行之前需要完成的操作
    @abstractmethod
    def initial_before_run(self):
        pass

    # 重写为具体的测试流程
    @abstractmethod
    def run(self, dict_input, dict_mc):
        return {}, {}

    # 重写为中断时的操作
    @abstractmethod
    def interrupt(self):
        pass
