import importlib
import sys
import paho.mqtt.client as mqtt
import json
from ScriptWidget import ScriptWidget
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, QProcess
from PyQt5.QtWidgets import QApplication

MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883

OFFSET = [610, 120]
SIZE = [1200, 800]

if len(sys.argv) == 4:
    TOPIC = sys.argv[1] + "_Script"
    TOPIC_GUI = sys.argv[2] + "_Metrology"
    PID = int(sys.argv[3])
else:
    TOPIC = "CTL_TL_MET_Script"
    TOPIC_GUI = "GUI_TL_MET_Metrology"
    PID = 0

COMMAND = "tasklist /NH /FO CSV /FI \"PID eq " + str(PID) + "\""


class ScriptManager(QObject):
    sig_message = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.MqttClient = mqtt.Client()
        self.MqttClient.connect(MQTT_HOST, MQTT_PORT)
        self.MqttClient.subscribe(TOPIC_GUI)
        self.MqttClient.on_message = self.on_message
        self.dic_Script = {}
        self.sig_message.connect(self.on_deal_message)
        self.myapp = QApplication([])
        self.myapp.setQuitOnLastWindowClosed(False)
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_cpp_process)
        self.timer.start(1000)
        self.process = QProcess()

    def check_cpp_process(self):
        self.process.start(COMMAND)
        self.process.waitForFinished()
        if not self.process.readAllStandardOutput().contains(str(PID).encode()):
            print("C++ process has been terminated.")
            self.myapp.quit()

    def on_message(self, client, userdata, msg):
        self.sig_message.emit(msg)

    def on_deal_message(self, msg):
        print(msg.topic + str(msg.payload))
        data = json.loads(msg.payload)
        cmd = data['task']
        context = data['context']
        uuid = data['uuid']
        if cmd == 'CMD_End':
            for Script in self.dic_Script.values():
                Script.close()
            json_end = {
                'context': "",
                'result': True,
                'errMsg': None,
                'task': 'CMD_End',
                'uuid': uuid
            }
            data = json.dumps(json_end)
            self.MqttClient.publish(TOPIC, data, 0)
            self.myapp.quit()
        elif cmd == 'CMD_Hello':
            for Script in self.dic_Script.values():
                Script.close()
            self.dic_Script.clear()
            json_hello = {
                'context': "",
                'result': True,
                'errMsg': None,
                'task': 'CMD_Hello',
                'uuid': uuid
            }
            data = json.dumps(json_hello)
            self.MqttClient.publish(TOPIC, data, 0)
        elif cmd == 'CMD_AddScript':
            tag = 1
            try:
                module = importlib.import_module(context)
                klass = getattr(module, context)
                if issubclass(klass, ScriptWidget):
                    self.dic_Script[uuid] = klass()
                    self.dic_Script[uuid].initial_(self.MqttClient, uuid, TOPIC)
                else:
                    tag = 0
            except Exception as err:
                tag = str(err)
            json_add_script = {
                'context': str(tag),
                'result': True,
                'errMsg': None,
                'task': 'CMD_AddScript',
                'uuid': uuid
            }
            data = json.dumps(json_add_script)
            self.MqttClient.publish(TOPIC, data, 0)
        elif cmd == 'CMD_DeleteScript':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].close()
                self.dic_Script.pop(uuid)
        elif cmd == 'CMD_SetParam':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].set_param_(context)
        elif cmd == 'CMD_GetParam':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].get_param_(True)
        elif cmd == 'CMD_Run':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].begin_()
        elif cmd == 'CMD_SetHidden':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].setHidden(json.loads(context) == 1)
        elif cmd == 'CMD_SetBlur':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].set_blur_(json.loads(context) == 1)
        elif cmd == 'CMD_SetEnabled':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].setEnabled(json.loads(context) == 1)
        elif cmd == 'CMD_Display':
            if uuid in self.dic_Script:
                pos = json.loads(context)
                x = int(pos['x'])
                y = int(pos['y'])
                self.dic_Script[uuid].display_(x, y, SIZE[0], SIZE[1])
        elif cmd == 'CMD_Break':
            if uuid in self.dic_Script:
                self.dic_Script[uuid].get_interrupt_()

    def run(self):
        self.MqttClient.loop_start()
        try:
            self.myapp.exec()
        except Exception as e:
            print(e)
        self.MqttClient.loop_stop()
        self.MqttClient.disconnect()


if __name__ == '__main__':
    script_manager = ScriptManager()
    script_manager.run()
    print("End Python Script Manager")
    sys.exit(1)
