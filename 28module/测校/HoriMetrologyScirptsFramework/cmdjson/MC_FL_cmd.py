import json
# from comm.mqtt_client import paho_client
import time


def CMD_RunTimeData_FlsMacroStage(task="CMD_RunTimeData_FlsMacroStage"):
    cmd = {
        "context": None,
        "result": False,
        "task": "CMD_RunTimeData_FlsMacroStage"
    }

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)

    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    # json_str = payloadon.dumps(cmd, indent=1)
    # bytes = json_str.encode('utf-8', 'ignore')
    # print(bytes)
    # print("move start")
    # print("move end")

    return json_str


def MCFL_CMD_Initialize(task="CMD_Initialize"):
    cmd = {
        "context": None,
        "result": False,
        "task": "CMD_Initialize"
    }

    # cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)

    json_str = json.dumps(cmd, indent=1)
    return json_str


def MCFL_CMD_SnapOnce(task="CMD_CF_MeasureOneChannel", sensor_id=0, collection_quantity=10):
    js_data = {
        "result": False,
        "task": task
    }

    payload = {
        "collectionQuantity": collection_quantity,
        "id": 0,
        "sensorId": sensor_id

    }
    js_data['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(js_data, indent=1)
    return json_str


def MCFL_CMD_SnapOnce_mask(task="CMD_SnapOnce"):
    js_data = {
        "context": "{\n    \"id\": 0,\n    \"param\": {\n        \"acqNum\": 100,\n        \"averageNum\": 1,\n        \"bufferNum\": 1,\n        \"bufferSize\": 10,\n        \"emEventType\": 1,\n        \"emTriggerLevel\": 0,\n        \"enableAltitude\": true,\n        \"enableCounter\": true,\n        \"enableTrigger\": false,\n        \"exposureTime\": 0.0985,\n        \"gain\": 2,\n        \"scanRate\": 1000,\n        \"sleeptime\": 50,\n        \"timeout\": 100\n    },\n    \"res\": {\n        \"count\": 0,\n        \"data\": [\n        ],\n        \"isContinous\": false,\n        \"judge\": false\n    }\n}\n",
        "result": False,
        "task": "CMD_SnapOnce"
    }
    json_str = json.dumps(js_data, indent=1)
    return json_str


def MCFL_CMD_SnapOnce_wafer(task="CMD_SnapOnce"):
    js_data = {
        "context": "{\n    \"id\": 1,\n    \"param\": {\n        \"acqNum\": 100,\n        \"averageNum\": 1,\n        \"bufferNum\": 1,\n        \"bufferSize\": 10,\n        \"emEventType\": 1,\n        \"emTriggerLevel\": 0,\n        \"enableAltitude\": true,\n        \"enableCounter\": true,\n        \"enableTrigger\": false,\n        \"exposureTime\": 0.0985,\n        \"gain\": 2,\n        \"scanRate\": 1000,\n        \"sleeptime\": 50,\n        \"timeout\": 100\n    },\n    \"res\": {\n        \"count\": 0,\n        \"data\": [\n        ],\n        \"isContinous\": false,\n        \"judge\": false\n    }\n}\n",
        "result": False,
        "task": "CMD_SnapOnce"
    }
    json_str = json.dumps(js_data, indent=1)
    return json_str


def MCFL_CMD_SnapOnce_vertical(task="CMD_SnapOnce"):

    js_data = {
        "context": "{\n    \"id\":0,\n    \"param\": {\n        \"acqNum\": 1,\n        \"averageNum\": 1,\n        \"bufferNum\": 1,\n        \"bufferSize\": 10,\n        \"emEventType\": 1,\n        \"emTriggerLevel\": 0,\n        \"enableAltitude\": true,\n        \"enableCounter\": true,\n        \"enableTrigger\": false,\n        \"exposureTime\": 0.17,\n        \"gain\": 2,\n        \"scanRate\": 1000,\n        \"sleeptime\": 50,\n        \"timeout\": 100\n    },\n    \"res\": {\n        \"count\": 0,\n        \"data\": [\n        ],\n        \"isContinous\": false,\n        \"judge\": false\n    }\n}\n",
        "result": False,
        "task": "CMD_SnapOnce"
    }
    json_str = json.dumps(js_data, indent=1)
    return json_str



def MCFL_CMD_ContinousSnap(task="CMD_ContinousSnap"):
    cmd = {
        'context': 0,
        'result': False,
        'task': task
    }
    json_str = json.dumps(cmd, indent=1)
    return json_str


def MCFL_CMD_StopContinousSnap(task="CMD_StopContinousSnap"):
    cmd = {
        'context': 0,
        'result': False,
        'task': task
    }

    json_str = json.dumps(cmd, indent=1)
    return json_str


def MCFL_CMD_Data(task="CMD_Data"):
    cmd = {
        'context': None,
        'result': False,
        'task': task
    }

    context = {'id': True, 'isContinous': False, 'count': 3, 'data': [1, 22, 32]}

    cmd['context'] = json.dumps(context, indent=1)

    json_str = json.dumps(cmd, indent=1)

    return json_str


def CMD_FL():
    task_context = {
        "id": 1,
        "param": {
            "acqNum": 100,
            "averageNum": 1,
            "bufferNum": 1,
            "bufferSize": 10,
            "emEventType": 1,
            "emTriggerLevel": 0,
            "enableAltitude": True,
            "enableCounter": True,
            "enableTrigger": False,
            "exposureTime": 0.0985,
            "gain": 2,
            "scanRate": 1000,
            "sleeptime": 50,
            "timeout": 100
        },
        "res": {
            "count": 0,
            "data": [],
            "isContinous": False,
            "judge": False
        }
    }

#     MCFL_CMD_Initialize()
#     MCFL_CMD_SnapOnce()
#     MCFL_CMD_Data()
# if __name__ == '__main__':
