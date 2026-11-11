import json
# from comm.mqtt_client import paho_client
import time

LST_DO_INDEX = {
    'BondArm_YN': {'name': 'BondArm_YN', "index": 1001, "axisName": "Y"},
    'BondArm_YP': {'name': 'BondArm_YN', "index": 1002, "axisName": "Y"},
    'FlipStage_Y': {'name': 'FlipStage_Y', "index": 1004, "axisName": "Y"},
    'FlipStage_Z': {'name': 'FlipStage_Z', "index": 1005, "axisName": "Z"},
    'FlipStage_RZ': {'name': 'FlipStage_Rz', "index": 1006, "axisName": "RY"},
    'FlipStage_RY': {'name': 'FlipStage_RY', "index": 1007, "axisName": "RZ"},
    'COARSE_Y': {'name': 'COARSE_Y', "index": 1008, "axisName": "Y"},
    'COARSE_X': {'name': 'COARSE_X', "index": 1009, "axisName": "X"},
}

LST_AI_INDEX = {
    "horizon_micro_stage_x": 2051,
    "horizon_micro_stage_y": 2052,
    "horizon_micro_stage_rz": 2053,
}

def MCIO_CMD_DiGroupStatus(task="CMD_DiGroupStatus"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'dic_DIs': {'0': {'Di': 0, 'name': '', 'value': False}}}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)


    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    #payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    # json_str = payloadon.dumps(cmd, indent=1)
    # bytes = json_str.encode('utf-8', 'ignore')
    # print(bytes)

    return json_str


def MCIO_CMD_DoGroupStatus(task="CMD_DoGroupStatus"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'dic_DOs': {'0': {'Do': 0, 'name': '', 'targetValue': False, 'value': False}}}
    cmd['context'] = json.dumps(payload, indent=1)

    json_str = json.dumps(cmd, indent=1)

    return json_str

def MCIO_CMD_AiGroupValue(task="CMD_AiGroupValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'dic_AIs': {'0': {'Ai': 0, 'name': '', 'value': 0}}}
    cmd['context'] = json.dumps(payload, indent=1)

    json_str = json.dumps(cmd, indent=1)

    return json_str

def MCIO_CMD_AoGroupValue(task="CMD_AoGroupValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'dic_AOs': {'0': {'Ao': 0, 'name': '', 'targetValue': 0, 'value': 0}}}
    cmd['context'] = json.dumps(payload, indent=1)

    json_str = json.dumps(cmd, indent=1)
    return json_str

def MCIO_CMD_SetDo(task="CMD_IO_SetDo", Do_index = 2000, io_name="", target_value=False):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'Do': Do_index, 'name': io_name, 'targetVal': target_value, 'value': False}
    cmd['context'] = json.dumps(payload, indent=1)

    json_str = json.dumps(cmd, indent=1)
    return json_str

def MCIO_CMD_SetAo(task="CMD_IO_SetAo", Ao_index = 2050, io_name="", target_value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {'Ao': Ao_index, 'name': io_name, 'targetVal': target_value, 'value': 0}
    cmd['context'] = json.dumps(payload, indent=1)

    json_str = json.dumps(cmd, indent=1)
    return json_str

if __name__ == '__main__':
    MCIO_CMD_DiGroupStatus()
