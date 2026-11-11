# -*- coding: utf-8 -*-
"""
Filename: cmdjson/MC_Motion_cmd.py
Description:
    运动指令信封（逐字段复刻参考框架 cmdjson/MC_Motion_cmd.py 的
    one_axis MoveAbs/MoveRel：{task, context:json.dumps(...), result}）。
    轴名直接用仿真轴名（WS_LS_X/WS_LS_Y/AS_X/AS_Y/FA_X，后端适配器
    恒等透传+别名）；stage_index 为名义编号，仅用于保持报文形状。
"""
import json

LST_AXIS_NAME = {
    'WS_LS_X': {'name': 'WS_LS_X', "index": 1008, "axisName": "X"},
    'WS_LS_Y': {'name': 'WS_LS_Y', "index": 1009, "axisName": "Y"},
    'AS_X':    {'name': 'AS_X',    "index": 1010, "axisName": "X"},
    'AS_Y':    {'name': 'AS_Y',    "index": 1011, "axisName": "Y"},
    'FA_X':    {'name': 'FA_X',    "index": 1007, "axisName": "X"},
}


def MCMotion_CMD_one_axis_MoveAbs(task="CMD_Motion_MoveAbs", pos=0.0, stage_name="", stage_index="", axis_name=""):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": stage_name, "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0},
                "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos, "moveRel": 0, "name": axis_name,
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    return json_str


def MCMotion_CMD_one_axis_MoveRel(task="CMD_Motion_MoveRel", pos=0, stage_name="", stage_index="", axis_name=""):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": stage_name, "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0},
                "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": pos, "name": axis_name,
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    return json_str


if __name__ == '__main__':
    print(MCMotion_CMD_one_axis_MoveAbs(
        pos=1.0, stage_name=LST_AXIS_NAME['WS_LS_X']['name'],
        stage_index=LST_AXIS_NAME['WS_LS_X']['index'],
        axis_name=LST_AXIS_NAME['WS_LS_X']['axisName']))
