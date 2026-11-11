import json
# from comm.mqtt_client import paho_client
import time

LST_AXIS_NAME = {
    'BondArm_YN': {'name': 'BondArm_YN', "index": 1001, "axisName": "Y"},
    'BondArm_YP': {'name': 'BondArm_YP', "index": 1002, "axisName": "Y"},
    'FlipStage_Y': {'name': 'FlipStage_Y', "index": 1004, "axisName": "Y"},
    'FlipStage_Z': {'name': 'FlipStage_Z', "index": 1005, "axisName": "Z"},
    'FlipStage_RZ': {'name': 'FlipStage_Rz', "index": 1006, "axisName": "RY"},
    'FlipStage_RY': {'name': 'FlipStage_RY', "index": 1007, "axisName": "RZ"},
    'COARSE_Y': {'name': 'COARSE_Y', "index": 1009, "axisName": "Y"},
    'COARSE_X': {'name': 'COARSE_X', "index": 1008, "axisName": "X"},
    'BondHead_YNXN': {'name': 'BondHead_YNXN', "index": 1008, "axisName": "X"},
    'BondHead_YNXP': {'name': 'BondHead_YNXP', "index": 1008, "axisName": "X"},
    'BondHead_YPXN': {'name': 'BondHead_YPXN', "index": 1008, "axisName": "X"},
    'BondHead_YPXP': {'name': 'BondHead_YPXP', "index": 1008, "axisName": "X"},
}


def MCMotion_CMD_flip_module_MoveAbs(y_pos=0, z_pos=0, rz_pos=0, ry_pos=0):
    """
    :param y_pos:
    :param z_pos:
    :param ry_pos:
    :param rz_pos:
    :return:
    """
    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_MoveAbs"
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": 1003,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": y_pos, "moveRel": 0, "name": "Y",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": z_pos, "moveRel": 0, "name": "Z",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": rz_pos, "moveRel": 0, "name": "Rz",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": ry_pos, "moveRel": 0, "name": "Ry",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },

        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_flip_module_MoveRel(y_pos=0, z_pos=0, rz_pos=0, ry_pos=0):
    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_MoveRel"
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": 1003,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": y_pos, "name": "Y",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": z_pos, "name": "Z",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": rz_pos, "name": "Rz",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": ry_pos, "name": "Ry",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },

        ]
    }
    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_flip_module_SetVel(y_vel=0, z_vel=0, rz_vel=0, ry_vel=0):
    """
    :return:
    """
    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_SetVel"
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": 1003,
        "lst_Axis": [
            {
                "acc": {"targetVal": y_vel * 10, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": y_vel * 10, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": y_vel * 100, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Y",
                "unit": "mm", "vel": {"targetVal": y_vel, "value": 0}
            },
            {
                "acc": {"targetVal": z_vel * 10, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": z_vel * 10, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": z_vel * 100, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Z",
                "unit": "mm", "vel": {"targetVal": z_vel, "value": 0}
            },
            {
                "acc": {"targetVal": rz_vel * 10, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": rz_vel * 10, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": rz_vel * 100, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Rz",
                "unit": "mm", "vel": {"targetVal": rz_vel, "value": 0}
            },
            {
                "acc": {"targetVal": ry_vel * 10, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": ry_vel * 10, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": ry_vel * 100, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Ry",
                "unit": "mm", "vel": {"targetVal": ry_vel, "value": 0}
            }
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_flip_module_Home():
    """
    :return:
    """
    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_Home"
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": 1003,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Y",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Z",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Ry",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Rz",
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_bonding_arm_MoveAbs(task="CMD_Motion_MoveAbs", pos=0, stage_type=0, move_type=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    if stage_type == 0:
        stage_index = 1001
        axis_name = 'Y'
        stage_name = 'BondArm_YN'
    else:
        stage_index = 1002
        stage_name = 'BondArm_YP'
        axis_name = 'Y'

    payload = {
        "enable": False, "inPos": False, "moving": False, "name": stage_name, "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos, "moveRel": pos, "name": axis_name,
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_bonding_arm_Home(task="CMD_Motion_Home", axis_type=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    if axis_type == 0:
        stage_index = 1001
        name = 'YP'
    else:
        stage_index = 1002
        name = 'YN'

    payload = {
        "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": name,
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


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
    # print(json_str)
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
    # print(json_str)
    return json_str


def MCMotion_CMD_two_axis_MoveAbs(task="CMD_Motion_MoveAbs", pos=None, stage_name="", stage_index="", axis_name=None):
    if pos is None:
        pos = [0, 0]
    if axis_name is None:
        axis_name = ["", ""]

    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": stage_name, "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos[0], "moveRel": 0, "name": axis_name[0],
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos[1], "moveRel": 0, "name": axis_name[1],
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def MCMotion_CMD_three_axis_MoveAbs(task="CMD_Motion_MoveAbs", pos=None, stage_name="",
                                    stage_index="", axis_name=None):
    if pos is None:
        pos = [0, 0, 0]
    if axis_name is None:
        axis_name = ["", "", ""]

    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_MoveAbs"
    }
    payload = {
        "enable": False, "inPos": False, "moving": False, "name": stage_name, "stage": stage_index,
        "lst_Axis": [
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos[0], "moveRel": 0, "name": axis_name[0],
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos[1], "moveRel": 0, "name": axis_name[1],
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },
            {
                "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 0, "value": 0},
                "dec": {"targetVal": 0, "value": 0},
                "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                "jogRate": 0, "moveAbs": pos[2], "moveRel": 0, "name": axis_name[2],
                "unit": "mm", "vel": {"targetVal": 0, "value": 0}
            },

        ]
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


def test_json():
    """
    :return:
    """
    cmd = {
        'context': "",
        'result': False,
        'task': "CMD_Motion_Home"
    }
    payload = {
        "dic_Stage": {
            "1001": {
                "enable": False, "inPos": False, "moving": False, "name": "CoarseMoveStage", "stage": 1001,
                "lst_Axis": [
                    {
                        "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 20, "value": 20},
                        "dec": {"targetVal": 0, "value": 0},
                        "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                        "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Y",
                        "unit": "mm", "vel": {"targetVal": 0, "value": 0}
                    },
                    {
                        "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 10, "value": 10},
                        "dec": {"targetVal": 0, "value": 0},
                        "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                        "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Z",
                        "unit": "mm", "vel": {"targetVal": 0, "value": 0}
                    },
                    {
                        "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 9, "value": 9},
                        "dec": {"targetVal": 0, "value": 0},
                        "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                        "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Ry",
                        "unit": "mm", "vel": {"targetVal": 0, "value": 0}
                    },
                    {
                        "acc": {"targetVal": 0, "value": 0}, "coord": {"targetVal": 1, "value": 1},
                        "dec": {"targetVal": 0, "value": 0},
                        "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                        "jogRate": 0, "moveAbs": 0, "moveRel": 0, "name": "Rz",
                        "unit": "mm", "vel": {"targetVal": 0, "value": 0}
                    },
                ]
            }
        }
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


if __name__ == '__main__':
    print(test_json())
