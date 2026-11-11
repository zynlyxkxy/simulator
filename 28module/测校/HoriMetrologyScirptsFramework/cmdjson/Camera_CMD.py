import json


def VisionCmdJson(id, template, exposure=2200, gain=1.0, framerate=10, imagename="", is_image_send=False):
    cellparams = {
        "0": {
            "m_dExposureTime": exposure,
            "m_dFrameRate": framerate,
            "m_dGain": gain,
            "m_nCellId": 0,
            "m_strName": "",
            "m_triggerMode": 0
        },
        "1": {
            "lstShapeName": [
                template
            ],
            "m_nCellId": 0,
            "m_strName": ""
        }
    }
    content = {
        "camID": id,
        "cmdType": 2,
        "content": json.dumps(cellparams),
        "image": imagename,
        "imgType": -1,
        "isImageSend": is_image_send,
        "lstCellType": [
            0,
            1
        ],
        "task": ""
    }
    dic = {
        "context": json.dumps(content),
        "module": "",
        "result": 0,
        "task": ""
    }
    json_str = json.dumps(dic)
    return json_str


def VisionCmdJsonHardTriggerType(id, template, exposure=2200, gain=1.0, framerate=10, imagename="", is_image_send=False):
    cellparams = {
        "0": {
            "m_dExposureTime": exposure,
            "m_dFrameRate": framerate,
            "m_dGain": gain,
            "m_nCellId": 0,
            "m_strName": "",
            "m_triggerMode": 1
        },
        "1": {
            "lstShapeName": [
                template
            ],
            "m_nCellId": 0,
            "m_strName": ""
        }
    }
    content = {
        "camID": id,
        "cmdType": 2,
        "content": json.dumps(cellparams),
        "image": imagename,
        "imgType": -1,
        "isImageSend": is_image_send,
        "lstCellType": [
            0,
            1
        ],
        "task": ""
    }
    dic = {
        "context": json.dumps(content),
        "module": "",
        "result": 0,
        "task": ""
    }
    json_str = json.dumps(dic)
    return json_str



def VisionCmdJson2(id, template, exposure=1000, gain=1.0, framerate=10, imagename=""):
    cellparams = {
        "0": {
            "m_dExposureTime": exposure,
            "m_dFrameRate": framerate,
            "m_dGain": gain,
            "m_nCellId": 0,
            "m_strName": "",
            "m_triggerMode": 0
        },
        "1": {
            "lstShapeName": [
                template
            ],
            "m_nCellId": 0,
            "m_strName": ""
        }
    }
    content = {
        "camID": id,
        "cmdType": 2,
        "content": json.dumps(cellparams),
        "image": imagename,
        "imgType": -1,
        "isImageSend": 0,
        "lstCellType": [
            0,
            1
        ],
        "task": ""
    }

    dic = {
        "context": json.dumps(content),
        "module": "",
        "result": 0,
        "task": ""
    }
    json_str = json.dumps(dic)
    return json_str

def VisionCmdJsonHardTrigger(id, template, exposure=0, gain=1.0, framerate=10, imagename="",task=''):
    cellparams = {
        "0": {
            "m_dExposureTime": exposure,
            "m_dFrameRate": framerate,
            "m_dGain": gain,
            "m_nCellId": 0,
            "m_strName": "",
            "m_triggerMode": 1
        },
        "1": {
            "lstShapeName": [
                template
            ],
            "m_nCellId": 0,
            "m_strName": ""
        }
    }
    content = {
        "camID": int(id),
        "cmdType": 2,
        "content": json.dumps(cellparams),
        "image": imagename,
        "imgType": -100,
        "isImageSend": 0,
        "lstCellType": [
            0,
            1
        ],
        "task": ""
    }

    dic = {
        "context": json.dumps(content),
        "module": "",
        "result": 0,
        "task": task
    }
    json_str = json.dumps(dic)
    print(json_str)
    return json_str


if __name__ == '__main__':
    print(f"{VisionCmdJson(id=0, template='template', exposure=0)}")
