# -*- coding: utf-8 -*-
"""
Filename: cmdjson/Camera_CMD.py
Description:
    视觉指令信封（VisionCmdJson 逐字段复刻参考框架
    cmdjson/Camera_CMD.py；camID 用仿真相机 id up/down/coarse）。
    迭代 27 偏差：新增 task 参数（请求序号，默认 "" 与参考一致），
    供接收端做 VSS_Com 回显匹配、防陈旧帧。
"""
import json


def VisionCmdJson(id, template, exposure=2200, gain=1.0, framerate=10,
                  imagename="", is_image_send=False, task=""):
    """视觉识别命令信封。task 为请求序号（如 "IMG_000123"）：后端适配器
    会在 VSS_Com 结果帧原样回显（外层 task 与 content.task 双写），
    接收端据此丢弃上一拍迟到的陈旧帧。参考框架恒 ""——本软件刻意利用
    该闲置字段做帧序号（偏差，迭代 27）。"""
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
        "task": task
    }
    dic = {
        "context": json.dumps(content),
        "module": "",
        "result": 0,
        "task": task
    }
    json_str = json.dumps(dic)
    return json_str


if __name__ == '__main__':
    print(f"{VisionCmdJson(id='up', template='cv_cross_0824', exposure=0)}")
