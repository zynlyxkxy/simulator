# -*- coding: utf-8 -*-
"""
Filename: metrology_config.py
Date: 2025/9/1
Description:
    测校配置常数
"""

class MetrologyConfig:
    def __init__(self):
        self.MQTTHOST = "127.0.0.1"
        self.MQTTPORT = 1883
        self.DATA_SAVE_ROOT = r"D:/jianming/3000/HoriMetrologyScirptsFramework/data/"

        self.camera_dict = {
            "上精对准": 0,
            "下精对准_XP": 1,
            "下精对准_XN": 2,
            "粗对准": 3,
        }

        self.bonding_head_dict = {
            "负键合臂负键合头": "bonding_arm_YN_head_XN",
            "负键合臂正键合头": "bonding_arm_YN_head_XP",
            "正键合臂负键合头": "bonding_arm_YP_head_XN",
            "正键合臂正键合头": "bonding_arm_YP_head_XP",
        }

        self.camera_setting = {
            0: {
                'name': 'camera_0',
                'pixel_size': 2.74e-3,
                'magnification': 20,
                'image_center_x': 2256,
                'image_center_y': 2256,
            },
            1: {
                'name': 'camera_0',
                'pixel_size': 2.74e-3,
                'magnification': 20,
                'image_center_x': 2256,
                'image_center_y': 2256,
            },
            2: {
                'name': 'camera_0',
                'pixel_size': 2.74e-3,
                'magnification': 20,
                'image_center_x': 2256,
                'image_center_y': 2256,
            },
            3: {
                'name': 'camera_0',
                'pixel_size': 2.74e-3,
                'magnification': 20,
                'image_center_x': 2256,
                'image_center_y': 2256,
            }
        }
