# -*- coding: utf-8 -*-
"""
Filename: metrology_config.py
Description:
    测校配置常数（对照参考框架 metrology_config.py）。
"""
import json
import os
import urllib.request


class MetrologyConfig:
    def __init__(self):
        self.MQTTHOST = "127.0.0.1"
        self.MQTTPORT = 1883

        # CSV 落盘目录：与后端测校同目录（backend/runtime/metrology_data/）
        self.DATA_SAVE_ROOT = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), os.pardir,
            "runtime", "metrology_data")) + os.sep

        # 相机表：中文名 -> camID（后端视觉服务相机 id）
        self.camera_dict = {
            "上精对准": "up",
            "下精对准": "down",
            "粗对准": "coarse",
        }

        # 相机像素当量（nm/px）。初值为机器常数派生值
        # （pixel_um/mag*1000：up/down=0.6um/20x=600，coarse≈9133），
        # 以 GET http://127.0.0.1:9010/api/machine_constants 派生值为准，
        # 启动时可刷新（refresh_nm_per_px）。
        self.camera_info = {
            "up":     {"name_cn": "上精对准", "nm_per_px": 600.0},
            "down":   {"name_cn": "下精对准", "nm_per_px": 600.0},
            "coarse": {"name_cn": "粗对准",   "nm_per_px": 9133.0},
        }

        # 相机 -> 载体运动映射（动态测校）：
        # down→晶圆台 WS_LS_X/Y、up→对准台 AS_X/Y、coarse→翻转模块 FA_X 仅X；
        # "align" 为 down 相机动态测校可选 jog_group（对准台 AS_X/Y，与
        # 后端 /api/calibration start 的 jog_group 契约同名）
        self.camera_stage_map = {
            "down":   {"name_cn": "晶圆台 (WS_LS_X/Y)",  "axes": ["WS_LS_X", "WS_LS_Y"]},
            "up":     {"name_cn": "对准台 (AS_X/Y)",     "axes": ["AS_X", "AS_Y"]},
            "coarse": {"name_cn": "翻转模块 (FA_X 仅X)", "axes": ["FA_X"]},
            "align":  {"name_cn": "对准台 (AS_X/Y)",     "axes": ["AS_X", "AS_Y"]},
        }

    def nm_per_px(self, cam_id):
        return self.camera_info[cam_id]["nm_per_px"]

    def refresh_nm_per_px(self, host="127.0.0.1", port=9010, timeout=2.0):
        """从后端机器常数刷新各相机 nm_per_px（pixel_um/mag*1000）。
        后端不在线时静默保留初值。返回 True=已刷新。"""
        try:
            url = "http://%s:%d/api/machine_constants" % (host, port)
            with urllib.request.urlopen(url, timeout=timeout) as resp:
                body = json.loads(resp.read().decode("utf-8"))
            cams = (body.get("constants") or {}).get("cameras") or {}
            for cam_id, info in self.camera_info.items():
                c = cams.get(cam_id)
                if c and c.get("pixel_um") and c.get("mag"):
                    info["nm_per_px"] = float(c["pixel_um"]) / float(c["mag"]) * 1000.0
            return True
        except Exception:
            return False


if __name__ == "__main__":
    cfg = MetrologyConfig()
    print("DATA_SAVE_ROOT =", cfg.DATA_SAVE_ROOT)
    print("refresh_nm_per_px ->", cfg.refresh_nm_per_px())
    for k, v in cfg.camera_info.items():
        print(k, v)
