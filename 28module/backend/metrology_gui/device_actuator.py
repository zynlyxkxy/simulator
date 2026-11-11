# -*- coding: utf-8 -*-
"""
Filename: device_actuator.py
Description:
    设备执行（对照参考框架 device_actuator.py，裁剪到本迭代需要的
    move_stage / take_photo 两个组合原语）：
    - move_stage：逐轴 publish GUI_DR_ACS_Motion MoveAbs → wait_stage_stop；
    - take_photo：reset → publish VSC_Com VisionCmdJson → wait_image_process。

    与参考框架的刻意偏差（迭代 27）：
    - 参考 take_photo 把 wait_image_process 的 [0,0] 超时哨兵原样返回，
      0 值混进统计致 3σ 爆表；本实现失败/超时会重拍 1 次，仍败抛
      TakePhotoError（带原因），绝不把 0 值交给调用方；
    - 每拍在 VSC_Com 信封 task 字段塞单调递增序号 "IMG_NNNNNN"，
      接收端只收回显同序号的 VSS_Com（防上一拍迟到帧污染）。
"""
import itertools

from cmdjson.PahoMqttClient import PahoMqttClient
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from mqtt_receive_worker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera

# 拍照请求序号（单调递增，回显匹配用）
_IMG_SEQ = itertools.count(1)


class TakePhotoError(RuntimeError):
    """拍照失败（含重拍仍败）：message 带后端给的失败原因/超时。"""


def move_stage(axes, positions, paho_client: PahoMqttClient,
               mqtt_receive_worker: MqttReceiveWorker):
    """把 axes 中的各轴 MoveAbs 到 positions 对应位置（mm），并等待台架停止。

    :param axes: 仿真轴名列表（如 ["WS_LS_X", "WS_LS_Y"]）
    :param positions: 与 axes 等长的目标位置列表（mm）
    """
    for axis, pos in zip(axes, positions):
        paho_client.publish(
            "GUI_DR_ACS_Motion",
            mcmotion.MCMotion_CMD_one_axis_MoveAbs(
                pos=pos,
                stage_name=LST_AXIS_NAME[axis]["name"],
                stage_index=LST_AXIS_NAME[axis]["index"],
                axis_name=LST_AXIS_NAME[axis]["axisName"]),
            0)
    mqtt_receive_worker.wait_stage_stop()


def take_photo(camera_id, template_name, paho_client: PahoMqttClient,
               mqtt_receive_worker: MqttReceiveWorker,
               timeout=15.0, max_retry=1):
    """触发指定相机拍照并等待识别结果。返回 (row, col)。

    失败语义（迭代 27 新契约）：识别失败（judge=false/空 lst/error）或
    超时返回 None 时重拍 max_retry 次；仍败抛 TakePhotoError（带原因）。
    绝不返回 0 值。

    :param timeout: 单拍等待秒数（默认 15.0s 直给——迭代 42 用户拍板；
      None 用 wait_image_process 默认）
    :param max_retry: 失败后的重拍次数（默认 1，即最多 2 拍）
    """
    last_reason = "unknown"
    prev_tag = None
    for attempt in range(1 + max_retry):
        tag = "IMG_%06d" % next(_IMG_SEQ)
        # 迭代 33：登记期望 camID——camID 不符的迟到/错位帧按陈旧帧丢弃；
        # 迭代 48：also_accept 收下上一 attempt 的迟到帧（后端单拍最坏
        # ≈12s 帧等待+渲染，贴着 15s 窗时 attempt1 回帧迟到被陈旧过滤、
        # attempt2 再超时的双杀放大；同机位下上一拍结果物理等价）
        mqtt_receive_worker.reset_image_process_status(
            task=tag, cam=camera_id,
            also_accept={prev_tag} if prev_tag else None)
        prev_tag = tag
        paho_client.publish("VSC_Com",
                            mcCamera.VisionCmdJson(id=camera_id,
                                                   template=template_name,
                                                   exposure=0, task=tag),
                            0)
        if timeout is None:
            res = mqtt_receive_worker.wait_image_process()
        else:
            res = mqtt_receive_worker.wait_image_process(timeout=timeout)
        if res is not None:
            return res[0], res[1]
        last_reason = mqtt_receive_worker.last_image_error() or "unknown"
        print("[take_photo] 第 %d 拍失败(%s)%s"
              % (attempt + 1, last_reason,
                 "，重拍" if attempt < max_retry else ""))
    raise TakePhotoError("拍照失败（重拍 %d 次仍败）: %s"
                         % (max_retry, last_reason))
