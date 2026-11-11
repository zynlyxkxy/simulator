# -*- coding: utf-8 -*-
"""
Filename: device_actuator.py
Author: Jianming.Liu
Date: 2025/12/2 
Description:
    设备执行
"""
from cmdjson.PahoMqttClient import PahoMqttClient
from plc_grpc_client import PlcClient
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.MC_FL_cmd as mcFls
import cmdjson.MC_IO_cmd as mcIO
import cmdjson.Camera_CMD as mcCamera
from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from cmdjson.MC_IO_cmd import LST_AI_INDEX
from metrology_config import MetrologyConfig

import copy

def check_safe_condition(paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker):
    # mqtt_receive_worker.get_bonding_head_position
    pass


def move_coarse_stage(target_position, paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker):
    """
    Move the coarse stage to a specified target position.

    :param target_position: The target position to move the coarse stage to.
    :param paho_client: An instance of PahoMqttClient used for publishing MQTT messages.
    :param mqtt_receive_worker: An instance of MqttReceiveWorker used for waiting until the stage stops moving.
    """
    current_axis = 'COARSE_X'
    paho_client.publish("GUI_DR_ACS_Motion",
                        mcmotion.MCMotion_CMD_two_axis_MoveAbs(pos=target_position,
                                                               stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                               stage_index=LST_AXIS_NAME[current_axis]["index"]
                                                               ),
                        0)
    mqtt_receive_worker.wait_stage_stop()
    return mqtt_receive_worker.get_coarse_stage_pos()

def move_coarse_stage_ifm(target_position, paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker,
                          tolerance = 5e-4, max_iterations=3):
    """
    Move the coarse stage to a specified target position.

    :param tolerance:
    :param threshold:
    :param target_position: The target position to move the coarse stage to.
    :param paho_client: An instance of PahoMqttClient used for publishing MQTT messages.
    :param mqtt_receive_worker: An instance of MqttReceiveWorker used for waiting until the stage stops moving.
    """
    current_axis = 'COARSE_X'
    paho_client.publish("GUI_DR_ACS_Motion",
                        mcmotion.MCMotion_CMD_two_axis_MoveAbs(pos=target_position,
                                                               stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                               stage_index=LST_AXIS_NAME[current_axis]["index"]
                                                               ),
                        0)
    mqtt_receive_worker.wait_stage_stop()
    ifm_data = mqtt_receive_worker.get_interferometer_data()
    error_x, error_y = target_position[0] - ifm_data[0], target_position[1] - ifm_data[1]
    iteration = 0
    while (error_x > tolerance or error_y > tolerance) and iteration < max_iterations:
        cur_pos = mqtt_receive_worker.get_coarse_stage_pos()
        grating_ruler_target_pos = [cur_pos[0] + error_x, cur_pos[1] + error_y]
        current_axis = 'COARSE_X'
        paho_client.publish("GUI_DR_ACS_Motion",
                            mcmotion.MCMotion_CMD_two_axis_MoveAbs(pos=grating_ruler_target_pos,
                                                                   stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                                   stage_index=LST_AXIS_NAME[current_axis]["index"]
                                                                   ),
                            0)
        mqtt_receive_worker.wait_stage_stop()

        ifm_data = mqtt_receive_worker.get_interferometer_data()
        error_x, error_y = target_position[0] - ifm_data[0], target_position[1] - ifm_data[1]

    return mqtt_receive_worker.get_coarse_stage_pos()


def move_vertical_micro_stage(paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker,
                              target_rx=None, target_ry=None, target_z=None):
    """
    Move the vertical micro stage to a target position.

    :param paho_client: The MQTT client used for publishing commands.
    :param mqtt_receive_worker: The worker object for handling MQTT messages and getting stage positions.
    :param target_rx: The target X coordinate. If None, the current X coordinate is used.
    :param target_ry: The target Y coordinate. If None, the current Y coordinate is used.
    :param target_z: The target Z coordinate. If None, the current Z coordinate is used.
    :return: The final position of the vertical micro stage after the move.
    """
    cur_position = mqtt_receive_worker.get_vertical_micro_stage_pos()  # Rx, Ry, Z
    target_position = [0, 0, 0]
    if target_rx is not None:
        target_position[0] = target_rx
    else:
        target_position[0] = cur_position[0]

    if target_ry is not None:
        target_position[1] = target_ry
    else:
        target_position[1] = cur_position[1]

    if target_z is not None:
        target_position[2] = target_z
    else:
        target_position[2] = cur_position[2]

    paho_client.publish("GUI_DR_PLC_Motion",
                        mcmotion.MCMotion_CMD_three_axis_MoveAbs(pos=target_position, stage_index=2005),
                        0)
    mqtt_receive_worker.wait_vertical_micro_stage_in_position(target_position)
    return mqtt_receive_worker.get_vertical_micro_stage_pos()


def move_horizontal_micro_stage(paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker,
                                target_x=None, target_y=None, target_rz=None):
    target_position = copy.deepcopy(mqtt_receive_worker.get_horizon_micro_stage_pos())
    if target_x is not None:
        paho_client.publish("GUI_DR_PLC_IO",
                            mcIO.MCIO_CMD_SetAo(Ao_index=LST_AI_INDEX['horizon_micro_stage_x'], target_value=target_x),
                            0)
        target_position[0] = target_x
    if target_y is not None:
        paho_client.publish("GUI_DR_PLC_IO",
                            mcIO.MCIO_CMD_SetAo(Ao_index=LST_AI_INDEX['horizon_micro_stage_y'], target_value=target_y),
                            0)
        target_position[1] = target_y
    if target_rz is not None:
        paho_client.publish("GUI_DR_PLC_IO",
                            mcIO.MCIO_CMD_SetAo(Ao_index=LST_AI_INDEX['horizon_micro_stage_rz'],target_value=target_rz),
                            0)
        target_position[2] = target_rz
    mqtt_receive_worker.wait_horizontal_micro_stage_stop(target_position)
    return mqtt_receive_worker.get_horizon_micro_stage_pos()


def take_photo(camera_id, template_name,
               paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker, plc_client: PlcClient):
    """
    Trigger a camera to take a photo and process the image.

    :param camera_id: The identifier of the camera to trigger.
    :param template_name: The name of the template to use for the photo.
    :param paho_client: PahoMqttClient instance used for publishing MQTT messages.
    :param mqtt_receive_worker: MqttReceiveWorker instance for handling MQTT messages and image processing.
    :param plc_client: PlcClient instance for interacting with the PLC to trigger the camera.
    :return: A tuple containing the processed image data as (arr_row, arr_col).
    """
    mqtt_receive_worker.reset_image_process_status()
    paho_client.publish("VSC_Com",
                        mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0),
                        0)
    plc_client.camera_trigger(camera_id)
    arr_row, arr_col = mqtt_receive_worker.wait_image_process()
    return arr_row, arr_col


def get_picture(camera_id, paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker):
    """
    Sends a command to a specified camera to capture an image and retrieves the raw image data.

    :param camera_id: The unique identifier of the camera from which to capture the image.
    :param paho_client: An instance of PahoMqttClient used for publishing MQTT messages.
    :param mqtt_receive_worker: An instance of MqttReceiveWorker responsible for receiving and processing the image.
    :returns: The raw image data captured by the camera.
    """
    mqtt_receive_worker.reset_image_process_status()
    paho_client.publish("VSC_Com",
                        mcCamera.VisionCmdJson(id=camera_id, template="", exposure=0, is_image_send=True),
                        0)
    mqtt_receive_worker.wait_get_raw_image()
    raw_image = mqtt_receive_worker.get_raw_image()
    return raw_image


def fls_detect(fls_sensor_ID, paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker,
               collection_quantity=10):
    """
    publishing a command to the MQTT broker and waiting for the sensor's response.

    :param fls_sensor_ID: The identifier of the FLS sensor from which to collect data.
    :param paho_client: A PahoMqttClient instance used to publish commands to the MQTT broker.
    :param mqtt_receive_worker: An MqttReceiveWorker instance responsible for handling incoming messages and
                                waiting for the FLS process to complete.
    :param collection_quantity: The number of data points to collect from the FLS sensor. Default is 10.
    :return: The value or data collected from the FLS sensor after the process completes.
    """
    mqtt_receive_worker.reset_fls_process_status()
    paho_client.publish("CTL_WM_CF_ControlManager",
                        mcFls.MCFL_CMD_SnapOnce(sensor_id=fls_sensor_ID, collection_quantity=collection_quantity),
                        0)
    fls_value = mqtt_receive_worker.wait_fls_process()
    return fls_value


def interferometer_set_zero(paho_client: PahoMqttClient, mqtt_receive_worker: MqttReceiveWorker, plc_client: PlcClient):
    """
    Interferometer zero setting function.

    :param paho_client:
    :param mqtt_receive_worker: The MQTT receive worker for handling incoming messages.
    :param plc_client: The PLC client that interacts with the physical device.
    """
    move_coarse_stage([0, 0], paho_client, mqtt_receive_worker)
    plc_client.interferometer_set_zero()


def main():
    pass


if __name__ == '__main__':
    main()
