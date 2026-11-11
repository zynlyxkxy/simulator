# -*- coding: utf-8 -*-
"""
Filename: process_actuator.py
Author: Jianming.Liu
Date: 2025/11/24 
Description:
 
"""

from cmdjson.MC_Motion_cmd import LST_AXIS_NAME
from MqttReceiveWorker import MqttReceiveWorker
import cmdjson.MC_Motion_cmd as mcmotion
import cmdjson.Camera_CMD as mcCamera
from cmdjson.PahoMqttClient import PahoMqttClient
from plc_grpc_client import PlcClient


def take_photo(camera_id: int, template_name: str,
               paho_client: PahoMqttClient,
               mqtt_receive_worker: MqttReceiveWorker,
               plc_client: PlcClient
               ):
    """
    Triggers a camera to take a photo and processes the image.

    Summary:
    This function triggers a specified camera to capture an image using a given template.
    It sends a command through MQTT to trigger the camera, communicates with a PLC for the triggering process,and waits for the image processing to complete.
    Finally, it returns the processed image data as row and column arrays.

    Args:
        camera_id (int): The ID of the camera to be triggered.
        template_name (str): The name of the template to use for the image capture.
        paho_client (PahoMqttClient): A client for publishing MQTT messages.
        mqtt_receive_worker (MqttReceiveWorker): An instance of MqttReceiveWorker used for handling MQTT communication and image processing.
        plc_client (PlcClient): A client for communicating with the PLC to trigger the camera.

    Returns:
        tuple: A tuple containing two numpy arrays, arr_row and arr_col, representing the processed image data.
    """
    mqtt_receive_worker.reset_image_process_status()
    paho_client.publish("VSC_Com",
                        mcCamera.VisionCmdJsonHardTriggerType(id=camera_id, template=template_name, exposure=0),
                        0)
    plc_client.camera_trigger(camera_id)
    arr_row, arr_col = mqtt_receive_worker.wait_image_process()
    return arr_row, arr_col


def move_coarse_stage(coarse_coordinate: list,
                      mqtt_receive_worker: MqttReceiveWorker,
                      paho_client: PahoMqttClient
                      ):
    """
    Move the coarse stage to the specified coordinates.

    This function sends commands over MQTT to move a coarse stage in two axes (X and Y) to the given absolute positions.
    It waits for the stage to stop after sending the movement commands.

    Args:
        coarse_coordinate: list of float
            The target position for the X and Y axes.
        mqtt_receive_worker: MqttReceiveWorker
            An instance of MqttReceiveWorker used to wait for the stage to stop.
        paho_client: PahoMqttClient
            An instance of PahoMqttClient used to publish the movement commands.

    Raises:
        Exception: If an error occurs during the publishing or waiting process.
    """
    current_axis = 'COARSE_X'
    paho_client.publish("GUI_DR_ACS_Motion",
                        mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_coordinate[0],
                                                               stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                               stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                               axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                        0)
    current_axis = 'COARSE_Y'
    paho_client.publish("GUI_DR_ACS_Motion",
                        mcmotion.MCMotion_CMD_one_axis_MoveAbs(pos=coarse_coordinate[1],
                                                               stage_name=LST_AXIS_NAME[current_axis]["name"],
                                                               stage_index=LST_AXIS_NAME[current_axis]["index"],
                                                               axis_name=LST_AXIS_NAME[current_axis]["axisName"]),
                        0)
    mqtt_receive_worker.wait_stage_stop()
    return mqtt_receive_worker.get_coarse_stage_pos()
