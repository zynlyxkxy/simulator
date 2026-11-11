import json


def MCAlgorithm_CMD_CoarseAlignResult(task="CMD_GetCoarseAlignResult",
                                  FA_Rough_alignment_ry=-89.3,  FA_Rough_alignment_rz=0,
                                  FA_Rough_alignment_y=-20, FA_Rough_alignment_z=105,
                                  calibration_pixel_x=2037, calibration_pixel_y=1509, calibration_rz=0,
                                  chip_pixel_x=0, chip_pixel_y=0,
                                  image_height=4512, image_width=4512, magnification=0.33, pixel_size=0.00274,
                                  rotation_angle=0, rotation_center_pixel_x=1606.409, rotation_center_pixel_y=2296.318,
                                  theta=0.409779):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {
        "FA_Rough_alignment_ry": FA_Rough_alignment_ry,
        "FA_Rough_alignment_rz": FA_Rough_alignment_rz,
        "FA_Rough_alignment_y": FA_Rough_alignment_y,
        "FA_Rough_alignment_z": FA_Rough_alignment_z,
        "calibration_pixel_x": calibration_pixel_x,
        "calibration_pixel_y": calibration_pixel_y,
        "calibration_rz": calibration_rz,
        "chip_pixel_x": chip_pixel_x,
        "chip_pixel_y": chip_pixel_y,
        "image_height": image_height,
        "image_width": image_width,
        "magnification": magnification,
        "pixel_size": pixel_size,
        "rotation_angle": rotation_angle,
        "rotation_center_pixel_x": rotation_center_pixel_x,
        "rotation_center_pixel_y": rotation_center_pixel_y,
        "theta": theta
    }

    cmd['context'] = json.dumps(payload, indent=1)
    json_str = json.dumps(cmd, indent=1)
    # print(json_str)
    return json_str


if __name__ == '__main__':
    print(MCAlgorithm_CMD_CoarseAlignResult())
