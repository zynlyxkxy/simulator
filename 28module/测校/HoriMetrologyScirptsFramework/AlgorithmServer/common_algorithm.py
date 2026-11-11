import numpy as np
# from test.test import iterate_poly

def ega_nominal_to_real(nominal, params):
    u = nominal[0]
    v = nominal[1]
    x = params[0] + params[2] * u - params[4] * v
    y = params[1] + params[3] * v + params[5] * u
    return np.array([x, y])

def ega_nominal_list_to_real(nominal, params):
    real_positions = []
    for (u, v) in nominal:
        x = params[0] + params[2] * u - params[4] * v
        y = params[1] + params[3] * v + params[5] * u
        real_positions.append((x, y))
    return np.array(real_positions)

def create_chip_position(wafer_radius, chip_width, chip_height, spacing):
    """创建晶圆上的芯片位置"""
    step_x = chip_width + spacing
    step_y = chip_height + spacing

    # 生成x坐标网格
    x_coords = np.arange(-wafer_radius + step_x / 2, wafer_radius, step_x)
    if len(x_coords[x_coords >= 0]) > 0:
        x_coords = x_coords - np.min(x_coords[x_coords >= 0])

    # 生成y坐标网格
    y_coords = np.arange(-wafer_radius + step_y / 2, wafer_radius, step_y)
    if len(y_coords[y_coords >= 0]) > 0:
        y_coords = y_coords - np.min(y_coords[y_coords >= 0])

    xv, yv = np.meshgrid(x_coords, y_coords)
    grid_positions = np.column_stack((xv.ravel(), yv.ravel()))

    # 计算芯片角点到中心的距离（用于边界检查）
    corner_distance = np.sqrt((chip_width / 2) ** 2 + (chip_height / 2) ** 2)

    # 过滤超出晶圆的芯片
    distances = np.sqrt(grid_positions[:, 0] ** 2 + grid_positions[:, 1] ** 2)
    nominal_positions = grid_positions[distances + corner_distance < wafer_radius]

    return nominal_positions

def image_coordinate_to_real(image_coordinate, stage_position, camera_id, image_center, pixel_length):
    """
    像素坐标转世界坐标
    :param image_coordinate: 像素坐标 [row, col]
    :param stage_position: 运动台坐标 [x, y]
    :param camera_id: 相机ID
    :param image_center: 相机图像中心位置坐标 [center_row, center_col]
    :param pixel_length: 单个像素对应世界坐标大小 mm
    :return: 世界坐标 [x, y] mm
    """
    world_position = [0, 0]
    if camera_id == 0:
        world_position[0] = -(stage_position[0] + (image_coordinate[1] - image_center[1])) * pixel_length
        world_position[1] = -(stage_position[1] - (image_coordinate[0] - image_center[0])) * pixel_length
    elif camera_id == 1 or camera_id == 2:
        world_position[0] = (stage_position[0] + (image_coordinate[1] - image_center[1])) * pixel_length
        world_position[1] = (stage_position[1] - (image_coordinate[0] - image_center[0])) * pixel_length

    return world_position