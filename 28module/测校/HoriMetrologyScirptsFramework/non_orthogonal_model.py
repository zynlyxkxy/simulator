import numpy as np

class non_orthogonal_model:
    @staticmethod
    def X_rotation(coords):
        """
        计算测校版上台后的旋转量
        :param coords: 实际被测对象坐标
        :return: 测校版旋转量
        """
        alpha = np.arctan((coords[:, 1][-1] - coords[:, 1][0]) / (coords[:, 0][-1] - coords[:, 0][0]))
        return alpha

    @staticmethod
    def Y_rotation( coords):
        """
        计算测校版上台后的旋转量
        :param real_x_positions: 实际被测对象坐标
        :return: 测校版旋转量
        """
        alpha = np.arctan((coords[:, 0][-1] - coords[:, 0][0]) / (coords[:, 1][-1] - coords[:, 1][0]))
        return -alpha

    @staticmethod
    def Compensated_rotation(coords, alpha):
        """
        补偿测校版上台旋转量
        :param x: 需补偿的x坐标
        :param y: 需补偿的y坐标
        :param alpha: 旋转角
        :return: 补偿后的x, y
        """
        rotation_center = np.array([np.mean(coords[:, 0]), np.mean(coords[:, 1])])
        # rotation_center = np.array([0, 0])
        c = np.cos(-alpha)
        s = np.sin(-alpha)
        R = np.array([[c, -s],
                      [s, c]], dtype=float)
        new_coords = []
        for i in range(coords.shape[0]):
            n_rot = R @ (coords[i] - rotation_center) + rotation_center
            new_coords.append(n_rot)
        return np.array(new_coords)

    @staticmethod
    def apply_compensation(instruction_coords, matrix):
        compensated_instruction = matrix @ instruction_coords
        display_coords = np.linalg.inv(matrix) @ compensated_instruction
        return np.array(compensated_instruction), np.array(display_coords)