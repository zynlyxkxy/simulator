import numpy as np


class RotationCalculator:
    """
    旋转角度及旋转中心计算器

    功能如下：
    1. 计算两条线段之间的夹角
    2. 计算两点连线的垂直平分线
    3. 计算两条直线的交点
    4. 计算旋转中心
    5. 将像素坐标转换为世界坐标
    """

    @staticmethod
    def angle_between_segments(p1, p2, p3, p4):
        """
        计算两条线段之间的夹角（弧度）

        参数:
            p1, p2: 第一条线段的两个端点坐标，例如 [x, y]
            p3, p4: 第二条线段的两个端点坐标，例如 [x, y]

        返回:
            两条线段对应向量之间的夹角（弧度）
        """
        # 将坐标转换为numpy数组，构建向量
        v1 = np.array(p2) - np.array(p1)
        v2 = np.array(p4) - np.array(p3)

        # 计算点积
        dot = np.dot(v1, v2)

        # 计算向量的模
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        # 计算夹角余弦值并用np.clip确保在[-1, 1]范围内
        cos_angle = dot / (norm_v1 * norm_v2)
        cos_angle = np.clip(cos_angle, -1, 1)

        # 计算并返回夹角（弧度）
        return np.arccos(cos_angle)

    @staticmethod
    def perpendicular_bisector(p, q):
        """
        求通过点 p 和 q 的连线的垂直平分线的线性方程系数。
        线性方程形式为: A*x + B*y = C

        参数:
            p, q: 两个二维点，例如 [x, y]

        返回:
            (A, B, C) 三个系数，表示垂直平分线的直线方程。
        """
        p = np.array(p)
        q = np.array(q)
        # 计算中点
        mid = (p + q) / 2
        # 计算连线向量
        dx = q[0] - p[0]
        dy = q[1] - p[1]
        # 对于连线向量 (dx, dy)，垂直平分线的法向量正好可以取 (dx, dy)
        # 因为垂直平分线满足 (dx, dy) · ([x, y] - mid) = 0
        # 整理得 A*x + B*y = A*mid_x + B*mid_y，其中 A=dx, B=dy, C = dx*mid_x + dy*mid_y
        A = dx
        B = dy
        C = A * mid[0] + B * mid[1]
        return A, B, C

    @staticmethod
    def line_intersection(line1, line2):
        """
        计算两条直线的交点。
        两直线方程分别为: A1*x + B1*y = C1, A2*x + B2*y = C2

        参数:
            line1, line2: 分别为 (A, B, C)

        返回:
            交点坐标 [x, y]

        若两条直线平行或重合，则抛出异常。
        """
        A1, B1, C1 = line1
        A2, B2, C2 = line2
        det = A1 * B2 - A2 * B1
        if abs(det) < 1e-10:
            raise ValueError("两条垂直平分线平行或重合，无法确定唯一的旋转中心")
        x = (C1 * B2 - B1 * C2) / det
        y = (A1 * C2 - C1 * A2) / det
        return np.array([x, y])

    @staticmethod
    def find_rotation_center(A, A_prime, B, B_prime):
        """
        根据两组对应点（旋转前和旋转后的点）计算旋转中心坐标。

        参数:
            A, B: 旋转前线段的两个端点坐标，如 [x, y]
            A_prime, B_prime: 分别是旋转后与 A 和 B 对应的点坐标

        返回:
            旋转中心坐标 [x, y]
        """
        # 对应点 A 与 A_prime 连线的垂直平分线
        bisector1 = RotationCalculator.perpendicular_bisector(A, A_prime)
        # 对应点 B 与 B_prime 连线的垂直平分线
        bisector2 = RotationCalculator.perpendicular_bisector(B, B_prime)
        # 交点即为旋转中心
        center = RotationCalculator.line_intersection(bisector1, bisector2)
        return center

    @staticmethod
    def pixel_to_world(pixel_coord,  world_coord, pixel_size, magnification, image_center_x, image_center_y):
        """
        :param pixel_x: 输入标记的x向像素坐标
        :param pixel_y: 输入标记的y向像素坐标
        :param world_x: 输入标记所处的世界x向坐标
        :param world_y: 输入标记所处的世界y向坐标
        :param pixel_size: 像元大小
        :param magnification: 镜头放大倍率
        :param image_center_x: 图像像素中心的x向像素
        :param image_center_y: 图像像素中心的y向像素
        :return: 世界坐标
        """
        world_pixel_size = pixel_size/magnification
        pixel2world_x = (pixel_coord[0] - image_center_x) * world_pixel_size
        pixel2world_y = (image_center_y - pixel_coord[1]) * world_pixel_size

        final_world_x = world_coord[0] + pixel2world_x
        final_world_y = world_coord[1] + pixel2world_y
        return final_world_x, final_world_y