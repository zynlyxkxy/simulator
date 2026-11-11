import sys
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class ChipItem:
    def __init__(self, center_x, center_y, real_center_x, real_center_y, width, height, index):
        self.center = QPointF(center_x, center_y)
        self.real_center = QPointF(real_center_x, real_center_y)
        self.width = width
        self.height = height
        self.index = index
        self.is_selected = False
        self.bounds = QRectF(
            center_x - width / 2,
            center_y - height / 2,
            width, height
        )

    def contains(self, point):
        return self.bounds.contains(point)

    def get_center(self):
        return self.center

    def get_position_text(self):
        return f"Chip {self.index}: ({self.center.x():.2f}, {self.center.y():.2f})"

    def get_size_text(self):
        return f"{self.width}×{self.height}"

    def get_real_position_text(self):
        return f"({self.real_center.x():.2f}, {self.real_center.y():.2f})"


class WaferWidget(QWidget):
    def __init__(self, wafer_radius=300, chip_center_position_list = None, chip_width=25, chip_height=20):
        super().__init__()
        self.wafer_radius = wafer_radius
        self.chip_width = chip_width
        self.chip_height = chip_height
        self.chips = []

        self.selected_chips = set()
        self.hovered_chip = None
        self.show_coordinates = False

        # 添加缩放相关属性
        self.scale_factor = 1.5
        self.init_scale = self.scale_factor
        self.min_scale = 0.3
        self.max_scale = 3.0
        self.translation = QPointF(0, 0)
        self.is_panning = False
        self.last_mouse_pos = QPoint()

        # 设置鼠标跟踪
        self.setMouseTracking(True)

        # 设置固定大小
        self.setFixedSize(650, 650)

        # 创建示例芯片数据
        self.create_sample_chips()
        #
        # if chip_center_position_list is not None:
        #     self.set_chip_info(chip_center_position_list)

        self.fit_to_view()

    def set_chip_info(self, chip_center_position_list):
        self.chips = []
        chip_index = 0
        width = self.chip_width
        height = self.chip_height
        for chip_center in chip_center_position_list:
            self.chips.append(ChipItem(chip_center[0], chip_center[1], chip_center[2], chip_center[3],
                                       width, height, chip_index))
            chip_index += 1

        self.fit_to_view()

    def create_sample_chips(self):
        """创建示例芯片布局"""
        self.chips = []

        # 在晶圆范围内生成网格状芯片
        grid_x = int(self.wafer_radius * 2 / (self.chip_width * 1.8))
        grid_y = int(self.wafer_radius * 2 / (self.chip_height * 1.8))
        chip_index = 0

        for i in range(-grid_x, grid_x + 1):
            for j in range(-grid_y, grid_y + 1):
                x = i * self.chip_width * 1.8
                y = j * self.chip_height * 1.8

                # 只添加在晶圆范围内的芯片
                distance = np.sqrt(x ** 2 + y ** 2)
                if distance <= self.wafer_radius * 0.9:  # 留出边界
                    # 随机改变一些芯片的尺寸，增加可视化效果
                    width = self.chip_width
                    height = self.chip_height

                    self.chips.append(ChipItem(x, y, x, y,width, height, chip_index))
                    chip_index += 1

    def fit_to_view(self):
        if self.chips is None or len(self.chips) == 0:
            return

        min_x, max_x = self.chips[0].get_center().x(),  self.chips[0].get_center().x()
        min_y, max_y = self.chips[0].get_center().y(),  self.chips[0].get_center().y()

        for chip in self.chips:
            min_x = min(min_x, chip.get_center().x())
            max_x = max(max_x, chip.get_center().x())
            min_y = min(min_y, chip.get_center().y())
            max_y = max(max_y, chip.get_center().y())

        padding = max(max_x - min_x, max_y - min_y) * 0.05
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding

        content_width = max_x- min_x
        content_height = max_y- min_y
        if content_width > 0 and content_height > 0 :
            scale_x = (self.width() - 50) / content_width
            scale_y = (self.height() - 50) / content_height

            scale = min(scale_x, scale_y)

            self.scale_factor = max(self.min_scale, min(self.max_scale, scale))
            self.init_scale = self.scale_factor

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 应用缩放和平移变换
        painter.translate(self.width() / 2, self.height() / 2)
        painter.translate(self.translation.x(), self.translation.y())
        painter.scale(self.scale_factor, self.scale_factor)

        # 绘制白色背景
        painter.fillRect(
            -self.width() // 2, -self.height() // 2,
            self.width(), self.height(),
            QColor(255, 255, 255)
        )

        # 绘制晶圆
        self.draw_wafer(painter)

        # 绘制所有芯片
        for chip in self.chips:
            self.draw_chip(painter, chip)

        # 绘制坐标轴
        if self.show_coordinates:
            self.draw_axes(painter)

        # 绘制鼠标悬停提示
        if self.hovered_chip is not None:
            self.draw_hover_tooltip(painter, self.hovered_chip)

        # 绘制缩放比例提示
        self.draw_scale_info(painter)

    def draw_wafer(self, painter):
        """绘制晶圆"""
        # 绘制晶圆阴影
        gradient = QRadialGradient(0, 0, self.wafer_radius)
        gradient.setColorAt(0, QColor(245, 245, 250))
        gradient.setColorAt(1, QColor(230, 230, 240))
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(200, 200, 210), 2))
        painter.drawEllipse(
            -int(self.wafer_radius), -int(self.wafer_radius),
            int(self.wafer_radius) * 2, int(self.wafer_radius) * 2
        )

        # 绘制晶圆边缘高光
        pen = QPen(QColor(255, 255, 255, 150), 2)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(
            -int(self.wafer_radius) + 1, -int(self.wafer_radius) + 1,
            int(self.wafer_radius) * 2 - 2, int(self.wafer_radius) * 2 - 2
        )

    def draw_chip(self, painter, chip):
        """绘制单个芯片"""
        rect = chip.bounds

        # 根据状态设置颜色
        if chip.is_selected:
            # 选中状态：蓝色渐变
            gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
            gradient.setColorAt(0, QColor(100, 150, 255))
            gradient.setColorAt(1, QColor(50, 100, 220))
            brush = QBrush(gradient)
            pen_color = QColor(30, 80, 200)
        elif chip == self.hovered_chip:
            # 悬停状态：绿色渐变
            gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
            gradient.setColorAt(0, QColor(100, 220, 150))
            gradient.setColorAt(1, QColor(50, 180, 120))
            brush = QBrush(gradient)
            pen_color = QColor(30, 150, 100)
        else:
            # 普通状态：根据芯片大小使用不同颜色
            color_intensity = 150
            gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
            gradient.setColorAt(0, QColor(color_intensity, color_intensity, color_intensity))
            gradient.setColorAt(1, QColor(color_intensity - 30, color_intensity - 30, color_intensity - 30))
            brush = QBrush(gradient)
            pen_color = QColor(100, 100, 100)

        # 绘制芯片主体
        painter.setBrush(brush)
        painter.setPen(QPen(pen_color, 1.5))
        painter.drawRoundedRect(rect, 2, 2)

        # 绘制芯片中心的点
        painter.setBrush(QBrush(pen_color))
        painter.drawEllipse(chip.center, 1.5, 1.5)

        # 在芯片上显示编号（缩放级别足够大时显示）
        if rect.width() > 15 and rect.height() > 10 and self.scale_factor > 0.5:
            painter.setPen(QPen(QColor(50, 50, 50), 1))
            font_size = max(6, int(min(rect.width(), rect.height()) / 3))
            font = QFont("Arial", font_size)
            painter.setFont(font)
            painter.drawText(rect, Qt.AlignCenter, str(chip.index))

    def draw_axes(self, painter):
        """绘制坐标轴"""
        # 根据缩放级别调整坐标轴线条粗细
        line_width = 1.5 / self.scale_factor if self.scale_factor < 1 else 1.5
        pen = QPen(QColor(150, 150, 150), line_width)
        painter.setPen(pen)

        # 根据缩放级别调整显示范围
        display_radius = int(self.wafer_radius / self.scale_factor)

        # X轴
        painter.drawLine(-display_radius, 0, display_radius, 0)
        # Y轴
        painter.drawLine(0, -display_radius, 0, display_radius)

        # 坐标轴标签
        if self.scale_factor > 0.5:  # 缩放级别足够大时才显示标签
            painter.setPen(QPen(QColor(100, 100, 100), 1))
            font = QFont("Arial", max(8, int(10 / self.scale_factor)))
            painter.setFont(font)

            # X轴刻度
            tick_interval = 50
            if self.scale_factor > 2:
                tick_interval = 20
            elif self.scale_factor < 0.8:
                tick_interval = 100

            for i in range(-int(display_radius), int(display_radius) + tick_interval, tick_interval):
                if i != 0 and abs(i) <= display_radius:
                    painter.drawLine(i, int(-5 / self.scale_factor), i, int(5 / self.scale_factor))
                    painter.drawText(i - 10, int(20 / self.scale_factor), str(i))

            # Y轴刻度
            for i in range(-int(display_radius), int(display_radius) + tick_interval, tick_interval):
                if i != 0 and abs(i) <= display_radius:
                    painter.drawLine(int(-5 / self.scale_factor), i, int(5 / self.scale_factor), i)
                    painter.drawText(int(10 / self.scale_factor), int(i + 5 / self.scale_factor), str(i))

            # 原点标签
            if self.scale_factor > 1:
                painter.drawText(int(5 / self.scale_factor), int(15 / self.scale_factor), "(0,0)")

    def draw_hover_tooltip(self, painter, chip):
        """绘制鼠标悬停提示"""
        if chip is None:
            return

        # 保存当前绘制状态
        painter.save()

        # 重置变换（不应用缩放和平移）
        painter.resetTransform()

        # 获取鼠标在窗口中的位置
        mouse_pos = self.mapFromGlobal(QCursor.pos())

        # 绘制提示框
        text1 = chip.get_position_text()
        text2 = f"运动台坐标: {chip.get_real_position_text()}"
        font = QFont("Arial", 9)
        painter.setFont(font)
        fm = QFontMetrics(font)

        # 计算文本宽度和高度
        text_width = max(fm.width(text1), fm.width(text2)) + 20
        text_height = fm.height() * 2 + 15

        # 确定提示框位置（在鼠标右下方，避免超出窗口边界）
        tooltip_x = mouse_pos.x() + 15
        tooltip_y = mouse_pos.y() + 15

        # 如果提示框超出右边界，调整到左侧
        if tooltip_x + text_width > self.width():
            tooltip_x = mouse_pos.x() - text_width - 15

        # 如果提示框超出下边界，调整到上方
        if tooltip_y + text_height > self.height():
            tooltip_y = mouse_pos.y() - text_height - 15

        # 提示框矩形
        tooltip_rect = QRectF(
            tooltip_x,
            tooltip_y,
            text_width, text_height
        )

        # 绘制圆角矩形背景
        painter.setBrush(QBrush(QColor(50, 50, 50, 220)))
        painter.setPen(QPen(QColor(30, 30, 30), 1))
        painter.drawRoundedRect(tooltip_rect, 5, 5)

        # 绘制文本
        painter.setPen(QPen(QColor(255, 255, 255), 1))

        # 绘制第一行文本
        text1_rect = QRectF(tooltip_rect)
        text1_rect.setHeight(fm.height() + 5)
        painter.drawText(text1_rect, Qt.AlignCenter, text1)

        # 绘制第二行文本
        text2_rect = QRectF(tooltip_rect)
        text2_rect.setTop(text1_rect.bottom())
        painter.drawText(text2_rect, Qt.AlignCenter, text2)

        # 恢复绘制状态
        painter.restore()

    def draw_scale_info(self, painter):
        """绘制缩放比例信息"""
        # 保存当前变换状态
        painter.save()

        # 重置变换（不应用缩放和平移）
        painter.resetTransform()

        # 在右下角显示缩放比例
        font = QFont("Arial", 10)
        painter.setFont(font)
        scale_text = f"缩放: {self.scale_factor:.2f}x"

        # 计算文本尺寸
        fm = QFontMetrics(font)
        text_width = fm.width(scale_text)
        text_height = fm.height()

        # 绘制背景框
        bg_rect = QRect(
            self.width() - text_width - 20,
            self.height() - text_height - 10,
            text_width + 10,
            text_height + 5
        )

        painter.setBrush(QBrush(QColor(0, 0, 0, 180)))
        painter.setPen(QPen(QColor(0, 0, 0, 0)))
        painter.drawRoundedRect(bg_rect, 5, 5)

        # 绘制文本
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(
            self.width() - text_width - 15,
            self.height() - 10,
            scale_text
        )

        # 恢复变换状态
        painter.restore()

    def wheelEvent(self, event):
        """鼠标滚轮事件 - 实现缩放"""
        # 获取鼠标在晶圆坐标系中的位置（应用当前变换前的位置）
        mouse_pos = QPointF(
            event.x() - self.width() / 2,
            event.y() - self.height() / 2
        )

        # 计算缩放前的点相对于晶圆中心的位置
        point_before_scale = (mouse_pos - self.translation) / self.scale_factor

        # 计算缩放因子
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            # 放大
            self.scale_factor *= zoom_factor
            if self.scale_factor > self.max_scale:
                self.scale_factor = self.max_scale
        else:
            # 缩小
            self.scale_factor /= zoom_factor
            if self.scale_factor < self.min_scale:
                self.scale_factor = self.min_scale

        # 计算缩放后的点相对于晶圆中心的位置
        point_after_scale = point_before_scale * self.scale_factor

        # 调整平移以使鼠标点保持在相同位置
        self.translation += mouse_pos - point_after_scale

        # 限制平移范围
        max_translation = self.wafer_radius * (self.scale_factor - 0.5)
        if max_translation > 0:
            self.translation.setX(max(-max_translation, min(max_translation, self.translation.x())))
            self.translation.setY(max(-max_translation, min(max_translation, self.translation.y())))

        # 更新显示
        self.update()

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        if event.button() == Qt.LeftButton:
            # 转换鼠标位置到晶圆坐标系（考虑缩放和平移）
            mouse_pos = QPointF(
                event.x() - self.width() / 2,
                event.y() - self.height() / 2
            )

            # 应用逆变换获取实际晶圆坐标
            actual_pos = (mouse_pos - self.translation) / self.scale_factor

            # 检查是否按下了Ctrl键
            ctrl_pressed = event.modifiers() & Qt.ControlModifier

            # 如果没有按Ctrl键，清空之前的选择
            if not ctrl_pressed:
                for chip in self.chips:
                    chip.is_selected = False
                self.selected_chips.clear()

            # 查找点击的芯片
            clicked_chip = None
            for chip in self.chips:
                if chip.contains(actual_pos):
                    clicked_chip = chip
                    break

            if clicked_chip:
                # 切换选中状态
                clicked_chip.is_selected = not clicked_chip.is_selected

                if clicked_chip.is_selected:
                    self.selected_chips.add(clicked_chip.index)
                else:
                    self.selected_chips.discard(clicked_chip.index)

                # 更新显示
                self.update()

        elif event.button() == Qt.MiddleButton or event.button() == Qt.RightButton:
            # 中键或右键开始拖拽平移
            self.is_panning = True
            self.last_mouse_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        # 处理平移
        if self.is_panning:
            delta = event.pos() - self.last_mouse_pos
            self.translation += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()

            # 限制平移范围
            max_translation = self.wafer_radius * (self.scale_factor - 0.5)
            if max_translation > 0:
                self.translation.setX(max(-max_translation, min(max_translation, self.translation.x())))
                self.translation.setY(max(-max_translation, min(max_translation, self.translation.y())))

            self.update()
            return

        # 转换鼠标位置到晶圆坐标系（考虑缩放和平移）
        mouse_pos = QPointF(
            event.x() - self.width() / 2,
            event.y() - self.height() / 2
        )

        # 应用逆变换获取实际晶圆坐标
        actual_pos = (mouse_pos - self.translation) / self.scale_factor

        # 查找悬停的芯片
        old_hovered = self.hovered_chip
        self.hovered_chip = None

        for chip in self.chips:
            if chip.contains(actual_pos):
                self.hovered_chip = chip
                break

        # 如果悬停状态改变，更新显示
        if old_hovered != self.hovered_chip:
            self.update()

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.MiddleButton or event.button() == Qt.RightButton:
            self.is_panning = False
            self.setCursor(Qt.ArrowCursor)

    def reset_view(self):
        """重置视图到初始状态"""
        self.scale_factor = self.init_scale
        self.translation = QPointF(0, 0)
        self.update()

    def get_selected_chip_positions(self)->list[ChipItem]:
        """获取选中芯片的位置信息"""
        positions = []
        for chip in self.chips:
            if chip.is_selected:
                positions.append({
                    'index': chip.index,
                    'center_x': chip.center.x(),
                    'center_y': chip.center.y(),
                    'real_center_x': chip.real_center.x(),
                    'real_center_y': chip.real_center.y(),
                    'width': chip.width,
                    'height': chip.height
                })
        return positions


class WaferViewer(QWidget):
    sig_selected_points = pyqtSignal(list)
    sig_move_to_mark_position = pyqtSignal(list)

    def __init__(self, wafer_radius=300, chip_center_position_list=None, chip_width=25, chip_height=20,
                 wafer_mark_offset_x = 0, wafer_mark_offset_y = 0, ega_param=None):
        super().__init__()

        self.wafer_mark_offset_x = wafer_mark_offset_x
        self.wafer_mark_offset_y = wafer_mark_offset_y
        if ega_param is not None:
            self.ega_param = ega_param
        else:
            ega_params = np.array([-54.2204, -186.045, 0.999983, 1.00003, 0.00280008, 0.002721])
            self.ega_param = [0, 0, 1, 1, 0, 0]

        self.setWindowTitle("晶圆芯片布局")
        self.setGeometry(100, 100, 900, 750)

        # 设置应用样式
        self.setStyleSheet("""
            QWidget {
                background-color: #f5f5f5;
            }
            QPushButton {
                background-color: #4a86e8;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5d9bf7;
            }
            QPushButton:pressed {
                background-color: #3c78d8;
            }
            QPushButton#resetButton {
                background-color: #e67e22;
            }
            QPushButton#resetButton:hover {
                background-color: #f39c12;
            }
            QLabel {
                color: #333;
                font-weight: bold;
            }
            QGroupBox {
                border: 2px solid #ccc;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                color: #444;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QCheckBox {
                color: #444;
            }
        """)
        self.init_ui(wafer_radius, chip_center_position_list, chip_width, chip_height)

    def init_ui(self, wafer_radius=300, chip_center_position_list = None, chip_width=25, chip_height=20):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # 左侧面板（晶圆显示）
        left_panel = QFrame()
        left_panel.setStyleSheet("background-color: white; border-radius: 8px;")
        left_layout = QVBoxLayout(left_panel)

        # 晶圆显示区域
        self.wafer_widget = WaferWidget(wafer_radius, chip_center_position_list, chip_width, chip_height)
        left_layout.addWidget(self.wafer_widget)

        # 添加晶圆信息标签
        info_label = QLabel(
            f"晶圆半径: {self.wafer_widget.wafer_radius:.3f} | 芯片尺寸: {self.wafer_widget.chip_width}×{self.wafer_widget.chip_height} | 总芯片数: {len(self.wafer_widget.chips)}")
        info_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(info_label)

        # 右侧控制面板
        right_panel = QFrame()
        right_panel.setMinimumWidth(250)
        right_panel.setStyleSheet("""
            QFrame {
                background-color: white;
                border-radius: 8px;
                padding: 10px;
            }
        """)
        right_layout = QVBoxLayout(right_panel)

        # 标题
        title = QLabel("晶圆芯片")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #2c3e50; padding: 10px 0;")
        title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(title)

        # 添加分隔线
        right_layout.addWidget(self.create_h_line())

        # 操作按钮
        btn_print = QPushButton("选中芯片位置")
        btn_print.clicked.connect(self.print_selected_chips)
        btn_print.clicked.connect(self.emit_selected_chips)
        right_layout.addWidget(btn_print)

        btn_move_to_mark_position = QPushButton("移动至标记位置")
        btn_move_to_mark_position.clicked.connect(self.emit_move_to_mark_position)
        right_layout.addWidget(btn_move_to_mark_position)

        btn_clear = QPushButton("清除所有选择")
        btn_clear.clicked.connect(self.clear_selections)
        right_layout.addWidget(btn_clear)

        btn_reset = QPushButton("重置视图")
        btn_reset.setObjectName("resetButton")
        btn_reset.clicked.connect(self.reset_view)
        right_layout.addWidget(btn_reset)

        # 添加分隔线
        right_layout.addWidget(self.create_h_line())

        # 显示选项
        options_group = QGroupBox("显示选项")
        options_layout = QVBoxLayout()

        self.cb_axes = QCheckBox("显示坐标轴")
        self.cb_axes.setChecked(False)
        self.cb_axes.stateChanged.connect(self.toggle_axes)
        options_layout.addWidget(self.cb_axes)

        options_group.setLayout(options_layout)
        right_layout.addWidget(options_group)

        # 添加分隔线
        right_layout.addWidget(self.create_h_line())

        # 芯片信息显示
        info_group = QGroupBox("选中芯片信息")
        info_group_layout = QVBoxLayout()

        self.selected_info = QTextEdit()
        self.selected_info.setReadOnly(True)
        self.selected_info.setMaximumHeight(200)
        self.selected_info.setStyleSheet("""
            QTextEdit {
                border: 1px solid #ddd;
                border-radius: 4px;
                padding: 5px;
                font-family: monospace;
                background-color: #f9f9f9;
            }
        """)
        info_group_layout.addWidget(self.selected_info)

        info_group.setLayout(info_group_layout)
        right_layout.addWidget(info_group)

        # 使用说明
        help_group = QGroupBox("使用说明")
        help_layout = QVBoxLayout()

        help_text = QLabel()
        help_text.setText("""
        <b>操作指南:</b><br>
        • 左键点击选择芯片<br>
        • Ctrl+左键多选芯片<br>
        • 鼠标悬停显示芯片坐标和尺寸<br>
        • 鼠标滚轮缩放视图<br>
        • 中键/右键拖拽平移视图<br>
        • 点击按钮打印选中芯片<br>
        • 点击"重置视图"恢复默认<br>
        """)
        help_text.setWordWrap(True)
        help_layout.addWidget(help_text)

        help_group.setLayout(help_layout)
        right_layout.addWidget(help_group)

        # 添加弹性空间
        right_layout.addStretch()

        # 将左右面板添加到主布局
        main_layout.addWidget(left_panel, 4)
        main_layout.addWidget(right_panel, 1)

    def create_h_line(self):
        """创建水平分隔线"""
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("color: #ddd;")
        return line

    def print_selected_chips(self):
        """打印选中芯片的位置信息"""
        selected_chips = self.wafer_widget.get_selected_chip_positions()

        if not selected_chips:
            self.selected_info.setText("当前没有选中任何芯片")
            QMessageBox.information(self, "提示", "请先选择芯片！")
            return

        # 在文本框中显示选中芯片信息
        info_text = f"已选中 {len(selected_chips)} 个芯片:\n"
        info_text += "=" * 50 + "\n"

        for chip in selected_chips:
            info_text += f"芯片 {chip['index']}:\n"
            info_text += f"  中心坐标: ({chip['center_x']:.2f}, {chip['center_y']:.2f})\n"
            info_text += f"  芯片尺寸: {chip['width']:.2f} × {chip['height']:.2f}\n"
            info_text += "-" * 40 + "\n"

        self.selected_info.setText(info_text)

        # 同时在控制台打印
        # print("\n" + "=" * 60)
        # print("选中芯片位置信息:")
        # print("=" * 60)
        # for chip in selected_chips:
        #     print(
        #         f"芯片 {chip['index']}: 中心({chip['center_x']:.2f}, {chip['center_y']:.2f}), 尺寸: {chip['width']:.2f}×{chip['height']:.2f}")
        # print("=" * 60 + "\n")

        # 显示信息框
        # QMessageBox.information(self, "芯片信息", f"已选中 {len(selected_chips)} 个芯片，详细信息已显示在右侧面板。")

    def emit_selected_chips(self):
        selected_chips = self.wafer_widget.get_selected_chip_positions()
        self.sig_selected_points.emit(selected_chips)
        self.close()

    def emit_move_to_mark_position(self):
        selected_chips = self.wafer_widget.get_selected_chip_positions()

        if not selected_chips:
            self.selected_info.setText("当前没有选中任何芯片")
            QMessageBox.information(self, "提示", "请先选择芯片！")
            return

        center_x = selected_chips[0]['center_x']
        center_y = selected_chips[0]['center_y']

        center_x += self.wafer_mark_offset_x
        center_y += self.wafer_mark_offset_y

        mark_x = self.ega_param[0] + self.ega_param[2] * center_x - self.ega_param[4] * center_y
        mark_y = self.ega_param[1] + self.ega_param[3] * center_y + self.ega_param[5] * center_x

        self.sig_move_to_mark_position.emit([mark_x, mark_y])

    def clear_selections(self):
        """清除所有选择"""
        for chip in self.wafer_widget.chips:
            chip.is_selected = False
        self.wafer_widget.selected_chips.clear()
        self.wafer_widget.update()
        self.selected_info.setText("已清除所有选择")

        # 显示确认信息
        QMessageBox.information(self, "清除选择", "已清除所有芯片的选择状态。")

    def reset_view(self):
        """重置视图"""
        self.wafer_widget.reset_view()
        self.selected_info.setText("视图已重置到默认状态")

    def toggle_axes(self, state):
        """切换坐标轴显示"""
        self.wafer_widget.show_coordinates = (state == Qt.Checked)
        self.wafer_widget.update()

    def display_move_error(self, message_text: str):
        QMessageBox.information(self, "move error", f"{message_text}")


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


def compute_mark_positions(centers, a, b):
    """计算每个芯片的标记点位置"""
    offsets = np.array([
        [a, b],
        [a, -b],
        [-a, b],
        [-a, -b]
    ])
    marks = centers[:, None, :] + offsets[None, :, :]
    return marks.reshape(-1, 2)


def nominal_to_real(nominal, params):
    real_positions = []
    for (u, v) in nominal:
        x = params[0] + params[2] * u - params[4] * v
        y = params[1] + params[3] * v + params[5] * u
        real_positions.append((x, y))
    return np.array(real_positions)


def select_fitting_points(nominal_positions, points_number, wafer_radius):
    target_radius = wafer_radius * 0.6

    angles = np.linspace(0, 2 * np.pi, points_number, endpoint=False)
    fit_points = []
    for angle in angles:
        target_x = target_radius * np.cos(angle)
        target_y = target_radius * np.sin(angle)
        distances = np.sqrt(
            (nominal_positions[:, 0] - target_x) ** 2 +
            (nominal_positions[:, 1] - target_y) ** 2
        )
        closest_point_index = np.argmin(distances)
        fit_points.append(nominal_positions[closest_point_index])
    return np.array(fit_points)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion样式

    # 设置应用图标和字体
    app.setFont(QFont("Arial", 10))

    # 参数设置
    wafer_radius = 12 * 25.4 / 2 - 3  # 12英寸晶圆减去3mm边缘
    chip_width = 10  # 芯片宽度
    chip_height = 10  # 芯片高度
    spacing = 0.08  # 芯片间距
    ega_params = np.array([-54.2204, -186.045, 0.999983, 1.00003, 0.00280008, 0.002721])

    cells_layout = create_chip_position(wafer_radius, chip_width, chip_height, spacing)
    real_position = nominal_to_real(cells_layout, ega_params)
    cells_layout = np.hstack([cells_layout, real_position])

    # 创建主窗口
    viewer = WaferViewer(wafer_radius, cells_layout, chip_width, chip_height)
    viewer.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()