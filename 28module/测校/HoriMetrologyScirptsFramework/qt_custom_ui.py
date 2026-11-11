# -*- coding: utf-8 -*-
"""
Filename: qt_custom_ui.py
Author: Jianming.Liu
Date: 2025/9/29 
Description:
    定制化控件
"""
from PyQt5.QtCore import Qt, QAbstractTableModel, QModelIndex

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QCheckBox, QComboBox, QStackedWidget,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox, QTableView, QHeaderView,
    QAbstractSpinBox, QTableWidget, QTableWidgetItem, QProgressBar, QTextEdit, QSpacerItem, QSizePolicy
)

import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QTableWidget, QTableWidgetItem, QPushButton,
                             QHeaderView, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor


def create_spinbox(min_val, max_val, decimals=3):
    spinbox = QDoubleSpinBox()
    spinbox.setRange(min_val, max_val)
    spinbox.setDecimals(decimals)
    spinbox.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spinbox.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 6px 10px;
                background-color: white;
                min-width: 100px;
            }
            QDoubleSpinBox:hover {
                border-color: #4a90e2;
            }
            QDoubleSpinBox:focus {
                border-color: #4a90e2;
                outline: none;
            }
        """)
    return spinbox


def create_combobox(items):
    combobox = QComboBox()
    if len(items) > 0:
        combobox.addItems(items)
    combobox.setStyleSheet("""
            QComboBox {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 6px 10px;
                background-color: white;
                min-width: 150px;
            }
            QComboBox:hover {
                border-color: #4a90e2;
            }
            QComboBox::drop-down {
                border-left: 1px solid #d0d0d0;
                border-top-right-radius: 4px;
                border-bottom-right-radius: 4px;
            }
        """)
    return combobox


def create_label(text):
    """创建带样式的标签"""
    label = QLabel(text)
    label.setStyleSheet("font-weight: 350; color: #333;")
    return label


def create_groupbox(title, min_width=400):
    """创建带样式的分组框"""
    groupbox = QGroupBox(title)
    style_sheet = """
            QGroupBox {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 15px;
                min-width: %dpx;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #333;
            }
        """ % min_width
    groupbox.setStyleSheet(style_sheet)
    return groupbox


def create_lineedit(text=""):
    """创建带样式的文本输入框"""
    lineedit = QLineEdit(text)
    lineedit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                padding: 6px 10px;
                background-color: white;
                min-width: 60px;
            }
            QLineEdit:hover {
                border-color: #4a90e2;
            }
            QLineEdit:focus {
                border-color: #4a90e2;
                outline: none;
            }
        """)
    return lineedit


class StatisticsTableWidget(QWidget):
    def __init__(self, column_names=None, parent=None):
        super().__init__(parent)

        # 默认列名
        if column_names is None:
            column_names = ["名称", "数值1", "数值2", "数值3"]

        self.column_names = column_names
        self.data_start_row = 0  # 数据起始行
        self.stat_rows_count = 2  # 统计信息行数（平均值和标准差）

        self.init_ui()

    def init_ui(self):
        """初始化用户界面"""
        main_layout = QVBoxLayout(self)

        # 创建表格
        self.table = QTableWidget()
        self.table.setColumnCount(len(self.column_names))
        self.table.setHorizontalHeaderLabels(self.column_names)

        # 设置表格属性
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # 添加按钮布局 - 放在左下角
        button_layout = QHBoxLayout()

        self.delete_button = QPushButton("删除选中行")

        self.delete_button.clicked.connect(self.delete_selected_rows)

        button_layout.addWidget(self.delete_button)
        button_layout.addStretch()  # 将按钮推到左侧

        # 添加到主布局 - 先添加表格，再添加按钮，这样按钮就在下方
        main_layout.addWidget(self.table)
        main_layout.addLayout(button_layout)

        # 初始化表格结构
        self.update_table_structure()

    def update_table_structure(self):
        """更新表格结构，确保统计行在底部"""
        current_data_rows = max(0, self.table.rowCount() - self.stat_rows_count)

        # 设置总行数：数据行 + 统计行
        total_rows = current_data_rows + self.stat_rows_count
        self.table.setRowCount(total_rows)

        # 设置统计行标签
        if total_rows >= 2:
            # 平均值行
            avg_item = QTableWidgetItem("平均值")
            avg_item.setFlags(avg_item.flags() & ~Qt.ItemIsEditable)
            avg_item.setBackground(QBrush(QColor(240, 240, 240)))
            self.table.setItem(total_rows - 2, 0, avg_item)

            # 标准差行
            std_item = QTableWidgetItem("标准差")
            std_item.setFlags(std_item.flags() & ~Qt.ItemIsEditable)
            std_item.setBackground(QBrush(QColor(240, 240, 240)))
            self.table.setItem(total_rows - 1, 0, std_item)

        # 更新数据起始行
        self.data_start_row = 0

        # 计算统计信息
        self.calculate_statistics()

    def add_empty_row(self):
        """添加空行 - 用于按钮点击"""
        self.add_row()

    def add_row(self, data=None):
        """添加一行数据"""
        # 在统计行之前插入新行
        insert_position = max(0, self.table.rowCount() - self.stat_rows_count)
        self.table.insertRow(insert_position)

        # 如果提供了数据且是可迭代对象，填充该行
        if data is not None and hasattr(data, '__iter__') and not isinstance(data, (str, bytes)):
            try:
                self.set_row_data(insert_position, data)
            except Exception as e:
                print(f"设置行数据时出错: {e}")
                # 如果设置数据失败，添加空行
                for col in range(self.table.columnCount()):
                    item = QTableWidgetItem("")
                    self.table.setItem(insert_position, col, item)
        else:
            # 如果没有提供数据或数据不可迭代，添加空行
            for col in range(self.table.columnCount()):
                item = QTableWidgetItem("")
                self.table.setItem(insert_position, col, item)

        # 更新统计信息
        self.calculate_statistics()

    def delete_selected_rows(self):
        """删除选中的行"""
        selected_ranges = self.table.selectedRanges()

        if not selected_ranges:
            QMessageBox.warning(self, "警告", "请先选择要删除的行")
            return

        # 获取所有选中的行索引（排除统计行）
        rows_to_delete = set()
        for selected_range in selected_ranges:
            for row in range(selected_range.topRow(), selected_range.bottomRow() + 1):
                if row < self.table.rowCount() - self.stat_rows_count:  # 确保不是统计行
                    rows_to_delete.add(row)

        # 按降序删除行（避免索引变化问题）
        for row in sorted(rows_to_delete, reverse=True):
            self.table.removeRow(row)

        # 更新统计信息
        self.calculate_statistics()

    def set_row_data(self, row, data):
        """设置指定行的数据"""
        if row >= self.table.rowCount() - self.stat_rows_count:
            return False

        for col, value in enumerate(data):
            if col < self.table.columnCount():
                item = QTableWidgetItem(str(value))
                self.table.setItem(row, col, item)

        return True

    def calculate_statistics(self):
        """计算每列的统计信息（平均值和标准差）"""
        data_rows = self.table.rowCount() - self.stat_rows_count

        if data_rows <= 0:
            # 没有数据行，清空统计信息
            for col in range(1, self.table.columnCount()):
                if self.table.rowCount() > 1:
                    avg_item = QTableWidgetItem("")
                    avg_item.setFlags(avg_item.flags() & ~Qt.ItemIsEditable)
                    avg_item.setBackground(QBrush(QColor(240, 240, 240)))
                    self.table.setItem(self.table.rowCount() - 2, col, avg_item)

                    std_item = QTableWidgetItem("")
                    std_item.setFlags(std_item.flags() & ~Qt.ItemIsEditable)
                    std_item.setBackground(QBrush(QColor(240, 240, 240)))
                    self.table.setItem(self.table.rowCount() - 1, col, std_item)
            return

        # 计算每列的统计信息
        for col in range(1, self.table.columnCount()):
            column_data = []

            # 收集该列的所有数据
            for row in range(data_rows):
                item = self.table.item(row, col)
                if item is not None and item.text().strip():
                    try:
                        value = float(item.text())
                        column_data.append(value)
                    except ValueError:
                        continue

            # 计算统计信息
            if column_data:
                avg = np.mean(column_data)
                std = np.std(column_data, ddof=1)  # 样本标准差

                # 更新平均值行
                avg_item = QTableWidgetItem(f"{avg:.4f}")
                avg_item.setFlags(avg_item.flags() & ~Qt.ItemIsEditable)
                avg_item.setBackground(QBrush(QColor(240, 240, 240)))
                self.table.setItem(self.table.rowCount() - 2, col, avg_item)

                # 更新标准差行
                std_item = QTableWidgetItem(f"{std:.4f}")
                std_item.setFlags(std_item.flags() & ~Qt.ItemIsEditable)
                std_item.setBackground(QBrush(QColor(240, 240, 240)))
                self.table.setItem(self.table.rowCount() - 1, col, std_item)
            else:
                # 没有有效数据
                avg_item = QTableWidgetItem("")
                avg_item.setFlags(avg_item.flags() & ~Qt.ItemIsEditable)
                avg_item.setBackground(QBrush(QColor(240, 240, 240)))
                self.table.setItem(self.table.rowCount() - 2, col, avg_item)

                std_item = QTableWidgetItem("")
                std_item.setFlags(std_item.flags() & ~Qt.ItemIsEditable)
                std_item.setBackground(QBrush(QColor(240, 240, 240)))
                self.table.setItem(self.table.rowCount() - 1, col, std_item)

    # 对外接口方法
    def add_data_row(self, data):
        """
        对外接口：添加数据行
        :param data: 列表，包含每列的数据
        """
        if data is not None and hasattr(data, '__iter__') and not isinstance(data, (str, bytes)):
            self.add_row(data)
        else:
            print(f"错误: 传入的数据必须是可迭代对象，当前类型: {type(data)}")

    def get_all_data(self):
        """
        对外接口：获取所有数据
        :return: 二维列表，包含所有数据行的数据
        """
        data = []
        data_rows = self.table.rowCount() - self.stat_rows_count

        for row in range(data_rows):
            row_data = []
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item is not None:
                    row_data.append(item.text())
                else:
                    row_data.append("")
            data.append(row_data)

        return data

    def clear_all_data(self):
        """对外接口：清空所有数据"""
        data_rows = self.table.rowCount() - self.stat_rows_count
        for row in range(data_rows - 1, -1, -1):
            self.table.removeRow(row)

        self.calculate_statistics()


class TableDataModel(QAbstractTableModel):
    """
    自定义表格数据模型，继承自QAbstractTableModel。
    负责管理表格的底层数据，并通知视图更新。
    """

    def __init__(self, data=None, headers=None, column_types=None, column_validators=None, column_formats=None):
        super().__init__()
        # 初始化表头和数据
        self._headers = headers if headers else ['x', 'y', 'z', 'camera_ID', 'template_name']
        self._data = data if data is not None else []

        # 定义每列的数据类型和验证规则
        if column_types is not None:
            self.column_types = column_types
        else:
            self.column_types = {
                0: 'float',  # x列：浮点数
                1: 'float',  # y列：浮点数
                2: 'float',  # z列：浮点数
                3: 'int',  # camera_ID列：字符串
                4: 'string'  # template_name列：字符串
            }

        # 列验证器配置
        if column_validators is not None:
            self.column_validators = column_validators
        else:
            self.column_validators = {
                0: {'min': -250, 'max': 250},  # x的范围限制
                1: {'min': -450, 'max': 10},  # y的范围限制
                2: {'min': -1000.0, 'max': 1000.0},  # z的范围限制
                3: {'min': 0, 'max': 5},  # camera_ID最大长度
                4: {'max_length': 50}  # template_name最大长度
            }

        # 列显示格式
        if column_formats is not None:
            self.column_formats = column_formats
        else:
            self.column_formats = {
                0: '{:.3f}',  # x列：显示3位小数
                1: '{:.3f}',  # y列：显示3位小数
                2: '{:.3f}',  # z列：显示3位小数
                3: '{}',  # camera_ID：原样显示
                4: '{}'  # template_name：原样显示
            }

    def rowCount(self, parent=QModelIndex()):
        # 返回表格的行数
        return len(self._data)

    def columnCount(self, parent=QModelIndex()):
        # 返回表格的列数
        return len(self._headers)

    def setData(self, index, value, role=Qt.EditRole):
        """
        设置指定索引位置的数据 - 增强版（带类型验证）
        """
        if not index.isValid() or role != Qt.EditRole:
            return False

        row, col = index.row(), index.column()

        # 检查行列范围
        if 0 <= row < len(self._data) and 0 <= col < len(self._headers):
            # ========== 新增：数据类型验证 ==========
            validation_result = self.validate_cell_value(col, value)
            if not validation_result['valid']:
                # 验证失败，可以显示错误信息
                self.show_validation_error(validation_result['message'])
                return False

            # 根据数据类型转换值
            converted_value = self.convert_value_by_type(col, value)

            # 存储转换后的值
            self._data[row][col] = converted_value
            self.dataChanged.emit(index, index, [role])
            return True

        return False

    def validate_cell_value(self, column, value):
        """验证单元格值是否符合列的数据类型要求"""
        value_str = str(value).strip() if value is not None else ""

        # 获取列的数据类型
        col_type = self.column_types.get(column, 'string')
        validator = self.column_validators.get(column, {})

        # 空值检查（如果需要可以设为不允许空值）
        if value_str == "":
            return {'valid': True, 'message': ''}

        # 根据数据类型验证
        if col_type == 'float':
            try:
                num_value = float(value_str)
                # 范围检查
                if 'min' in validator and num_value < validator['min']:
                    return {'valid': False, 'message': f'值不能小于{validator["min"]}'}
                if 'max' in validator and num_value > validator['max']:
                    return {'valid': False, 'message': f'值不能大于{validator["max"]}'}
            except ValueError:
                return {'valid': False, 'message': '请输入有效的数字'}

        elif col_type == 'int':
            try:
                int_value = int(value_str)
                # 范围检查
                if 'min' in validator and int_value < validator['min']:
                    return {'valid': False, 'message': f'值不能小于{validator["min"]}'}
                if 'max' in validator and int_value > validator['max']:
                    return {'valid': False, 'message': f'值不能大于{validator["max"]}'}
            except ValueError:
                return {'valid': False, 'message': '请输入有效的整数'}

        elif col_type == 'string':
            # 字符串长度检查
            if 'max_length' in validator and len(value_str) > validator['max_length']:
                return {'valid': False,
                        'message': f'字符串长度不能超过{validator["max_length"]}个字符'}

        return {'valid': True, 'message': ''}

    def convert_value_by_type(self, column, value):
        """根据列的数据类型转换值"""
        if value is None:
            return ""

        col_type = self.column_types.get(column, 'string')
        value_str = str(value).strip()

        if value_str == "":
            return ""

        try:
            if col_type == 'float':
                return float(value_str)
            elif col_type == 'int':
                return int(value_str)
            else:  # string 或其他类型
                return value_str
        except ValueError:
            # 转换失败时返回原始字符串
            return value_str

    def show_validation_error(self, message):
        """显示验证错误信息"""
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.warning(None, "输入错误", message)

    def data(self, index, role=Qt.DisplayRole):
        """
        返回指定索引位置的数据 - 增强版（格式化显示）
        """
        if not index.isValid():
            return None

        row, col = index.row(), index.column()

        if 0 <= row < len(self._data) and 0 <= col < len(self._headers):
            cell_value = self._data[row][col]

            # 显示角色：格式化输出
            if role == Qt.DisplayRole:
                if cell_value is None or cell_value == "":
                    return ""

                # 获取格式化字符串
                fmt = self.column_formats.get(col, '{}')

                # 应用格式化
                try:
                    return fmt.format(cell_value)
                except:
                    return str(cell_value)

            # 编辑角色：返回原始值用于编辑
            elif role == Qt.EditRole:
                return str(cell_value) if cell_value is not None else ""

            # 对齐角色：设置不同类型的不同对齐方式
            elif role == Qt.TextAlignmentRole:
                if self.column_types.get(col, 'string') in ('float', 'int'):
                    return Qt.AlignRight | Qt.AlignVCenter
                else:
                    return Qt.AlignLeft | Qt.AlignVCenter

        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        # 返回表头数据
        if role != Qt.DisplayRole:
            return None
        if orientation == Qt.Horizontal and 0 <= section < len(self._headers):
            return self._headers[section]
        return None

    def flags(self, index):
        # 设置单元格标志，使表格可编辑
        return super().flags(index) | Qt.ItemIsEditable

    def insertRows(self, position, rows=1, parent=QModelIndex()):
        # 在指定位置插入新行（空数据）
        if position < 0 or position > len(self._data):
            return False
        # 必须调用begin/end通知视图进行更新[citation:1]
        self.beginInsertRows(parent, position, position + rows - 1)
        for _ in range(rows):
            # 插入一行空数据
            self._data.insert(position, [''] * self.columnCount())
        self.endInsertRows()
        return True

    def removeRows(self, position, rows=1, parent=QModelIndex()):
        # 从指定位置开始删除指定行数的数据
        if position < 0 or position + rows > len(self._data):
            return False
        # 必须调用begin/end通知视图进行更新[citation:1]
        self.beginRemoveRows(parent, position, position + rows - 1)
        del self._data[position:position + rows]
        self.endRemoveRows()
        return True

    def set_all_data(self, data):
        """
        设置完整表格数据的接口
        参数：
            data: list of list，二维列表数据，每行是一个列表，长度应与列数匹配
        """
        # 验证数据格式
        if data and len(data[0]) != self.columnCount():
            raise ValueError(f"数据列数({len(data[0])})与表格列数({self.columnCount()})不匹配")

        self.beginResetModel()
        self._data = data.copy() if data else []
        self.endResetModel()

    def get_all_data(self):
        # 获取所有表格数据
        return self._data.copy()

    def load_data(self, new_data):
        # 加载新数据并重置模型
        self.beginResetModel()
        self._data = new_data
        self.endResetModel()


class CustomTableWidget(QWidget):
    def __init__(self, headers=None, column_types=None, column_validators=None, column_formats=None ):
        super().__init__()
        self.init_ui(headers, column_types, column_validators, column_formats)

    def init_ui(self, headers=None, column_types=None, column_validators=None, column_formats=None):
        # 创建中心部件和主布局
        main_layout = QVBoxLayout(self)

        # 创建表格视图和数据模型
        self.table_view = QTableView()
        self.model = TableDataModel(None, headers, column_types, column_validators, column_formats)  # 使用自定义模型
        self.table_view.setModel(self.model)

        # 设置表格选择行为：选中整行
        self.table_view.setSelectionBehavior(QTableView.SelectRows)
        # 设置列宽自适应内容
        self.table_view.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        main_layout.addWidget(self.table_view)

        # 创建按钮布局
        button_layout = QHBoxLayout()

        # 添加行按钮
        self.add_button = QPushButton("添加行")
        self.add_button.clicked.connect(self.add_row)
        button_layout.addWidget(self.add_button)

        # 删除行按钮
        self.delete_button = QPushButton("删除选中行")
        self.delete_button.clicked.connect(self.delete_rows)
        button_layout.addWidget(self.delete_button)

        # 清空按钮
        self.clear_button = QPushButton("清空表格")
        self.clear_button.clicked.connect(self.clear_table)
        button_layout.addWidget(self.clear_button)

        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        self.configure_table_style()

    def configure_table_style(self):
        """配置表格的视觉样式"""
        self.table_view.setShowGrid(True)

        style_sheet = """
            QTableView {
                border: 1px solid #c0c0c0; /* 表格外边框 */
                gridline-color: #d0d0d0;   /* 内部网格线颜色*/
                background-color: white;
            }
            QTableView::item:selected {
                background-color: #a0c6e9; /* 选中行的背景色 */
                color: black;
            }
            QHeaderView::section {
                background-color: #f0f0f0;
                border: 1px solid #c0c0c0;
                padding: 4px;
            }
        """
        self.table_view.setStyleSheet(style_sheet)

    def set_table_data(self, data):
        """
        设置表格数据的公共接口
        参数：
            data: list of list，二维列表数据
        """
        try:
            self.model.set_all_data(data)
        except ValueError as e:
            QMessageBox.warning(self, "数据格式错误", str(e))
            raise

    def get_all_data(self):
        return self.model.get_all_data()

    def add_row(self):
        """在表格末尾添加一个新行"""
        row_count = self.model.rowCount()
        self.model.insertRows(row_count)

    def delete_rows(self):
        """删除所有选中的行"""
        selection_model = self.table_view.selectionModel()
        if not selection_model.hasSelection():
            QMessageBox.warning(self, "警告", "请先选择要删除的行！")
            return

        selected_rows = selection_model.selectedRows()
        if not selected_rows:
            return

        # 从后向前删除，避免索引变化导致的问题
        rows_to_delete = sorted([index.row() for index in selected_rows], reverse=True)
        for row in rows_to_delete:
            self.model.removeRows(row)

    def clear_table(self):
        """清空表格中的所有数据"""
        reply = QMessageBox.question(
            self, "确认清空",
            "确定要清空表格中的所有数据吗？此操作不可撤销。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.model.load_data([])


def test_table_widget():
    app = QApplication(sys.argv)

    table_widget = CustomTableWidget(['x', 'y', 'z'])
    # # 创建表格控件，指定列名
    # table_widget = StatisticsTableWidget(["名称", "数值1", "数值2"])
    #
    # # # 添加一些示例数据
    # table_widget.add_data_row(["A", 10, 20])
    # table_widget.add_data_row(["B", 15, 25])
    # table_widget.add_data_row(["C", 12, 22])

    # 显示窗口
    window = QWidget()
    layout = QVBoxLayout(window)
    layout.addWidget(table_widget)
    window.setWindowTitle("带统计功能的表格")
    window.resize(600, 400)
    window.show()

    sys.exit(app.exec_())


# 示例使用
if __name__ == "__main__":
    test_table_widget()
