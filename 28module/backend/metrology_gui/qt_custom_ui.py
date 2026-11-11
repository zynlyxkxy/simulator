# -*- coding: utf-8 -*-
"""
Filename: qt_custom_ui.py
Description:
    定制化控件工厂 + QSS（蓝标题条风格，复刻参考框架
    HoriMetrologyScirptsFramework/qt_custom_ui.py 的控件与样式）。
"""
from PyQt5.QtCore import Qt, QObject, QEvent, QTimer
from PyQt5.QtGui import QFont, QValidator
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QComboBox, QPushButton,
    QVBoxLayout, QGridLayout, QGroupBox, QDoubleSpinBox, QAbstractSpinBox,
    QTextEdit, QSpacerItem, QSizePolicy
)
import sys

# 全角 → 半角归一表（中文 IME 残留：全角数字/句点/句号/正负号/空格）
_FULLWIDTH_MAP = {ord(c): str(i) for i, c in
                  enumerate("０１２３４５６７８９")}
_FULLWIDTH_MAP.update({ord("．"): ".", ord("。"): ".", ord("·"): ".",
                       ord("－"): "-", ord("＋"): "+", ord("　"): " "})


def _normalize_num_text(text):
    """全角数字/句点（IME 残留）归一为半角，去首尾空白。"""
    return (text or "").translate(_FULLWIDTH_MAP).strip()


class _SelectAllOnFocus(QObject):
    """focusIn 时全选（singleShot 延后到鼠标事件处理完，避免点击
    定位光标把选区冲掉）——不清空直接敲=整体替换旧值。"""

    def eventFilter(self, obj, event):
        if event.type() == QEvent.FocusIn:
            QTimer.singleShot(0, obj.selectAll)
        return False


class MetrologyDoubleSpinBox(QDoubleSpinBox):
    """测校专用 QDoubleSpinBox——弹回根治（迭代 31）。

    根因（用户实测"角度改了弹回 1°"，逐键复现确认）：默认
    keyboardTracking=True + CorrectToPreviousValue——不清空直接敲
    （中间态与旧文本拼接）、越界、低于下限、IME 全角句点四类输入
    被静默丢键并弹回上次有效值。

    修复（工厂统一生效，全部测校项受益）：
    - setKeyboardTracking(False)：敲键过程不做中间值纠偏，失焦/
      回车才提交；
    - focusIn 事件过滤器全选：直接敲=整体替换旧值；
    - validate/fixup 重写 + CorrectToNearestValue：越界/低于下限
      提交时夹取到边界（Qt 原生对 ">max" 仍弹回，validate 把越界
      判 Intermediate 交给夹取，fixup 兜底）；全角数字/句点归一后
      参与解析；不可解析文本回退当前值（不跳到边界）。
    - 迭代 34：超精度文本（小数位 > decimals）判 Intermediate 逐键
      放行，提交时由 fixup 四舍五入到 decimals。根治"变回默认"：
      Qt 私有 validateAndInterpret 对超精度判 Invalid 并返回 range
      最小值，用户看到弹到下限（-250mm/0.0001rad）。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setKeyboardTracking(False)
        self.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
        # 事件过滤器须自持引用防 GC
        self._focus_all_filter = _SelectAllOnFocus(self)
        self.installEventFilter(self._focus_all_filter)

    def validate(self, text, pos):
        t = _normalize_num_text(text)
        if t in ("", "+", "-", ".", "+.", "-."):
            return (QValidator.Intermediate, t, len(t))
        try:
            v = float(t)
        except ValueError:
            return (QValidator.Invalid, t, len(t))
        mant = t.split("e")[0].split("E")[0]          # 去指数部分再数小数位
        if "." in mant and len(mant.split(".", 1)[1]) > self.decimals():
            # 超精度：逐键放行（Intermediate 不丢键），提交时由 fixup
            # 夹取并四舍五入到 decimals（与越界同一条纠偏路径）
            return (QValidator.Intermediate, t, len(t))
        if v < self.minimum() or v > self.maximum():
            # 越界不判 Invalid 丢键：Intermediate 允许敲入，提交时由
            # CorrectToNearestValue 夹取到边界（低于下限同口径）
            return (QValidator.Intermediate, t, len(t))
        return (QValidator.Acceptable, t, len(t))

    def fixup(self, text):
        t = _normalize_num_text(text)
        try:
            v = float(t)
        except ValueError:
            # 不可解析 → 回退当前有效值（而非跳到边界）
            return self.textFromValue(self.value())
        return self.textFromValue(min(max(v, self.minimum()),
                                      self.maximum()))


def create_spinbox(min_val, max_val, decimals=3):
    spinbox = MetrologyDoubleSpinBox()
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


def create_lineedit(text="", placeholder=""):
    """创建带样式的文本输入框"""
    lineedit = QLineEdit(text)
    if placeholder:
        lineedit.setPlaceholderText(placeholder)
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


def create_title_label(text):
    """蓝色渐变标题条（参考框架各测校页顶部样式）。"""
    label = QLabel(text)
    label.setAlignment(Qt.AlignCenter)
    label.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
            padding: 10px;
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
            color: white;
            border-radius: 5px;
        """)
    return label


def create_log_group(height=160):
    """Operation Log 分组框（参考框架底部日志区样式）。
    返回 (group, textEdit)。"""
    log_group = QGroupBox("Operation Log")
    log_group.setFixedHeight(height)
    log_group.setStyleSheet("""
            QGroupBox {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #333;
            }
        """)
    log_layout = QVBoxLayout()
    text_edit = QTextEdit()
    text_edit.setReadOnly(True)
    text_edit.setFixedHeight(height - 50)
    text_edit.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 8px;
                font-family: Consolas, 'Courier New', monospace;
                font-size: 9pt;
            }
        """)
    log_layout.addWidget(text_edit)
    log_group.setLayout(log_layout)
    return log_group, text_edit


def create_result_group(min_width=550):
    """Result Display 分组框（深色结果展示区，嵌入 matplotlib canvas）。
    返回 (group, res_display)。"""
    group = QGroupBox("Result Display")
    group.setStyleSheet("""
            QGroupBox {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
                color: #333;
            }
        """)
    layout = QVBoxLayout()
    res_display = QWidget()
    res_display.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #2c3e50, stop:1 #34495e);
            border: 1px solid #34495e;
            border-radius: 6px;
        """)
    res_display.setMinimumSize(min_width, 450)
    layout.addWidget(res_display)
    group.setLayout(layout)
    return group, res_display


def create_run_button(text="开始"):
    btn = QPushButton(text)
    btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 18px;
                font-weight: bold;
            }
            QPushButton:hover { background: #357abd; }
            QPushButton:disabled { background: #b0c4de; }
        """)
    return btn


def create_stop_button(text="中断"):
    btn = QPushButton(text)
    btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #c0392b;
                border: 1px solid #c0392b;
                border-radius: 4px;
                padding: 8px 18px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #fdedec; }
            QPushButton:disabled { color: #d0d0d0; border-color: #d0d0d0; }
        """)
    return btn


def apply_global_style(widget):
    widget.setStyleSheet("background-color: #f5f7fa;")
    widget.setFont(QFont("Microsoft YaHei", 9))


def create_stats_table(headers, rows=1, height=110):
    """统计表（结果区内嵌，蓝标题条同款配色的浅色表）。
    返回 QTableWidget：行数固定 rows，列头 headers，单元格只读。"""
    from PyQt5.QtWidgets import QTableWidget, QHeaderView, QAbstractItemView
    table = QTableWidget(rows, len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.verticalHeader().setVisible(False)
    table.setEditTriggers(QAbstractItemView.NoEditTriggers)
    table.setSelectionMode(QAbstractItemView.NoSelection)
    table.setFixedHeight(height)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.setStyleSheet("""
            QTableWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                gridline-color: #e8e8e8;
                font-size: 9pt;
            }
            QHeaderView::section {
                background-color: #eaf3fd;
                color: #357abd;
                font-weight: bold;
                border: none;
                border-bottom: 1px solid #d0d0d0;
                padding: 4px;
            }
        """)
    return table


def fill_stats_row(table, row, values):
    """把一行统计值写进统计表（数值转字符串，None 显示 —）。"""
    from PyQt5.QtWidgets import QTableWidgetItem
    from PyQt5.QtCore import Qt as _Qt
    for col, v in enumerate(values):
        text = "—" if v is None else (v if isinstance(v, str) else str(v))
        item = QTableWidgetItem(text)
        item.setTextAlignment(_Qt.AlignCenter)
        table.setItem(row, col, item)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = QWidget()
    apply_global_style(window)
    layout = QVBoxLayout(window)
    layout.addWidget(create_title_label("qt_custom_ui demo"))
    layout.addWidget(create_groupbox("Input Parameters"))
    window.resize(600, 300)
    window.show()
    sys.exit(app.exec_())
