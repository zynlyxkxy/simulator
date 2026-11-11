# -*- coding: utf-8 -*-
"""
Filename: taught_points_table.py
Description:
    示教点表格行编辑器（迭代 32 新；迭代 35 扩展为"每点两个位姿"）。
    旋转类测校（WsRotateRepeat/WsRotateCenterCalib/WsRotateGainCalib）
    共用：旋转前先走遍各点 pose（旋前）求世界坐标 P_k，旋转后走遍
    pose_after（旋后）求 Q_k，后端按 find_rotation_center 口径拟合
    θ 与中心——每点一行扩为 8 坐标列 + 模板列，两个取位按钮各自回填
    对应四格（→旋前 / →旋后）。组标题"旋前"/"旋后"作视觉分组，
    列宽紧凑防过宽。

    对外接口（迭代 35 起；逻辑层 _read_pose/_snap_pose/get/set_saved_param
    按 *_aft 后缀区分两个位姿）：
      point_groups → [{"tpl": QLineEdit,
                       "spins": {attr: QDoubleSpinBox},   # 8 键：
                           # as_x/as_y/ws_x/ws_y=旋前，
                           # as_x_aft/as_y_aft/ws_x_aft/ws_y_aft=旋后
                       "btn_snap_before": QPushButton,    # →旋前
                       "btn_snap_after": QPushButton,     # →旋后
                       "widgets": [本行全部控件（重建销毁用）]}]
      rebuild(n)：行数调整为 n，已填值按下标保留（同旧 rebuild_points）。

    向后兼容：读旧存档（无 pose_after）时由逻辑层把旋前值填进旋后四格。
"""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QLabel, QPushButton, QGridLayout, QVBoxLayout, QScrollArea)

from qt_custom_ui import create_lineedit, create_spinbox

# (组标题, 组底色, 组前景色, [(表头, spins attr), ...])
# attr 与逻辑层 _AXIS_OF_ATTR / *_aft 映射键一致
POINT_GROUPS = (
    ("旋前", "#eaf3fd", "#357abd",
     (("AS_X", "as_x"), ("AS_Y", "as_y"),
      ("WS_X", "ws_x"), ("WS_Y", "ws_y"))),
    ("旋后", "#fdf3e7", "#e67e22",
     (("AS_X", "as_x_aft"), ("AS_Y", "as_y_aft"),
      ("WS_X", "ws_x_aft"), ("WS_Y", "ws_y_aft"))),
)

_SPIN_W = 54        # 轴 spinbox 等宽（紧凑，8 列防过宽）
_TPL_MIN_W = 60     # 模板名列最小宽（该列拉伸吃余量）
_SEP_W = 8          # 旋前/旋后组间隔

_SNAP_QSS = """
    QPushButton {
        background-color: white;
        color: #357abd;
        border: 1px solid #4a90e2;
        border-radius: 4px;
        padding: 3px 5px;
        font-weight: bold;
    }
    QPushButton:hover { background-color: #eaf3fd; }
"""

_COMPACT_SPIN_QSS = """
    QDoubleSpinBox {
        border: 1px solid #d0d0d0;
        border-radius: 4px;
        padding: 3px 4px;
        background-color: white;
    }
    QDoubleSpinBox:hover { border-color: #4a90e2; }
    QDoubleSpinBox:focus { border-color: #4a90e2; }
"""


def _compact_spin():
    """行内轴位 spinbox：紧凑等宽（覆盖 create_spinbox 的 100px min-width）。"""
    sb = create_spinbox(-250, 250, 4)
    sb.setStyleSheet(_COMPACT_SPIN_QSS)
    sb.setFixedWidth(_SPIN_W)
    return sb


def _group_head(text, bg, fg):
    """组标题（旋前/旋后）：底色带 + 组色文字，跨本组全部列。"""
    lab = QLabel(text)
    lab.setAlignment(Qt.AlignCenter)
    lab.setStyleSheet(
        "font-weight: bold; color: %s; background-color: %s;"
        " border-radius: 3px; padding: 1px 2px;" % (fg, bg))
    return lab


class TaughtPointsTable(QWidget):
    """示教点表格（自带 QScrollArea：超高才纵向滚动，宽度随内容）。

    列布局：# | 模板名 | 旋前 AS_X AS_Y WS_X WS_Y |→旋前| 间隔 |
    旋后 AS_X AS_Y WS_X WS_Y |→旋后|；组标题/子表头跨列成视觉分组。
    """

    def __init__(self, parent=None, scroll_h=330):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.scroll = QScrollArea(self)
        self.scroll.setWidgetResizable(True)
        self.scroll.setMaximumHeight(scroll_h)
        self.scroll.setStyleSheet(
            "QScrollArea { border: none; background: transparent; }")
        self.container = QWidget()
        self.container.setStyleSheet("background: transparent;")
        self.grid = QGridLayout(self.container)
        self.grid.setContentsMargins(2, 2, 2, 2)
        self.grid.setHorizontalSpacing(4)
        self.grid.setVerticalSpacing(4)
        # ---- 表头两行：# / 模板名 纵跨两行；旋前/旋后组标题横跨本组 ----
        head = QLabel("#")
        head.setStyleSheet("font-weight: bold; color: #357abd;")
        head.setAlignment(Qt.AlignCenter)
        self.grid.addWidget(head, 0, 0, 2, 1)
        head = QLabel("模板名")
        head.setStyleSheet("font-weight: bold; color: #357abd;")
        head.setAlignment(Qt.AlignCenter)
        self.grid.addWidget(head, 0, 1, 2, 1)
        col = 2
        for gi, (title, bg, fg, cols) in enumerate(POINT_GROUPS):
            n = len(cols)
            self.grid.addWidget(_group_head(title, bg, fg),
                                0, col, 1, n + 1)   # +1：含组内取位按钮列
            for j, (sub, _attr) in enumerate(cols):
                lab = QLabel(sub)
                lab.setAlignment(Qt.AlignCenter)
                lab.setStyleSheet(
                    "font-weight: bold; color: %s; background-color: %s;"
                    " border-radius: 3px;" % (fg, bg))
                self.grid.addWidget(lab, 1, col + j)
            col += n + 1                   # 跳过本组取位按钮列
            if gi < len(POINT_GROUPS) - 1:
                sep = QLabel("")
                sep.setFixedWidth(_SEP_W)
                self.grid.addWidget(sep, 0, col, 2, 1)
                col += 1
        self.grid.setColumnStretch(1, 1)      # 模板名列吃横向余量
        self.scroll.setWidget(self.container)
        lay.addWidget(self.scroll)
        self.point_groups = []
        self._stretch_row = 2

    def rebuild(self, n):
        """行数调整为 n（已填值按下标保留，同旧 rebuild_points 语义）。"""
        old = [{"tpl": g["tpl"].text(),
                "spins": {a: sb.value() for a, sb in g["spins"].items()}}
               for g in self.point_groups]
        for g in self.point_groups:
            for w in g["widgets"]:
                self.grid.removeWidget(w)
                # 先 hide 再 deleteLater：offscreen 下 processEvents 不一定
                # 及时处理 DeferredDelete，未销毁的旧行仍按旧几何参与绘制
                # （截图残影/遮挡表头），hide 立即退出绘制
                w.hide()
                w.deleteLater()
        self.grid.setRowStretch(self._stretch_row, 0)
        self.point_groups = []
        for i in range(n):
            r = i + 2                      # 表头占前两行
            idx = QLabel(str(r - 1))
            idx.setAlignment(Qt.AlignCenter)
            idx.setStyleSheet("font-weight: bold; color: #333;")
            tpl = create_lineedit("", "必填：已注册模板")
            tpl.setMinimumWidth(_TPL_MIN_W)
            spins = {}
            for _t, _bg, _fg, cols in POINT_GROUPS:
                for _head, attr in cols:
                    spins[attr] = _compact_spin()
            btn_b = QPushButton("→旋前")
            btn_a = QPushButton("→旋后")
            btn_b.setStyleSheet(_SNAP_QSS)
            btn_a.setStyleSheet(_SNAP_QSS)
            # 行控件与表头同列对齐：# | 模板名 | 旋前4轴 →旋前 |
            # 间隔 | 旋后4轴 →旋后
            widgets = [idx, tpl]
            for gi, (_t, _bg, _fg, cols) in enumerate(POINT_GROUPS):
                for _head, attr in cols:
                    widgets.append(spins[attr])
                widgets.append(btn_b if gi == 0 else btn_a)
                if gi < len(POINT_GROUPS) - 1:
                    sep = QLabel("")
                    sep.setFixedWidth(_SEP_W)
                    widgets.append(sep)
            for c, w in enumerate(widgets):
                self.grid.addWidget(w, r, c)
            if i < len(old):
                tpl.setText(old[i]["tpl"])
                for a, v in old[i]["spins"].items():
                    spins[a].setValue(v)
            self.point_groups.append(
                {"tpl": tpl, "spins": spins,
                 "btn_snap_before": btn_b,
                 "btn_snap_after": btn_a,
                 "widgets": widgets})
        # 末行吃垂直余量：行数少时行贴顶部，不留空洞
        self._stretch_row = n + 2
        self.grid.setRowStretch(self._stretch_row, 1)
