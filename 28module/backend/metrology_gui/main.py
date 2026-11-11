# -*- coding: utf-8 -*-
"""
Filename: main.py
Description:
    D2W 键合设备测校软件主窗口：左侧测校项列表 QListWidget +
    右侧 QStackedWidget，选中即实例化对应 ScriptWidget 嵌入。
    （参考框架无自带列表主窗口，此为本软件自建，样式与整体 QSS 统一。）

    运行：python main.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _ensure_qt_interpreter():
    """直接双击 main.py 会落到系统 Python（C:\\Python311，无 PyQt5 或插件
    路径解析失败弹 "no Qt platform plugin"）——导入失败时自重定到
    测校 venv 解释器再跑（防循环用环境变量哨兵）。候选路径/兜底 glob
    统一走 _qt_bootstrap.find_venv_python（唯一事实源）。"""
    try:
        import PyQt5  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("D2W_MET_GUI_REEXEC"):
        raise                       # 已经重定过还失败，不再循环
    from _qt_bootstrap import find_venv_python
    venv_py = find_venv_python()
    if venv_py and os.path.realpath(
            sys.executable) != os.path.realpath(venv_py):
        os.environ["D2W_MET_GUI_REEXEC"] = "1"
        os.execv(venv_py, [venv_py, os.path.abspath(__file__)]
                 + sys.argv[1:])
    raise


_ensure_qt_interpreter()

# venv 路径含中文时 Qt 解析不出插件目录（QLibraryInfo 返回空），
# 显式指定 platform 插件路径，须在 QApplication 创建前完成（_qt_bootstrap
# 内部用强制赋值而非 setdefault——继承到错误/为空的外部
# QT_QPA_PLATFORM_PLUGIN_PATH 时 setdefault 不生效，2026-09-09 实测）。
from _qt_bootstrap import setup_qt_env  # noqa: E402
_PLUGINS = setup_qt_env()
if not os.path.isfile(os.path.join(_PLUGINS, "qwindows.dll")):
    # 迭代 34 预检：宁可中文报错退出，不让 Qt qFatal 弹系统窗
    # "no Qt platform plugin"（无排查信息）。
    sys.stderr.write(
        "[错误] Qt platform 插件缺失：%s\\qwindows.dll 不存在。\n"
        "[错误] 测校 venv（3000测校SOP_u 下 .venv）的 PyQt5 安装不完整，\n"
        "[错误] 请重装 PyQt5 或用完整 venv 后重试。\n" % _PLUGINS)
    sys.exit(2)

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QListWidget, QStackedWidget,
    QHBoxLayout, QListWidgetItem, QLabel
)

from SingleCameraAlignStability import SingleCameraAlignStability
from AlignStabilityDynamic import AlignStabilityDynamic
from ConfocalRepeat import ConfocalRepeat
from StageMoveRepeat import StageMoveRepeat
from DieTransferRepeat import DieTransferRepeat
from WsRotateRepeat import WsRotateRepeat
from WsRotateCenterCalib import WsRotateCenterCalib
from WsRotateGainCalib import WsRotateGainCalib
from AsStageNonortho import AsStageNonortho
from WsStageNonortho import WsStageNonortho
from CameraMicrorotCalib import CameraMicrorotCalib
from AsHeadCenterCalib import AsHeadCenterCalib
from BestFocusCalib import BestFocusCalib
from BsRxryAxisCalib import BsRxryAxisCalib
from FlipHandoffHeightCalib import FlipHandoffHeightCalib
from BondHeightCalib import BondHeightCalib
from BondTempCalib import BondTempCalib
from BondForceCalib import BondForceCalib

SCRIPTS = [
    ("相机对准重复性·静态", SingleCameraAlignStability),
    ("相机对准重复性·动态", AlignStabilityDynamic),
    ("光谱共焦重复性", ConfocalRepeat),
    ("运动台移动重复性", StageMoveRepeat),
    ("芯片交接重复性", DieTransferRepeat),
    ("硅片台旋转重复性", WsRotateRepeat),
    ("硅片台旋转中心标定", WsRotateCenterCalib),
    ("旋转角度校准", WsRotateGainCalib),
    ("对准台非正交测校", AsStageNonortho),
    ("硅片台非正交测校", WsStageNonortho),
    ("相机微小旋转角标定", CameraMicrorotCalib),
    ("上吸盘位置标定", AsHeadCenterCalib),
    ("相机最佳焦平面标定", BestFocusCalib),
    ("键合头Rx/Ry旋转轴标定", BsRxryAxisCalib),
    ("Flip与键合头承接高度标定", FlipHandoffHeightCalib),
    ("键合头键合高度标定", BondHeightCalib),
    ("键合温度校准", BondTempCalib),
    ("键合力校准", BondForceCalib),
]

LIST_QSS = """
    QListWidget {
        background-color: white;
        border: 1px solid #e0e0e0;
        border-radius: 8px;
        padding: 6px;
        font-size: 10pt;
        outline: none;
    }
    QListWidget::item {
        padding: 10px 12px;
        border-radius: 5px;
        color: #333;
    }
    QListWidget::item:hover {
        background-color: #eaf3fd;
    }
    QListWidget::item:selected {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
        color: white;
        font-weight: bold;
    }
"""


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("D2W 键合设备测校软件")
        self.resize(1350, 860)
        self.setFont(QFont("Microsoft YaHei", 9))

        central = QWidget()
        central.setStyleSheet("background-color: #f5f7fa;")
        layout = QHBoxLayout(central)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        # 左侧：标题 + 测校项列表
        from PyQt5.QtWidgets import QVBoxLayout
        left = QWidget()
        left_vbox = QVBoxLayout(left)
        left_vbox.setContentsMargins(0, 0, 0, 0)
        left_vbox.setSpacing(8)
        title = QLabel("测校项")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 12pt;
            font-weight: bold;
            padding: 8px;
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4a90e2, stop:1 #357abd);
            color: white;
            border-radius: 5px;
        """)
        left_vbox.addWidget(title)
        self.script_list = QListWidget()
        self.script_list.setFixedWidth(240)
        self.script_list.setStyleSheet(LIST_QSS)
        for name, _cls in SCRIPTS:
            QListWidgetItem(name, self.script_list)
        left_vbox.addWidget(self.script_list)
        layout.addWidget(left)

        # 右侧：测校页栈（懒实例化）
        self.stack = QStackedWidget()
        self._pages = {}
        layout.addWidget(self.stack, 1)

        self.script_list.currentRowChanged.connect(self._on_select)
        self.setCentralWidget(central)
        if self.script_list.count() > 0:
            self.script_list.setCurrentRow(0)

    def _on_select(self, row):
        if row < 0:
            return
        if row not in self._pages:
            _name, cls = SCRIPTS[row]
            page = cls()
            self._pages[row] = page
            self.stack.addWidget(page)
        self.stack.setCurrentWidget(self._pages[row])


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
