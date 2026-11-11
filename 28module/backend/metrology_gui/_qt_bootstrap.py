# -*- coding: utf-8 -*-
"""
Filename: _qt_bootstrap.py
Description:
    Qt 解释器/插件环境引导（唯一事实源）。

    迭代 41 起环境内置：项目自带 env/python（Python 3.11，含
    PyQt5/matplotlib/pandas/cv2），不再依赖外部测校 venv；旧 venv
    （3000测校SOP_u 下，目录在 sop_tools ↔ _tools 间被反复改名，
    2026-09-09 起）仅留作历史容错兜底。venv 路径含中文时 Qt 解析不出
    平台插件目录，直接弹 "no Qt platform plugin" 系统弹窗。本模块把
    两件事收口到一处：

    - find_venv_python()：候选路径逐个探测（项目 env/python → sop_tools
      → _tools → 3000测校SOP_u 下一级目录 glob 兜底），全部落空返回
      None（调用方据此给出中文提示，绝不裸奔到 Qt 弹窗）；
    - setup_qt_env()：在 import PyQt5 之后、QApplication 创建之前，
      强制 QT_QPA_PLATFORM_PLUGIN_PATH 指向当前解释器自带的
      PyQt5/Qt5/plugins/platforms（强制赋值而非 setdefault——继承到
      错误/为空的外部值时 setdefault 不生效，2026-09-09 实测）。
"""
import glob
import os

# 旧测校 venv 所在根（参考框架/venv 宿主编；迭代 41 起仅历史容错兜底）
SOP_BASE = r"C:\Users\e-hao.wu\Desktop\3000测校SOP_u"


def venv_python_candidates():
    """GUI 解释器候选路径（按优先级排序；迭代 41 起项目内置 env/python
    居首，旧外部 venv 仅历史容错——其目录名被反复改名，须候选回退）。"""
    return [
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "env", "python", "python.exe"),
        os.path.join(SOP_BASE, "sop_tools", ".venv", "Scripts", "python.exe"),
        os.path.join(SOP_BASE, "_tools", ".venv", "Scripts", "python.exe"),
    ]


def find_venv_python():
    """探测可用的 GUI 解释器（迭代 41 起首选项目内置 env/python）；
    全部落空返回 None。"""
    for path in venv_python_candidates():
        if os.path.exists(path):
            return path
    # 兜底：3000测校SOP_u 下任一级目录里的 .venv（目录再改名也能命中）
    hits = sorted(glob.glob(
        os.path.join(SOP_BASE, "*", ".venv", "Scripts", "python.exe")))
    return hits[0] if hits else None


def setup_qt_env():
    """强制 QT_QPA_PLATFORM_PLUGIN_PATH=本解释器 PyQt5 的 platforms 目录。
    须在 import PyQt5 之后、QApplication 创建之前调用。返回插件目录。

    迭代 34 加固：预检 qwindows.dll 存在性——缺失时 Qt 会在 QApplication
    创建时 qFatal 弹系统窗 "no Qt platform plugin"，这里提前给中文警告。"""
    import PyQt5
    plugins = os.path.join(os.path.dirname(PyQt5.__file__),
                           "Qt5", "plugins", "platforms")
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins
    if not os.path.isfile(os.path.join(plugins, "qwindows.dll")):
        print("[qt-bootstrap] 警告: platform 插件缺失（%s），"
              "QApplication 将弹 'no Qt platform plugin'——检查 venv 的 "
              "PyQt5 安装是否完整。" % plugins, flush=True)
    return plugins
