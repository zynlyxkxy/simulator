# -*- coding: utf-8 -*-
"""
Filename: script_widget.py
Description:
    测校脚本基类 ScriptWidget（对照参考框架 ScriptWidget.py）。

    与参考框架的差异（刻意改动）：
    - 不做 MQTT 状态回传宿主（无 C++ 宿主，去掉 managerClient/uuid/topic
      以及 CMD_Display/CMD_Run/CMD_Finish 等回传报文）；
    - 所有 UI 更新必须走信号：worker 线程内只允许 log()/set_progress()/
      set_realtime_data()，真正的界面刷新在 GUI 线程的槽函数里做
      （修正参考框架 worker 直接 append textEdit / draw canvas 的写法）；
    - run() 的入参在 begin_() 时于 GUI 线程读取（get_saved_param），
      worker 线程内不再读控件值。
"""
import copy
import inspect
import os
import threading
from abc import abstractmethod
from datetime import datetime

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget


class ScriptWidget(QWidget):
    sig_log = pyqtSignal(str)              # 日志（GUI 线程落日志框）
    sig_progress = pyqtSignal(int, int)    # 进度 (当前, 总数)
    sig_finish = pyqtSignal(dict)          # 正常结束，携带输出 dict
    sig_error = pyqtSignal(str)            # 异常结束，携带错误信息
    sig_realtime = pyqtSignal(object)      # 中间/结果数据（绘图用）

    def __init__(self):
        super().__init__()
        self.ui = None
        self._worker = None
        self._bIsRunning = False
        current_class_file = inspect.getfile(self.__class__)
        file_name = os.path.basename(current_class_file)
        self._strName = os.path.splitext(file_name)[0]

        self.sig_log.connect(self.__slot_log)
        self.sig_realtime.connect(self.__slot_realtime)
        self.sig_finish.connect(self._on_finish)
        self.sig_error.connect(self._on_error)

    # ---------------- 运行控制 ----------------
    def begin_(self):
        """GUI 线程调用：读参数 → 起 daemon worker 线程跑 run()。"""
        if self._bIsRunning:
            return
        try:
            self.initial_before_run()
            dict_input, dict_mc = self.get_saved_param()
        except Exception as e:
            self.exception(str(e))
            return
        self._bIsRunning = True
        self._worker = threading.Thread(
            target=self.__worker,
            args=(copy.deepcopy(dict_input), copy.deepcopy(dict_mc)),
            name=self._strName + "-worker")
        self._worker.daemon = True
        self._worker.start()

    def stop_(self):
        """GUI 线程调用：中断（具体动作由子类 interrupt() 实现）。"""
        try:
            self.interrupt()
        except Exception as e:
            self.sig_error.emit(str(e))
        self._bIsRunning = False

    def is_running(self):
        return self._bIsRunning

    def __worker(self, dict_input, dict_mc):
        try:
            output = self.run(dict_input, dict_mc)
        except Exception as e:
            self._bIsRunning = False
            self.sig_error.emit(str(e))
            return
        self._bIsRunning = False
        self.sig_finish.emit(output if isinstance(output, dict) else {})

    # ---------------- 给 worker 线程用的安全接口（全部走信号） ----------------
    def log(self, log_msg):
        self.sig_log.emit(str(log_msg))

    def set_progress(self, progress, total):
        self.sig_progress.emit(int(progress), int(total))

    def set_realtime_data(self, realtime_data):
        self.sig_realtime.emit(realtime_data)

    def exception(self, err_msg):
        self._bIsRunning = False
        self.sig_error.emit(str(err_msg))

    # ---------------- GUI 线程槽 ----------------
    def __slot_log(self, log_msg):
        cur_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        self.plot_log("[%s]%s" % (cur_time, log_msg))

    def __slot_realtime(self, data):
        try:
            self.plot_realtime_data(data)
        except Exception as e:
            self.exception(str(e))

    def _on_finish(self, output):
        """正常结束槽：默认写日志，子类可重写（记得处理按钮状态）。"""
        self.plot_log("[%s]test finish: %s"
                      % (datetime.now().strftime("%Y/%m/%d %H:%M:%S"),
                         output))

    def _on_error(self, err_msg):
        """异常结束槽：默认写日志，子类可重写。"""
        self.plot_log("[%s]test error: %s"
                      % (datetime.now().strftime("%Y/%m/%d %H:%M:%S"),
                         err_msg))

    # ---------------- 子类重写 ----------------
    @abstractmethod
    def run(self, dict_input, dict_mc):
        """具体的测试流程（worker 线程内执行；禁止直接碰 UI）。"""
        return {}, {}

    @abstractmethod
    def interrupt(self):
        """中断时的操作（GUI 线程调用，须让 worker 尽快退出）。"""
        pass

    @abstractmethod
    def get_saved_param(self):
        """整理需要保存的输入参数与机器常数。返回 (dict_input, dict_mc)。"""
        return {}, {}

    @abstractmethod
    def set_saved_param(self, dict_input, dict_mc):
        """把保存的输入参数、机器常数写回控件。"""
        pass

    def initial_before_run(self):
        """每次运行之前需要完成的操作（可选重写）。"""
        pass

    def plot_realtime_data(self, realtime_data):
        """显示中间/结果数据（GUI 线程）。子类重写。"""
        pass

    def plot_log(self, log_msg):
        """显示 log 信息（GUI 线程）。默认 append 到 ui.textEdit。"""
        if self.ui is not None and hasattr(self.ui, "textEdit"):
            self.ui.textEdit.append(log_msg)
            scroll_bar = self.ui.textEdit.verticalScrollBar()
            scroll_bar.setValue(scroll_bar.maximum())
