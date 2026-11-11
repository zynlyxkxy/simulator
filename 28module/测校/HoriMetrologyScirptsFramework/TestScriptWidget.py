# -*- coding: utf-8 -*-
"""
Filename: TestScriptWidget.py
Author: Jianming.Liu
Date: 2025/11/6
Description:

"""
from PyQt5.QtWidgets import QMessageBox, QPushButton, QDialog, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, QObject, QTimer
import threading
from ScriptWidget import ScriptWidget
import copy
import sys
import traceback


class ThreadSignals(QObject):
    """定义线程信号，用于与主线程通信"""
    finished = pyqtSignal()
    error = pyqtSignal(str)


class TestScriptWidget(ScriptWidget):
    def __init__(self):
        self.control_dialog = None
        self.run_thread = None
        super().__init__()

    def showEvent(self, event):
        """重写showEvent，在窗口显示时弹出对话框"""
        super().showEvent(event)
        self._show_control_dialog()

    def _show_control_dialog(self):
        """显示非模态控制对话框"""
        if self.control_dialog is None:
            self.control_dialog = ControlDialog(self)

        # 确保对话框显示在最前面
        self.control_dialog.show()
        self.control_dialog.raise_()
        self.control_dialog.activateWindow()

    def start_run_thread(self):
        """在新线程中执行run函数"""
        # 如果已有线程在运行，先停止
        if self.run_thread and self.run_thread.is_alive():
            return

        # 创建并启动线程，传入参数
        input_config, input_mc = self.get_saved_param()
        running_mc = copy.deepcopy(input_mc)

        # 创建线程对象
        self.initial_before_run()
        self.run_thread = RunThread(self.run, copy.deepcopy(input_config), running_mc)

        # 连接信号
        self.run_thread.signals.finished.connect(self.on_run_finished)
        self.run_thread.signals.error.connect(self.on_run_error)

        # 启动线程
        self.run_thread.start()

        # 禁用Run按钮防止重复点击
        if self.control_dialog:
            self.control_dialog.run_button.setEnabled(False)

    def on_run_finished(self):
        """run函数执行完成"""
        print("Run执行完成")
        # 重新启用Run按钮
        if self.control_dialog:
            self.control_dialog.run_button.setEnabled(True)

    def on_run_error(self, error_msg):
        """run函数执行出错"""
        print(f"Run执行出错: {error_msg}")
        # 重新启用Run按钮
        if self.control_dialog:
            self.control_dialog.run_button.setEnabled(True)

    def interrupt(self):
        """停止执行"""
        # 停止线程（如果正在运行）
        if self.run_thread and self.run_thread.is_alive():
            # 设置停止标志
            self.run_thread.stop()
            # 等待线程结束（可选，可以设置超时）
            self.run_thread.join(timeout=2.0)

        # 调用父类的interrupt方法
        super().interrupt()

        # 重新启用Run按钮
        if self.control_dialog:
            self.control_dialog.run_button.setEnabled(True)


class RunThread(threading.Thread):
    """用于执行run函数的线程，使用threading.Thread"""

    def __init__(self, run_function, dict_input, dict_mc):
        super().__init__()
        self.run_function = run_function
        self.dict_input = dict_input
        self.dict_mc = dict_mc
        self._stop_event = threading.Event()
        self.daemon = True  # 设置为守护线程，主程序退出时自动结束
        self.signals = ThreadSignals()  # 创建信号对象

    def stop(self):
        """请求线程停止"""
        self._stop_event.set()

    def stopped(self):
        """检查是否请求停止"""
        return self._stop_event.is_set()

    def run(self):
        try:
            # 检查是否在运行前就被请求停止
            if self.stopped():
                return

            # 调用run函数并传入参数
            # 注意：这里假设run_function能够响应中断请求
            # 如果run_function是长时间运行的操作，可能需要在其内部定期检查停止标志
            self.run_function(self.dict_input, self.dict_mc)

            # 只有在没有被停止的情况下才发射完成信号
            if not self.stopped():
                self.signals.finished.emit()

        except Exception as e:
            # 只有在没有被停止的情况下才发射错误信号
            if not self.stopped():
                exc_type, exc_value, exc_traceback = sys.exc_info()
                stack_trace = traceback.format_exception(exc_type, exc_value, exc_traceback)
                stack_trace_str = "".join(stack_trace)

                # 发送详细的异常信息
                self.signals.error.emit(str(type(e).__name__) + "\n" + stack_trace_str)
                # self.signals.error.emit(str(e))


class ControlDialog(QDialog):
    """自定义控制对话框"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("脚本控制")
        self.setFixedSize(200, 100)

        layout = QVBoxLayout()

        # 按钮布局
        button_layout = QHBoxLayout()

        self.run_button = QPushButton("Run")
        self.stop_button = QPushButton("Stop")

        self.run_button.clicked.connect(self.on_run_clicked)
        self.stop_button.clicked.connect(self.on_stop_clicked)

        button_layout.addWidget(self.run_button)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(button_layout)
        self.setLayout(layout)

    def on_run_clicked(self):
        """Run按钮点击处理 - 不关闭对话框"""
        self.parent().start_run_thread()

    def on_stop_clicked(self):
        """Stop按钮点击处理 - 不关闭对话框"""
        self.parent().interrupt()
