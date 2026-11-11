# -*- coding: utf-8 -*-
"""metrology — 测校程序包（相对独立，不进主控全量回归 test_client.py）。

相机对准重复性测校（repeatability.py）：
- run_static  静态重复性：设备不动，定点连拍 N 次，统计质心/角度散布；
- run_dynamic 动态重复性：每循环 载体轴退 travel_mm → 回拍照位 → 拍照，
  同时记录各运动轴实际到位位置。

两个程序由 runtime/calibration.py 注册为
align_repeat_static / align_repeat_dynamic 测校作业（走真实 REAL 视觉
链路），也可经 metrology/run_cli.py 命令行独立跑（连已启动的后端
HTTP API）。
"""
