@echo off
chcp 936 >nul
title D2W 测校软件

set "PYEXE=%~dp0env\python\python.exe"
if not exist "%PYEXE%" (
    echo [错误] 未找到项目内置 Python 环境 env\python，请确认环境目录完整
    pause
    exit /b 1
)
set PYTHONNOUSERSITE=1

set APPDIR=%~dp0backend\metrology_gui

rem 先检查后端 9010 是否在线（不在线仍启动，仅提示）
curl -s -m 2 -o nul http://127.0.0.1:9010/api/machine_constants
if errorlevel 1 (
    echo [提示] 未检测到后端仿真（127.0.0.1:9010），测校运行会等待超时。
    echo [提示] 可先运行 start.bat 拉起后端；界面仍会正常打开。
) else (
    echo [OK] 后端仿真在线。
)

"%PYEXE%" "%APPDIR%\main.py"
if errorlevel 1 pause
