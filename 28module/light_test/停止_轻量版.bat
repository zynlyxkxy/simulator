@echo off
rem ==========================================================================
rem 停止细化前端旁路栈（9030 / ACS 9131 / PLC 9132 / MCS 9390 / MQTT 9184 / VTCP 9230）
rem 只杀「监听 9030 且命令行含 main.py 的 python」，不碰其它进程与主栈 9010
rem ==========================================================================
setlocal enabledelayedexpansion
set FOUND=0
for /f "tokens=5" %%p in ('netstat -ano ^| findstr /r /c:"127[.]0[.]0[.]1:9030 " ^| findstr LISTENING') do (
    set PID=%%p
    wmic process where "ProcessId=!PID!" get CommandLine 2>/dev/null | findstr /i /c:"main.py" | findstr /i /c:"python" >nul
    if !errorlevel! equ 0 (
        echo [kill] PID !PID! 占用 9030（仿真后端）
        taskkill /PID !PID! /F >nul 2>&1
        set FOUND=1
    ) else (
        echo [warn] 9030 被非仿真进程占用（PID !PID!），未处理
    )
)
if !FOUND! equ 0 echo [ok] 9030 上没有仿真后端在跑
echo.
echo [tip] 主栈 9010 用根目录 start.bat 启停；本脚本只管细化前端旁路栈
pause

