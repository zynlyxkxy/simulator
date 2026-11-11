@echo off
rem D2W sim one-click startup（迭代 37：9010 占用先做健康检查，陈旧 backend 自动替换）
cd /d "%~dp0backend"
setlocal enabledelayedexpansion
set PORT=9010
set URL=http://127.0.0.1:%PORT%/api/health

set REUSE=0
netstat -ano | findstr /r /c:"127[.]0[.]0[.]1:%PORT% " | findstr LISTENING >nul
if !errorlevel! neq 0 goto START_BACKEND
rem 端口已占用：健康检查是否为现行 backend（/api/health 含 build 字段）
set HEALTH=
for /f "usebackq delims=" %%h in (`curl -s -m 3 %URL%`) do set HEALTH=%%h
if not defined HEALTH goto OCCUPIED
echo !HEALTH! | findstr "\"build\"" >nul
if !errorlevel! equ 0 (
    echo [skip] backend already running on %PORT% (health ok)
    set REUSE=1
    goto OPEN_UI
)

:OCCUPIED
echo [warn] %PORT% occupied by unhealthy/stale process - checking owner...
set KILLED=0
for /f "tokens=5" %%p in ('netstat -ano ^| findstr "127.0.0.1:%PORT%" ^| findstr LISTENING') do (
    wmic process where "ProcessId=%%p" get CommandLine 2>nul | findstr /i /c:"main.py" | findstr /i /c:"python" >nul
    if !errorlevel! equ 0 (
        echo [ok] killing stale backend PID %%p
        taskkill /PID %%p /F >nul 2>nul
        set KILLED=1
    )
)
if !KILLED! equ 1 (
    timeout /t 2 >nul
    goto START_BACKEND
)
echo [error] %PORT% 被非本仿真进程占用，未自动处理。请手动释放端口后重试。
pause
exit /b 1

:START_BACKEND
set PYTHONNOUSERSITE=1
start "D2W-sim-backend" "%~dp0env\python\python.exe" runtime\main.py
echo [ok] backend starting
timeout /t 3 >nul

:OPEN_UI
start "" "http://127.0.0.1:%PORT%/panel"
start "" "%~dp0html_model\d2w_online.html"
echo [ok] panel and 3D viewer opened
pause
