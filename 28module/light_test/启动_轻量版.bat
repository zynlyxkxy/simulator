@echo off
rem ==========================================================================
rem 细化前端（1928_front_light）旁路演示栈 —— 独立端口，不动主栈 9010
rem   WS 9030 / ACS 9131 / PLC 9132 / MCS 9390 / MQTT 9184 / VTCP 9230
rem ==========================================================================
setlocal
cd /d "%~dp0"
set PYTHONNOUSERSITE=1
set PYTHONIOENCODING=utf-8
set D2W_PORT_WS=9030
set D2W_PORT_ACS=9131
set D2W_PORT_PLC=9132
set D2W_PORT_MCS=9390
set D2W_PORT_MQTT=9184
set D2W_PORT_VTCP=9230
set PYEXE=%~dp0..\env\python\python.exe
set HEALTH=http://127.0.0.1:9030/api/health

if not exist "%PYEXE%" (
    echo [error] 未找到项目 Python 环境：%PYEXE%
    pause
    exit /b 1
)

set HEALTHY=
for /f "usebackq delims=" %%h in (`curl -s -m 3 %HEALTH%`) do set HEALTHY=%%h
if defined HEALTHY (
    echo [skip] 9030 后端已在运行（健康检查通过）
    goto OPEN
)

echo [start] 启动细化前端后端（9030）...
cd /d "%~dp0backend"
start "D2W-light-backend" "%PYEXE%" runtime\main.py
cd /d "%~dp0"
timeout /t 4 >nul

:OPEN
rem 页面几何补丁（幂等）：晶圆坐面归零 + E_Pin 压紧零位 -2.5；只改产物，源文件不动
"%PYEXE%" "%~dp0tools\patch_light_page.py" >nul 2>&1
rem 本地路径直接带 "?ws=9030" 时，start 会把查询串当成文件名的一部分，
rem 报「Windows 找不到文件」；必须先拼成 file:/// URL 再交给默认浏览器。
set "HTMLURL=file:///%~dp0"
set "HTMLURL=%HTMLURL:\=/%"
start "" "http://127.0.0.1:9030/panel"
start "" "%HTMLURL%1928_front_light_online.html?ws=9030"
echo [ok] 面板 http://127.0.0.1:9030/panel 与细化前端已打开
echo [tip] 若浏览器未自动打开，手动访问上面地址；直接双击 html 时务必自行加 ?ws=9030
pause

