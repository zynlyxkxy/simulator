@echo off
rem ============================================================
rem  simbridge one-click entry (simulator\simbridge)
rem  1) wait for 28 model (port 9010 listening, up to 60s)
rem  2) start relay stack (motion x4 + io + bridge)
rem  3) auto prepare test scene (wafer + die on bond head, 8x then restore 1x)
rem  4) exit (caller continues, e.g. exe\start_all.bat -s)
rem ============================================================
cd /d %~dp0
set "CFG=%~dp0..\..\engineering\config"
set "DEPLOY=%~dp0deploy"
set "PATH=%DEPLOY%\bin;%PATH%"

echo [1/4] waiting for 28 model (9010) ...
call :wait_port 9010 60

echo [2/4] starting relay stack ...
start "relay-io"      "%DEPLOY%\bin\relay_io.exe"      --mw "%CFG%\mw_config.yaml" --iocfg "%CFG%\io_config" --simhost 127.0.0.1 --simport 9010
start "relay-motion1" "%DEPLOY%\bin\relay_motion.exe"  --mw "%CFG%\mw_config.yaml" --module motion1 --cfg "%CFG%\motion_config\motion1\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion2" "%DEPLOY%\bin\relay_motion.exe"  --mw "%CFG%\mw_config.yaml" --module motion2 --cfg "%CFG%\motion_config\motion2\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion3" "%DEPLOY%\bin\relay_motion.exe"  --mw "%CFG%\mw_config.yaml" --module motion3 --cfg "%CFG%\motion_config\motion3\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion4" "%DEPLOY%\bin\relay_motion.exe"  --mw "%CFG%\mw_config.yaml" --module motion4 --cfg "%CFG%\motion_config\motion4\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-bridge"  "%DEPLOY%\bin\relay_bridge.exe"  --mqtt 127.0.0.1:1883 --sim 127.0.0.1:9010

echo [3/4] starting scene prep in background (async, non-blocking) ...
start "simbridge-prep" cmd /c "cd /d %DEPLOY% && python prep_scene.py > prep.log 2>&1"

echo [4/4] simbridge relay up. scene prep running async -> see deploy\prep.log
echo       do NOT operate the master until prep.log shows "[prep] done".
exit /b 0

rem ---- wait until a loopback port is LISTENING (poll 2s) ----
:wait_port
set /a _t=0
:wait_port_loop
netstat -ano | findstr "127.0.0.1:%~1" | findstr LISTENING >nul 2>nul
if not errorlevel 1 (
    echo   [simbridge] port %~1 is up
    exit /b 0
)
set /a _t+=2
if %_t% GEQ %~2 (
    echo   [simbridge] WARN port %~1 not listening within %~2s
    exit /b 1
)
timeout /t 2 /nobreak >nul
goto :wait_port_loop
