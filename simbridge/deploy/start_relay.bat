@echo off
rem ============================================================
rem relay layer launcher (replaces the 5 CTL_MDL_* lines in start_all.bat)
rem reads the single config root under engineering\config\, relay is read-only:
rem    - mw_config.yaml                -> ..\..\..\engineering\config\mw_config.yaml
rem    - 4 motion stage configs        -> ..\..\..\engineering\config\motion_config\motionN\config.json
rem prereq: 28model (runtime/main.py) already running on 127.0.0.1 (9010/9101/9102/1883...)
rem ============================================================
cd /d %~dp0
set "CFG=%~dp0..\..\..\engineering\config"
set PATH=%CD%\bin;%PATH%

start "relay-io"      bin\relay_io.exe      --mw "%CFG%\mw_config.yaml" --iocfg "%CFG%\io_config" --simhost 127.0.0.1 --simport 9010
start "relay-motion1" bin\relay_motion.exe  --mw "%CFG%\mw_config.yaml" --module motion1 --cfg "%CFG%\motion_config\motion1\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion2" bin\relay_motion.exe  --mw "%CFG%\mw_config.yaml" --module motion2 --cfg "%CFG%\motion_config\motion2\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion3" bin\relay_motion.exe  --mw "%CFG%\mw_config.yaml" --module motion3 --cfg "%CFG%\motion_config\motion3\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-motion4" bin\relay_motion.exe  --mw "%CFG%\mw_config.yaml" --module motion4 --cfg "%CFG%\motion_config\motion4\config.json" --simhost 127.0.0.1 --simport 9010
start "relay-bridge"  bin\relay_bridge.exe  --mqtt 127.0.0.1:1883 --sim 127.0.0.1:9010

echo relay processes started (config from ..\..\..\engineering\config).
rem scene auto-prep is in ..\start_simbridge.bat (one-click entry)
