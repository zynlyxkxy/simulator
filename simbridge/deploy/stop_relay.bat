@echo off
rem stop relay layer only (does not touch master control / 28model / EMQX)
taskkill /F /IM relay_io.exe     >nul 2>&1
taskkill /F /IM relay_motion.exe >nul 2>&1
taskkill /F /IM relay_bridge.exe >nul 2>&1
echo relay stopped.
