@echo off
rem ============================================================
rem rebuild_bin.bat - (re)build relay and (re)create deploy\bin
rem   repo root = three levels above this file (relative, survives repo moves):
rem     <root>\simulator\simbridge\deploy\  ->  ..\..\..
setlocal
set "ROOT=%~dp0.."
set "PJ=%~dp0..\..\.."

echo [1/3] cmake configure+build ...
cmake -S "%ROOT%\relay" -B "%ROOT%\relay\build" -G "Visual Studio 16 2019" -A x64 >nul
if errorlevel 1 goto fail
cmake --build "%ROOT%\relay\build" --config Release --target relay_motion relay_io relay_bridge
if errorlevel 1 goto fail

echo [2/3] create deploy\bin ...
if not exist "%ROOT%\deploy\bin" mkdir "%ROOT%\deploy\bin"
rem a running relay locks its own exe + the dlls it loaded, so every copy
rem below can fail; collect it and report at the end instead of silently
rem leaving a stale deploy\bin behind.
set "COPYFAIL="
copy /y "%ROOT%\relay\build\Release\relay_motion.exe"      "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
copy /y "%ROOT%\relay\build\Release\relay_io.exe"          "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
copy /y "%ROOT%\relay\build\Release\relay_bridge.exe"      "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"

echo [3/3] copy runtime dlls from the repo root ...
rem post-refactor paths (old module\*\interface\lib + exe\lib\* are gone):
rem   module dll   -> module\<m>\output\interface\rpc\lib\release\
rem   platform dll -> platform_output\basiclibrary_output\lib\
rem   VC runtime   -> engineering\runtime\system\
copy /y "%PJ%\module\motion\output\interface\rpc\lib\release\motion.dll" "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
copy /y "%PJ%\module\io\output\interface\rpc\lib\release\io.dll"         "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
copy /y "%PJ%\platform_output\basiclibrary_output\lib\*.dll"             "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
copy /y "%PJ%\engineering\runtime\system\*.dll"                          "%ROOT%\deploy\bin\" >nul || set "COPYFAIL=1"
if not exist "%ROOT%\deploy\bin\motion.dll" set "COPYFAIL=1"
if not exist "%ROOT%\deploy\bin\io.dll" set "COPYFAIL=1"
if defined COPYFAIL goto copyfail

echo done. deploy\bin ready.
exit /b 0
:copyfail
echo [ERROR] a copy into deploy\bin failed - deploy\bin is incomplete.
echo         cause 1: module outputs not built
echo                  build them first: engineering\scripts\build_exe.bat io motion
echo         cause 2: relay still running locks deploy\bin
echo                  stop it first: simbridge\deploy\stop_relay.bat
exit /b 1
:fail
echo BUILD FAILED. see messages above.
exit /b 1
