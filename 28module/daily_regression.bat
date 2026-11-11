@echo off
rem D2W 每日定时回归（迭代 50）：计划任务 D2W_DailyRegression 调用本脚本，
rem 也可手动跑：daily_regression.bat [--full] [--segs a,b] [-k "..."]（空参=quick）
chcp 936 >nul
cd /d "%~dp0"
if not exist "env\python\python.exe" (
    echo [错误] 未找到项目内置 Python：env\python\python.exe，请确认项目目录完整。
    exit /b 1
)
set PYTHONNOUSERSITE=1
"env\python\python.exe" backend\tests\run_regression.py %*
exit /b %errorlevel%
