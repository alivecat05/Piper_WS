@echo off
setlocal
set "PYTHONUTF8=1"
pushd "%~dp0"
call ".conda\Scripts\activate.bat" "%~dp0.conda\envs\teleop"
if errorlevel 1 exit /b 1
".conda\envs\teleop\python.exe" -m piperx_teleop.app %*
set "teleop_exit=%errorlevel%"
popd
exit /b %teleop_exit%
