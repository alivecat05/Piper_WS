@echo off
setlocal
pushd "%~dp0"
set "PYTHONUTF8=1"
set "CAPTURE_ENV=%~dp0.conda\envs\capture"
if not exist "%CAPTURE_ENV%\python.exe" set "CAPTURE_ENV=%~dp0..\PiperX_Teleop\.conda\envs\teleop"
if not exist "%CAPTURE_ENV%\python.exe" (
  echo Run setup.cmd first.
  exit /b 1
)
set "PATH=%CAPTURE_ENV%;%CAPTURE_ENV%\Library\bin;%CAPTURE_ENV%\Scripts;%PATH%"
"%CAPTURE_ENV%\python.exe" -m piper_capture.app %*
set "capture_exit=%errorlevel%"
popd
exit /b %capture_exit%
