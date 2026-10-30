@echo off
setlocal
pushd "%~dp0"
set "PYTHONUTF8=1"
set "CAPTURE_ENV=%~dp0.conda\envs\export"
if not exist "%CAPTURE_ENV%\python.exe" set "CAPTURE_ENV=%~dp0..\PiperX_Teleop\.conda\envs\export"
if not exist "%CAPTURE_ENV%\python.exe" (
  echo Run setup.cmd -Export first.
  exit /b 1
)
set "PATH=%CAPTURE_ENV%;%CAPTURE_ENV%\Library\bin;%CAPTURE_ENV%\Scripts;%PATH%"
"%CAPTURE_ENV%\python.exe" -m piper_capture.export %*
set "capture_exit=%errorlevel%"
popd
exit /b %capture_exit%
