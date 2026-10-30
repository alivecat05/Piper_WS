@echo off
setlocal
pushd "%~dp0"
".conda\Scripts\conda.exe" run --prefix ".conda\envs\export" --no-capture-output python -m piperx_teleop.export %*
set "export_exit=%errorlevel%"
popd
exit /b %export_exit%
