@echo off
setlocal

REM check admin rights
net session >nul 2>&1
if not %errorLevel% == 0 (
    echo Unable to install: you must run this script as Administrator
    goto end
)

REM install Buildbot
:install_buildbot
echo Uninstalling BUILDBOT TEST SLAVE ...
%TMP_PYTHON_PATH%python setups/setup_mgr.py uninstall buildbot_testslave
if not %errorLevel% == 0 (
    echo Error during install
    goto end
)

:end
endlocal
