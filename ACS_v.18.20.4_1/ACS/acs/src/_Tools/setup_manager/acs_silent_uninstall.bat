@echo off
setlocal

REM check admin rights
net session >nul 2>&1
if not %errorLevel% == 0 (
    echo Unable to uninstall: you must run this script as Administrator
    goto end
)

REM uninstall ACS silently
echo uninstalling ACS ...
python setups/backup_mgr.py backup
if not %errorLevel% == 0 (
    echo Error during uninstall
    goto end
)
python setups/setup_mgr.py uninstall acs_sources
if not %errorLevel% == 0 (
    echo Error during uninstall
    goto end
)

:end
endlocal
