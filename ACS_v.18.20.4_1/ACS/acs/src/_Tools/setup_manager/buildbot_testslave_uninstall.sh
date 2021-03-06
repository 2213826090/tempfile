#!/bin/sh

if [ $(id -u) -ne 0 ]; then
  echo "Unable to install: you must run this script as root"
  exit 1
fi

echo Uninstalling BUILDBOT TEST SLAVE ...
python setups/setup_mgr.py uninstall buildbot_testslave
if [ $? -ne 0 ]; then
  echo "Error during install"
  exit 1
fi
