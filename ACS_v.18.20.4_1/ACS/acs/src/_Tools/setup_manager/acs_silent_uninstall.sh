#!/bin/sh

if [ $(id -u) -ne 0 ]; then
  echo "Unable to uninstall: you must run this script as root"
  exit 1
fi

echo uninstalling ACS ...
python setups/backup_mgr.py backup
if [ $? -ne 0 ]; then
  echo "Error during uninstall"
  exit 1
fi
python setups/setup_mgr.py uninstall acs_sources
if [ $? -ne 0 ]; then
  echo "Error during uninstall"
  exit 1
fi
