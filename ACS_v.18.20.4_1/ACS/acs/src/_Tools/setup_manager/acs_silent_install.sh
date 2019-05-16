#!/bin/sh

if [ $(id -u) -ne 0 ]; then
  echo "Unable to install: you must run this script as root"
  exit 1
fi

pkg="platformflashtool"
isInstalled=0
if dpkg --get-selections | grep -q "^$pkg[[:space:]]*install$" >/dev/null; then
    isInstalled=1
fi
if [ $isInstalled -ne 1 ]; then
  echo "Platform Flash Tool not found !"
  echo "You must install Platform Flash Tool before installing ACS."
  echo "For more details visit Platform flash tool wiki (https://wiki.ith.intel.com/display/DRD/Platform+Flash+Tool)"
  exit 1
fi

echo installing ACS ...
python setups/setup_mgr.py install acs_third_parties
if [ $? -ne 0 ]; then
  echo "Error during install"
  exit 1
fi
python setups/setup_mgr.py uninstall acs_sources
if [ $? -ne 0 ]; then
  echo "Error during install"
  exit 1
fi