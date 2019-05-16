#!/usr/bin/env python

######################################################################
#
# @filename:    SafeModeByProperty.py
# @description: Reboots in safe mode and checks if the step is
#               successful
#
#
# @run example:
#
#            python SafeModeByProperty.py -s 0BA8F2A0 --script-args
#                                               app_to_find="ApiTests"
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.connections.local import local_utils

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

app_to_find = args["app_to_find"]

# Test start #
if serial in local_utils.get_fastboot_devices():
    fastboot_steps.continue_to_adb(serial=serial)()

# Reboot in safe mode
adb_steps.reboot(serial=serial, safe_mode=True)()

# Check the app is NOT displayed in safe mode
ui_steps.find_app_from_allapps(serial=serial,
                               view_to_find={"text": app_to_find},
                               presence=False)()

# Reboot in normal mode
adb_steps.reboot(serial=serial)()

# Check the app is displayed in normal mode
ui_steps.find_app_from_allapps(serial=serial,
                               view_to_find={"text": app_to_find},
                               presence=True)()

# Test end #
