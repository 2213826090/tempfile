#!/usr/bin/env python

######################################################################
#
# @filename:    SafeModeOperation.py
# @description: Reboots in safe mode and checks if the step is
#               successful
#
#
# @run example:
#
#            python SafeModeOperation.py -s 0BA8F2A0 --script-args
#                                               app_to_find="ApiTests"
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
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
from testlib.scripts.relay import relay_steps

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
app_to_find = args["app_to_find"]

# Test start #
relay_steps.reboot_main_os(serial=serial,
                           relay_type=relay_type,
                           relay_port=relay_port,
                           power_port=power_port)()

ui_steps.wake_up_device(serial=serial)()
ui_steps.unlock_device(serial=serial)()

# Reboot in safe mode
relay_steps.reboot_safe_mode(serial=serial,
                             app_to_find=app_to_find,
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port)()

# Reboot in normal mode
adb_steps.reboot(serial=serial)()

# Check the app is displayed in normal mode
ui_steps.find_app_from_allapps(serial=serial,
                               view_to_find={"text": app_to_find},
                               presence=True)()

# Test end #
