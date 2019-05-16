#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_SafeMode_MagicKey.py
# @description: Reboots in safe mode and checks if the step is
#               successful
#
#
# @run example:
#
#            python fastboot_SafeMode_MagicKey.py -s 0BA8F2A0 --script-args
#                                               app_to_find="ApiTests"
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00011925-if00
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
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
v_up_port = args["v_up_port"]
v_down_port = args["v_down_port"]
app_to_find = args["app_to_find"]


# Test start #
try:
    # Enter safe mode using magic key
    relay_steps.reboot_safe_mode_magic_key(serial=serial,
                                           app_to_find=app_to_find,
                                           relay_type=relay_type,
                                           relay_port=relay_port,
                                           power_port=power_port,
                                           v_down_port=v_down_port,
                                           v_up_port=v_up_port)()

    # Reboot in normal mode
    adb_steps.reboot(serial=serial)()

    # Check the app is displayed in normal mode
    ui_steps.find_app_from_allapps(serial=serial,
                                   view_to_find={"text": app_to_find},
                                   presence=True)()
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port,
                               wait_ui=True)()

# Test end #
