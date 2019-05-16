#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_oem_lock_unlock.py
# @description: Boots into recovery mode via adb and locks the
#               bootloader
#
# @run example:
#
#            python fastboot_oem_lock_unlock.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#                                               lock_state = yes/ no
#
# lock_state = yes (to unlock)/ no (to lock)
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.ui import ui_steps

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
lock_state = args["lock_state"]

# Test start #
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port)()
    adb_steps.wait_for_ui(serial=serial)()

    ui_steps.wake_up_device(serial=serial)()
    ui_steps.unlock_device(serial=serial)()

    # Enable OEM unlock in developer options #
    ui_steps.enable_oem_unlock(serial=serial, enabled=False, blocking=False)()

    dessert = adb_utils.get_android_version(serial=serial)
    adb_steps.reboot(command="fastboot", serial=serial)()

    relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader=lock_state,
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port)()
    adb_steps.wait_for_ui(serial=serial)()
# Test end #
