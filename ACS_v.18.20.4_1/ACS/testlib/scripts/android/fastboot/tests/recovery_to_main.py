#!/usr/bin/env python

######################################################################
#
# @filename:    recovery_to_main.py
# @description: Boots into main os from recovery menu
#
# @run example:
#
#            python recovery_to_main.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               v_up_port=7
#                                               v_down_port=8
#
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_utils
from testlib.scripts.relay import relay_steps
from testlib.utils.statics.android import statics

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
v_up_port =  args["v_up_port"]
v_down_port = args["v_down_port"]

##### test start #####
try:
    # ensure main OS is booted
    if local_utils.get_device_boot_state(serial=serial) != "android":
        relay_steps.reboot_main_os(serial=serial,
                                   relay_type=relay_type,
                                   relay_port=relay_port,
                                   power_port=power_port)()

    menu_position = statics.Device(serial=serial).ros_menu_entry["android"]

    # boot to recovery
    adb_steps.reboot_recovery(serial=serial)()

    time.sleep(3)

    # boot to main from ROS menu
    relay_steps.recovery_reboot(serial=serial,
                                mode="android",
                                menu_position=menu_position,
                                timeout=60,
                                relay_type=relay_type,
                                relay_port=relay_port,
                                power_port=power_port,
                                v_up_port=v_up_port,
                                v_down_port=v_down_port)()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               wait_ui=True)()
##### test end #####
