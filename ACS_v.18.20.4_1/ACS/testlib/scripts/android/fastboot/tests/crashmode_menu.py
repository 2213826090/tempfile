#!/usr/bin/env python

######################################################################
#
# @filename:    crashmode_menu.py
# @description: Tests fastboot menu option
#
# @run example:
#
#            python crashmode_menu.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               v_up_port=7
#                                               v_down_port=8
#                                               option=normal_boot
#
#           - option: select the option from fastboot menu:
#                - normal_boot
#                - power_off
#                - bootloader
#                - recovery
#                - reboot
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import os
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.utils.statics.android import statics
from testlib.scripts.connections.local import local_steps

##### initialization #####
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
option = args["option"]

##### test start #####
try:
    # ensure main OS is booted
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port)()

    # get the option position in the menu as in crashmode cannot get the platform
    menu_position = statics.Device(serial=serial).crashmode_menu_entry[option]

    # boot to crashmode
    os.system("adb -s {} reboot crashmode > /dev/null 2>&1".format(serial))
    time.sleep(15)
    local_steps.wait_for_crashmode(serial=serial, timeout=60)()

    time.sleep(3)

    # select the desired option from menu
    relay_steps.choose_crashmode_menu(serial=serial,
                                      option=option,
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
