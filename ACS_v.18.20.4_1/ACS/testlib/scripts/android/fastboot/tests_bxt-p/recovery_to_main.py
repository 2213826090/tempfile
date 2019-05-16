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
# @author:      haojiex.xie@intel.com
#
#######################################################################

##### imports #####
import os
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local import local_utils
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps

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
def get_devices_info(command):
    r = os.popen(command)
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        for s in line:
            if "bxtp_abl" in s:
                return True
            if "gordon_peak" in s:
                return False

wait_ui = get_devices_info("adb shell getprop ro.product.device")

try:
    # ensure main OS is booted
    if local_utils.get_device_boot_state(serial=serial) != "android":
        relay_steps.reboot_main_os(serial=serial,
                                   relay_type = relay_type,
                                   relay_port = relay_port,
                                   power_port = power_port,
                                   wait_ui = wait_ui,
                                   delay_power_on = 30,
                                   device_info = "broxtonp")()

    # boot to recovery
    local_steps.command("adb -s {} reboot recovery".format(serial))()

    time.sleep(60)

    # boot to main from ROS menu
    relay_steps.long_press_power_shutdown(serial=serial,
                                          relay_type = relay_type,
                                          relay_port = relay_port,
                                          power_port = power_port,
                                          except_charging = True,
                                          long_press_time = 15,
                                          device_info = "broxtonp")()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = wait_ui,
                               delay_power_on = 30,
                               device_info = "broxtonp")()

##### test end #####