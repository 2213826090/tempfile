#!/usr/bin/env python

######################################################################
#
# @filename:    boot_android_natively.py
# @description: Press "lgnition" button to main OS.
#
# @run example:
#
#            python boot_android_natively.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
# @author:      haojiex.xie@intel.com
#
#######################################################################

##### imports #####
import os
import sys
import time
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
    relay_steps.reboot_main_os(serial = serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port,
                                 wait_ui = wait_ui,
                                 timeout = 300,
                                 delay_power_on = 30,
                                 device_info = "broxtonp")()

    relay_steps.reboot_main_os(serial = serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port,
                                 force_reboot = True,
                                 wait_ui = wait_ui,
                                 timeout = 300,
                                 delay_power_on = 30,
                                 device_info = "broxtonp")()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial = serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port,
                                 wait_ui = wait_ui,
                                 timeout = 300,
                                 delay_power_on = 30,
                                 device_info = "broxtonp")()

##### test end #####