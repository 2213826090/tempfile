#!/usr/bin/env python

######################################################################
#
# @filename:    get_product_value_in_pos.py
# @description: Type "fastboot getvar product" return ""
#
# @run example:
#
#            python get_product_value_in_pos.py --serial=A6EC8F70 --script-args
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
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.connections.local import local_steps
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

    adb_steps.reboot(command = "fastboot",
                     reboot_timeout = 300,
                     serial = serial)()

    # command = "fastboot getvar product"
    # result = False
    # r = os.popen(command)
    # info = r.readlines()
    # for line in info:
    #     line = line.strip("\r\n")
    #     line = line.split()
    #     for s in line:
    #         if "" in s:
    #             result = True
    # time.sleep(5)

    fastboot_steps.continue_to_adb(serial = serial)()

    # if not result:
    #     raise Exception("The test result did not achieve the desired results")

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