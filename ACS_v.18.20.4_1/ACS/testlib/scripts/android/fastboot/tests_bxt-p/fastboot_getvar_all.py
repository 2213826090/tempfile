#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_getvar_all.py
# @description: Boots into fastboot mode and runs the "fastboot getvar all" command
#               It then checks the output for expected values
#
# @run example:
#
#            python fastboot_getvar_all.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#
#
# @author:      haojiex.xie@intel.com
#
#######################################################################

# Imports #
import os
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.connections.local import local_utils

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

# Test start #
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
    if wait_ui: fastboot_steps.check_all_vars(serial=serial, device_info = "broxtonp")()
    fastboot_steps.reboot(timeout = 300, serial = serial)()

except:
    raise

finally:
    if serial in local_utils.get_fastboot_devices():
        fastboot_steps.continue_to_adb(serial = serial)()
    
    # relay_steps.reboot_main_os(serial = serial,
    #                          relay_type = relay_type,
    #                          relay_port = relay_port,
    #                          power_port = power_port,
    #                          wait_ui = wait_ui,
    #                          timeout = 300,
    #                          delay_power_on = 30,
    #                          device_info = "broxtonp")()
# Test end #