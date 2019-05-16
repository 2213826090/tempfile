#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_sanity_checks3.py
# @description: Type "fastboot flash bootloader file.txt" and "fastboot flash keystore file.txt" return "FAILED"
#
# @run example:
#
#            python fastboot_sanity_checks3.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
#
# @author:      haojiex.xie@intel.com
#
#######################################################################

# imports #
import os
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local import local_utils
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps

# initialization #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

# test start #
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
    relay_steps.reboot_main_os(serial=serial,
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

    result = True

    os.system("mkdir ./tmp")
    os.system("touch ./tmp/file.txt")
    os.system("fastboot flash bootloader ./tmp/file.txt > ./tmp/tmp.txt 2>&1")
    os.system("fastboot flash keystore ./tmp/file.txt > ./tmp/tmp1.txt 2>&1")

    f = open("./tmp/tmp.txt")
    info = f.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        if "FAILED" == line[0]:
            result = False
    f.close()

    r = open("./tmp/tmp1.txt")
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        if "FAILED" == line[0]:
            result = False
    r.close()

    os.system("rm -rf ./tmp")

    fastboot_steps.continue_to_adb(serial = serial)()

    if result:
        raise Exception("The test result did not achieve the desired results")

except:
    raise

finally:
    if serial in local_utils.get_fastboot_devices():
        fastboot_steps.continue_to_adb(serial = serial)()

    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = wait_ui,
                               timeout = 300,
                               delay_power_on = 30,
                               device_info = "broxtonp")()

# test end #