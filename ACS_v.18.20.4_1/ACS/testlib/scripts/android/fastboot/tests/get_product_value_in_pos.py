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
try:
    relay_steps.reboot_main_os(serial = serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = True)()

    adb_steps.reboot(command = "bootloader",
                     serial = serial)()

    result = False

    os.system("mkdir ./tmp")
    os.system("fastboot getvar product > ./tmp/tmp.txt 2>&1")

    f = open("./tmp/tmp.txt")
    info = f.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        if "product:" == line[0]:
            if "androidia_64" == line[1]:
                result = True
    f.close()

    os.system("rm -rf ./tmp")

    fastboot_steps.continue_to_adb(serial = serial)()

    if not result:
        raise Exception("The test result did not achieve the desired results")

except:
    raise

finally:
    relay_steps.reboot_main_os(serial = serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = True)()
##### test end #####
