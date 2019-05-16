#!/usr/bin/env python

######################################################################
#
# @filename:    graceful_shutdown_usbconnection.py
# @description: Gracefullty shut down the device.
#
# @run example:
#
#            python graceful_shutdown_usbconnection.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.adb import adb_steps
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

##### test start #####
def get_devices_info(command):
    r = os.popen(command)
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        for s in line:
            if "androidia_64" in s:
                return "androidia_64"
    return None

try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port)()

    devices_info = get_devices_info("adb shell getprop ro.product.device")

    # perform a graceful shutdown
    relay_steps.gracefully_power_off_device(serial=serial,
                                            relay_type = relay_type,
                                            relay_port = relay_port,
                                            power_port = power_port)()

    # check charging
    if devices_info != "androidia_64":
        local_steps.wait_for_cos(serial=serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port)()
    else:
        time.sleep(30)

    # power on device
    relay_steps.power_on_device(serial=serial,
                                relay_type = relay_type,
                                relay_port = relay_port,
                                power_port = power_port)()

except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port)()
##### test end #####