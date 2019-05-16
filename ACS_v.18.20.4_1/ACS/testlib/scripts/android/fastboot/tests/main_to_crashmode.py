#!/usr/bin/env python

######################################################################
#
# @filename:  main_to_crashmode.py
# @description: Boots into recovery mode via adb from main os
#
# @run example:
#
#            python main_to_crashmode.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# imports #
import os
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps
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
try:
    device_state = local_utils.get_device_boot_state(serial=serial)

    if device_state == "fastboot":
        fastboot_steps.continue_to_adb(serial=serial)()
        adb_steps.wait_for_ui_processes(serial=serial)()
    elif device_state == "recovery":
        adb_steps.reboot(serial=serial)()
    elif device_state == "crashmode":
        print "adb -s {} reboot".format(serial)
        local_steps.command("adb -s {} reboot".format(serial))()
        local_steps.wait_for_adb(serial=serial)()
        adb_steps.wait_for_ui_processes(serial=serial)()
    else:
        relay_steps.reboot_main_os(serial=serial,
                                   relay_type=relay_type,
                                   relay_port=relay_port,
                                   power_port=power_port,
                                   wait_ui=True)()

    # Reboot to crashmode
    os.system("adb -s {} reboot crashmode > /dev/null 2>&1".format(serial))
    time.sleep(15)
    local_steps.wait_for_crashmode(serial=serial, timeout=60)()

except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               wait_ui=True)()
# test end #