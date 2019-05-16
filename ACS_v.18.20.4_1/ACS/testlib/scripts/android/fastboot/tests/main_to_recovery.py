#!/usr/bin/env python

######################################################################
#
# @filename:    main_to_recovery.py
# @description: Boots into recovery mode via adb
#
# @run example:
#
#            python main_to_recovery.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_utils
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
    device_state = local_utils.get_device_boot_state(serial = serial)
    if device_state == "fastboot":
        fastboot_steps.continue_to_adb(serial = serial)()
    elif device_state == "recovery":
        adb_steps.reboot(serial = serial)()

    adb_steps.reboot_recovery(serial = serial)()
except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port,
                             wait_ui = True)()
##### test end #####
