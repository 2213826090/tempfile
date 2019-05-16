#!/usr/bin/env python

######################################################################
#
# @filename:    main_to_fastboot.py
# @description: Boots into fastboot from main OS
#
# @run example:
#
#            python main_to_fastboot.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
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
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port)()

    # Reboot to fastboot
    adb_steps.reboot(command="bootloader",
                     serial=serial)()

    fastboot_steps.continue_to_adb(serial=serial)()
except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               wait_ui=True)()
# test end #