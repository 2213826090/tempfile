#!/usr/bin/env python

######################################################################
#
# @filename:    main_to_main_50times.py
# @description: Boots into main OS from main OS
#
# @run example:
#
#            python main_to_main.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
#
# @author:      xuchaox.zhuang@intel.com
#
#######################################################################

# imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.connections.local import local_steps

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
relay_steps.reboot_main_os(serial=serial,
                           relay_type=relay_type,
                           relay_port=relay_port,
                           power_port=power_port,
                           wait_ui=True)()
    # Reboot to main OS
local_steps.command("adb -s {} reboot".format(serial))()
local_steps.wait_for_adb(serial=serial)()
adb_steps.wait_for_ui_processes(serial=serial)()
# test end #