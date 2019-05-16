#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_magic_key.py
# @description: Boots to fastboot using the key combination: Power + Vol Up
#
# @run example:
#
#            python fastboot_magic_key.py -s 0BA8F2A0  --script-args
#                                       relay_type=RLY08B
#                                       relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                       power_port=4
#                                       v_up_port=1
#                                       v_down_port=2
#
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.relay import relay_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
v_up_port =  args["v_up_port"]
v_down_port = args["v_down_port"]

##### test start #####
try:
    relay_steps.reboot_fastboot(serial=serial,
                                relay_type=relay_type,
                                relay_port=relay_port,
                                power_port=power_port,
                                v_up_port=v_up_port,
                                v_down_port=v_down_port)()

    fastboot_steps.continue_to_adb(serial=serial)()
except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               wait_ui=True)()
##### test end #####
