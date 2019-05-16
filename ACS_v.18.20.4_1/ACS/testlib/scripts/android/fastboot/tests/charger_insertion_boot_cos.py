#!/usr/bin/env python

######################################################################
#
# @filename:    charger_insertion_boot_cos.py
# @description: Test boot to COS when charger is inserted.
#
# @run example:
#
#            python charger_insertion_boot_cos.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               USB_VC_cut_port=1
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
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
USB_VC_cut_port = args["USB_VC_cut"]

##### test start #####
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port)()

    local_steps.command("adb -s {} shell reboot -p".format(serial))()

    # check charging
    local_steps.wait_for_cos(serial=serial)()

    ## disconnect adb
    relay_steps.connect_disconnect_usb(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               connect=False,
                               USB_VC_cut_port=USB_VC_cut_port)()
    time.sleep(10)
    ## reconnect adb
    relay_steps.connect_disconnect_usb(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               connect=True,
                               USB_VC_cut_port=USB_VC_cut_port)()

    ## check charging
    local_steps.wait_for_cos(serial=serial)()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port)()
##### test end #####