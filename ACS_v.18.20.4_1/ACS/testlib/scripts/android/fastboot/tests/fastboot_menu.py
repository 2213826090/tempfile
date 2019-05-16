#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_menu.py
# @description: Tests fastboot menu option
#
# @run example:
#
#            python fastboot_menu.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               v_up_port=7
#                                               v_down_port=8
#                                               option=normal_boot
#
#           - option: select the option from fastboot menu:
#                - normal_boot
#                - power_off
#                - bootloader
#                - recovery
#                - reboot
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_utils
from testlib.scripts.relay import relay_steps
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
v_up_port =  args["v_up_port"]
v_down_port = args["v_down_port"]
USB_VC_cut_port = args["USB_VC_cut"]
option = args["option"]

##### test start #####
try:
    # ensure main OS is booted
    if local_utils.get_device_boot_state(serial = serial) != "android":
        relay_steps.reboot_main_os(serial=serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port)()

    # boot to fastboot
    adb_steps.reboot(command="fastboot", serial=serial)()

    # select the desired option from menu
    relay_steps.choose_fastboot_menu(serial = serial,
                                option = option,
                                relay_type = relay_type,
                                relay_port = relay_port,
                                power_port = power_port,
                                v_up_port = v_up_port,
                                v_down_port = v_down_port)()

    if option == "power_off":
        time.sleep(15)
        ## disconnect adb
        relay_steps.connect_disconnect_usb(serial=serial, relay_type=relay_type, relay_port=relay_port,
                                                                        connect=False, USB_VC_cut_port=USB_VC_cut_port)()
        time.sleep(10)
        ## reconnect adb
        relay_steps.connect_disconnect_usb(serial=serial, relay_type=relay_type, relay_port=relay_port,
                                                                        connect=True, USB_VC_cut_port=USB_VC_cut_port)()
        ## check charging
        local_steps.wait_for_cos(serial=serial)()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port,
                             wait_ui = True)()
##### test end #####