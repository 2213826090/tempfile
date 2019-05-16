#!/usr/bin/env python

######################################################################
#
# @filename:    check_mac_after_reboot.py
# @description: Verifies the MAC after DUT reboot.
#
# @run example:
#
#            python check_mac_after_reboot.py -s 0BA8F2A0
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
if "use_adb" in args.keys():
    use_adb_value =args["use_adb"]
else:
   use_adb_value = "True"
use_adb_Value="False"

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# set wifi on and off
wifi_generic_steps.set_wifi(serial = serial, use_adb = False, state="ON")()
time.sleep(3)
wifi_generic_steps.set_wifi(serial = serial, use_adb = False, state="OFF")()

mac1=wifi_generic_steps.check_mac(serial = serial)()# reboot device
#adb_steps.reboot(serial = serial,
                 #ip_enabled = False)()
#ui_steps.set_orientation_vertical(serial = serial, orientation="portrait")()
adb_steps.root_connect_device(serial = serial)()
adb_steps.command(serial = serial,
            command = "settings put global captive_portal_detection_enabled 0",
            mode = "sync",
            timeout = 10)()

# wifi on
wifi_generic_steps.set_wifi(serial = serial, use_adb=use_adb_value,state="ON")()

# check the MAC existance
wifi_generic_steps.check_mac(serial = serial)()


##### test end #####
