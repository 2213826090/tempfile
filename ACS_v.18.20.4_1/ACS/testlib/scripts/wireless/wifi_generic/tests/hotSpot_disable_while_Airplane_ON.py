#!/usr/bin/env python

######################################################################
#
# @filename:    hotSpot_disable_while_Airplane_ON.py
# @description: Test that Hotspot is disabled when Airplane mode is ON
#
# @run example:
#
#            python hotSpot_disable_while_Airplane_ON.py -s 0BA8F2A0
#                                               --script-args
#                                                       hotSpot_band=2.4
#                                                       hotSpot_security=WPA2
#                                                       hotSpot_pass=test1234
#

#
# @author:      mihalachex.micu@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps


##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params

hotSpot_band = args["hotSpot_band"]
hotSpot_security = args["hotSpot_security"]
hotSpot_pass = args["hotSpot_pass"]
serial2 = args["serial2"]

# define the hotspot name
hotSpot_name = "HS_" + serial

# restart as root
adb_steps.root_connect_device(serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# get the DUT ip.
#  In case that ip is 192.168.43.1 we need to turn off the hotspot
dut_ip = wifi_utils.get_connection_parameter(parameter = "ip_address",
                                             serial=serial)

# turn of the hotspot before start the test
if dut_ip == wifi_defaults.wifi["hotSpot_ip"]:
    # turn off the HotSpot in case that hotspot is on
    wifi_generic_steps.set_hotSpot(serial=serial, hotSpot_state="OFF")()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# make sure there are no p2p connections
wifi_generic_steps.p2p_disconect_all(serial = serial)()

# turn on the airplane mode
wifi_generic_steps.set_airplane_mode(serial = serial,
                                     state = "ON")()

# navigate to Tethering & portable hotspot view
#  and check the Portable Wi-Fi hotspot

ui_steps.click_button(serial = serial,
                    view_to_find = {"text":"Tethering & portable hotspot"},
                    view_to_check = {"text": "USB tethering"})()
ui_steps.wait_for_view(serial = serial, view_to_find = {"className":"android.widget.TextView","textContains":"Portable Wi", "enabled":"false"})()