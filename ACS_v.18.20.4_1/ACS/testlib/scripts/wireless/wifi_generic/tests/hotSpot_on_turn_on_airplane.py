#!/usr/bin/env python

######################################################################
#
# @filename:    hotSpot_on_turn_on_airplane.py
# @description: Test that HotSpot is turned off when airplane mode is turned on
#
# @run example:
#
#            python hotSpot_on_turn_on_airplane.py -s 0BA8F2A0
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


# define the hotspot name
hotSpot_name = "HS_" + serial

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


# set the airplane mode off
wifi_generic_steps.set_airplane_mode(serial = serial,
                                     state = "OFF")()

# create the hot spot
wifi_generic_steps.configure_hotSpot(serial = serial, hotSpot_name=hotSpot_name,
                                     hotSpot_security=hotSpot_security,
                                     hotSpot_band=hotSpot_band,
                                     hotSpot_pass=hotSpot_pass)()

# turn on the HotSpot
wifi_generic_steps.set_hotSpot(serial=serial, hotSpot_state="ON")()

# turn on the airplane mode
wifi_generic_steps.set_airplane_mode(serial = serial,
                                     state = "ON")()

# verify the status of the wifi
wifi_generic_steps.check_connection_info(serial = serial,
                                         state = "DISCONNECTED/DISCONNECTED")()

# turn off the airplane mode
wifi_generic_steps.set_airplane_mode(serial = serial,
                                     state = "OFF")()

# turn off the HotSpot
wifi_generic_steps.set_hotSpot(serial=serial, hotSpot_state="OFF")()
