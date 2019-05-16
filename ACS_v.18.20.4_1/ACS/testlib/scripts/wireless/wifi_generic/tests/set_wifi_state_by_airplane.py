#!/usr/bin/env python

######################################################################
#
# @filename:    set_wifi_state_by_airplane.py
# @description: Tests that when correctly adding a network the device will
#               connect to it.
#
# @run example:
#
#            python set_wifi_state_by_airplane.py -s 0BA8F2A0
#
# @author:      corneliu.stoicescu@intel.com
#
#######################################################################
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_steps
import sys

globals().update(vars(get_args(sys.argv)))

# turn display on, if turned off
ui_steps.wake_up_device(serial=serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial=serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial=serial)()

# set airplane mode OFF
wifi_steps.set_am_state(serial=serial, state="off")()

# set wifi state on
wifi_steps.set_wifi_state(serial=serial, state="ON")()

# set airplane mode ON
wifi_steps.set_am_state(serial=serial, state="on")()

# open wifi settings
wifi_steps.open_wifi_settings(serial=serial)()
# check wifi settings
ui_steps.wait_for_view(serial=serial, view_to_find={"text": "Off"})()
# set airplane mode OFF
wifi_steps.set_am_state(serial=serial, state="off")()

ui_steps.wait_for_view(serial=serial, view_to_find={"text": "On"})()