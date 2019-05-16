#!/usr/bin/env python

######################################################################
#
# @filename:    flight_mode.py
# @description: Tests that all radios can be turned on without problems
#               ( wifi / bluetooth / NFC - if present )
#               test types: <none> / airplane_mode
#
# @run example:
#
#            python radios_coexistence.py -s 0BA8F2A0
#                                               --script-args
#                                                       wait_time=10
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.bluetooth import bluetooth_utils
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args:
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val


# optional params
test_type = None
if "test_type" in args.keys():
    test_type = args["test_type"]
wait_time = 5
if "wait_time" in args.keys():
    wait_time = int(args["wait_time"])
iterative = 1
if "iterative" in args.keys():
    wait_time = int(args["iterative"])

##### test start #####

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()
wifi_steps.set_am_state(serial = serial, state='off')()

# turn WiFi on
wifi_steps.set_wifi_state(serial = serial, state='ON')()

# turn BT on
adb_steps.command(serial = serial, command = "am start -n com.android.settings/.bluetooth.BluetoothSettings", timeout = wait_time)()
ui_steps.click_switch(serial = serial, view_to_find = {"className": "android.widget.Switch"}, state = "ON")()

# open More Wireless & networks page
adb_steps.command(serial = serial, command = "am start -a android.settings.WIRELESS_SETTINGS", timeout = wait_time)()
if ui_utils.view_exists(serial = serial, view_to_find = {"text":"NFC"}):
    ui_steps.click_switch(serial = serial, view_to_find = {"text":"NFC", "resourceId":"android:id/title"}, state = "ON", right_of = True )()


class check_radios_behaviour_airplane_mode(adb_step):

    def __init__(self, state = 'on', wait_time = 5, **kwargs):
        adb_step.__init__(self, **kwargs)
        self.state = state
        self.wait_time = wait_time

    def do(self):
        wifi_steps.set_am_state(serial = self.serial, state=self.state, wait_time = self.wait_time)()

    def check_condition(self):
        if self.state.lower() == 'on':
            c=0
            while c < self.wait_time:
                if ui_utils.view_exists(serial = serial, view_to_find = {"text":"NFC"}):
                    if not ( wifi_utils.check_wifi_state_on(serial = self.serial) or bluetooth_utils.check_bluetooth_state_on(serial = self.serial) or ui_utils.is_switch_on(serial = self.serial, view_to_find = {"text":"NFC", "resourceId":"android:id/title"}, right_of = True) ):
                        return True
                else:
                    if not ( wifi_utils.check_wifi_state_on(serial = self.serial) or bluetooth_utils.check_bluetooth_state_on(serial = self.serial) ):
                        return True
                c+=1
                time.sleep(1)
            else:
                self.set_errorm("", 'Not all radios have been turned off by airplane mode:')
                return False

        elif self.state.lower() == 'off':
            c=0
            while c < self.wait_time:
                if ui_utils.view_exists(serial = serial, view_to_find = {"text":"NFC"}):
                    if ( wifi_utils.check_wifi_state_on(serial = self.serial) and bluetooth_utils.check_bluetooth_state_on(serial = self.serial) and ui_utils.is_switch_on(serial = self.serial, view_to_find = {"text":"NFC", "resourceId":"android:id/title"}, right_of = True) ):
                        self.set_passm('All the radios have been turned back on after '+str(c)+' cycles')
                        return True
                else:
                    if wifi_utils.check_wifi_state_on(serial = self.serial) and bluetooth_utils.check_bluetooth_state_on(serial = self.serial):
                        self.set_passm('All the radios have been turned back on after '+str(c)+' cycles')
                        return True
                c+=1
                time.sleep(1)
            else:
                self.set_errorm("", 'Not all radios have been turned back on by airplane mode')
                return False


# check if all the radios are (still) enabled
check_radios_behaviour_airplane_mode(serial = serial, state = 'off', wait_time = wait_time)()

# check if the radios behave correctly with airplane mode
if test_type == "airplane_mode":

    for i in range(iterative):

        check_radios_behaviour_airplane_mode(serial = serial, state = 'on', wait_time = wait_time)()

        check_radios_behaviour_airplane_mode(serial = serial, state = 'off', wait_time = wait_time)()
