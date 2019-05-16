#!/usr/bin/env python

######################################################################
#
# @filename:    flight_mode.py
# @description: Tests that flight mode behaves correctly in relation
#               to Wi-Fi
#               test types: turn_am_on / turn_am_off / turn_wifi_on / wifi_off_toggle_am
#
# @run example:
#
#            python flight_mode.py -s 0BA8F2A0
#                                               --script-args
#                                                       test_type=am_turn_on
#                                                       wait_time=5
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args:
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val

# mandatory params
test_type = args["test_type"]

# optional params
wait_time = 5
if "wait_time" in args.keys():
    wait_time = args["wait_time"]

##### test start #####
# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

class flight_mode_test(adb_step):
    """ description:

        test_type(s): turn_am_on, turn_am_off, turn_wifi_on
    """
    def __init__(self, test_type = 'turn_am_on', wait_time = 5, **kwargs):
        adb_step.__init__(self, **kwargs)
        self.test_type = test_type
        self.wait_time = int(wait_time)

    def do(self):
        # go to expected/default settings
        wifi_steps.set_am_state(serial = self.serial, state='off')()
        if self.test_type == 'wifi_off_toggle_am':
            wifi_steps.set_wifi_state(serial = self.serial, state='OFF')()
        else:
            wifi_steps.set_wifi_state(serial = self.serial, state='ON')()
        adb_steps.command(serial = self.serial, command = "input keyevent 4", mode = "sync", timeout=self.wait_time)()

        wifi_steps.set_am_state(serial = self.serial, state='on')()
        if self.test_type == 'turn_wifi_on':
            wifi_steps.set_wifi_state(serial = self.serial, state='ON')()
            adb_steps.command(serial = self.serial, command = "input keyevent 4", mode = "sync", timeout=self.wait_time)()
        if self.test_type in ['turn_am_off', 'wifi_off_toggle_am']:
            wifi_steps.set_am_state(serial = self.serial, state='off')()

    def check_condition(self):
        for counter in range(5):
            self.process = self.adb_connection.run_cmd(command = "dumpsys wifi | grep Wi-Fi",
                                ignore_error = False,
                                timeout = self.wait_time,
                                mode = "sync")
            output = self.process.stdout.read()
            if self.test_type == 'turn_am_on':
                if 'disabled' in output:
                    return True
                self.set_errorm("","'disabled' not found in dumpsys wifi - after turning am ON")
            elif self.test_type == 'turn_am_off':
                if 'enabled' in output:
                    return True
                self.set_errorm("","'enabled' not found in dumpsys wifi - after turning am OFF")
            elif self.test_type == 'turn_wifi_on':
                if 'enabled' in output:
                    self.set_errorm("","wifi was enabled, but the AM turned off - after turning am then wifi ON")
                    return wifi_utils.check_airplane_mode_on(serial = self.serial)
                self.set_errorm("","'enabled' not found in dumpsys wifi - after turning am then wifi ON")
            elif self.test_type == 'wifi_off_toggle_am':
                if 'disabled' in output:
                    return True
                self.set_errorm("","'disabled' not found in dumpsys wifi - after wifi off, turn am ON then OFF")
            time.sleep(1)
        return False

flight_mode_test(serial = serial, test_type = test_type, wait_time = wait_time)()
