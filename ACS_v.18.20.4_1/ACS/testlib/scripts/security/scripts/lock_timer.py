#!/usr/bin/env python

# #############################################################################
#
# @filename:    lock_timer.py
#
# @description: Checks the lock timer feature
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

class check_lock_timer(android_step):

    """ description:
            Checks if DUT locks immediately after going into sleep mode
            while have the "Instant lock" disabled

        usage:
            check_lock_timer()()

        tags:
            adb, android, lock, PIN
    """

    def __init__(self, timeout = 30, **kwargs):
        android_step.__init__(self, **kwargs)
        self.timeout = timeout

    def do(self):
        adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        time.sleep(self.timeout)
        adb_steps.wake_up_device(serial = self.serial)()

    def check_condition(self):
        return not ui_utils.is_device_locked(serial = self.serial)


if __name__ == "__main__":
    # Run Prerequisites
    prerequisites.run_prereq(serial = serial,
                             pin = "1234",
                             set_screen_lock = True)()

    ui_steps.open_security_settings(serial = serial)()

    # Turn off instant lock
    ui_steps.click_checkbox_button(serial = serial,
                        view_to_find = {"text": "Power button instantly locks"},
                        is_switch = True,
                        scrollable = False,
                        state = "OFF",
                        relationship = "right")()

    # Increase the time after lock
    ui_steps.click_button(serial = serial,
                          view_to_find = {"text": "Automatically lock"},
                          view_to_check = {"text": "Automatically lock"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"textContains": "5 minutes"},
                          view_to_check = {"text": "Screen lock"})()
    # Check the feature
    check_lock_timer(serial = serial, timeout = 30)()

    # Remake device state
    ui_steps.open_security_settings(serial = serial)()
    ui_steps.click_checkbox_button(serial = serial,
                        view_to_find = {"text": "Power button instantly locks"},
                        is_switch = True,
                        scrollable = False,
                        state = "ON",
                        relationship = "right")()
