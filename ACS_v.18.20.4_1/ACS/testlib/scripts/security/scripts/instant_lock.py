#!/usr/bin/env python

# #############################################################################
#
# @filename:    instant_lock.py
#
# @description: Checks instant lock feature
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_screen_lock = True)()

ui_steps.open_security_settings(serial = serial)()

# Turn on instant lock
ui_steps.click_checkbox_button(serial = serial,
                    view_to_find = {"text": "Power button instantly locks"},
                    is_switch = True,
                    state = "ON",
                    relationship = "right")()

# Check the feature
adb_steps.put_device_into_sleep_mode(serial = serial)()
time.sleep(5)
adb_steps.wake_up_device(serial = serial)()
ui_steps.unlock_device_swipe(serial = serial)()
ui_steps.unlock_device_pin(serial = serial, pin = "1234")()
