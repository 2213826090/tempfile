#!/usr/bin/env python

# #############################################################################
#
# @filename:    brute_force_deterrent.py
#
# @description: Unlocks the DUT with wrong PIN 5 times in a row
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

# Block device (Enter wrong PIN 5 times in a row)
adb_steps.block_device(serial=serial,
                       pin="2222")()

# Timeout for entering 5 times in a row the wrong PIN
time.sleep(30)

# Remake initial state
adb_steps.wake_up_device(serial = serial)()
ui_steps.click_button_if_exists(serial = serial,
                                view_to_find = {"text": "OK"})()
adb_steps.unlock_device(serial=serial,
                        pin="1234")()
