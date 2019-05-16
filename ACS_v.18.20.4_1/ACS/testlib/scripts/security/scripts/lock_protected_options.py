#!/usr/bin/env python

# #############################################################################
#
# @filename:    lock_protected_options.py
#
# @description: Check if Smart Lock, OEM Unlock and Screen Lock requires
#               PIN if accessed
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.utils.statics.android import statics
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.security.scripts import prerequisites
import sys
from testlib.base.base_utils import get_args
globals().update(vars(get_args(sys.argv)))

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                  pin = "1234",
                  set_screen_lock = True)()

# Check if OEM Unlock requires PIN
adb_steps.enable_developer_options(serial = serial)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "OEM unlocking"},
                      view_to_check = {"text": statics.Device(serial = serial)
                                               .confirm_view_pin_oem_unlock})()

# Check if Smart Lock requires PIN
ui_steps.open_security_settings(serial = serial)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "Smart Lock"},
                      view_to_check = {"text": "Confirm your PIN"})()

# Check if Screen Lock requires PIN
ui_steps.open_security_settings(serial = serial)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "Screen lock"},
                      view_to_check = {"text": "Confirm your PIN"})()
