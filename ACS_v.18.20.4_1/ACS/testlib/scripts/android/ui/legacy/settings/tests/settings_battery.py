#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Open battery menu.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################


from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

import sys
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_settings(print_error = "Error - Settings page was not displayed",
                       blocking = True)()

ui_steps.click_button(print_error = "Failed to open Battery",
                      blocking = True,
                      view_to_find = {"text":"Battery"},
                      view_to_check = {"text":"Refresh"})()

ui_steps.press_home()()

