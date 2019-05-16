#!/usr/bin/env python

##############################################################################
#
# @filename:    settings_display_cast_screen.py
#
# @description: open Cast Screen panel.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

import sys                 
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_settings()()

ui_steps.click_button(print_error = "Failed to open Display",
                      blocking = True,
                      view_to_find = {"text":"Display"},
                      view_to_check = {"text":"Cast screen"})()

if version == "L":
    ui_steps.click_button(print_error = "Failed to open Cast screen",
                          view_to_find = {"text":"Cast screen"},
                          view_to_check = {"text":"Cast screen"})()

ui_steps.press_home()()

