#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Change font size found in Display menu.
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

ui_steps.open_settings(print_error = "Error - Settings page was not "
                                     "displayed")()

ui_steps.click_button(print_error = "Failed to open Display",
                      blocking = True, view_to_find = {"text":"Display"},
                      view_to_check = {"textContains":"Brightness"})()

ui_steps.click_button(print_error = "Failed to open Font size",
                      view_to_find = {"text":"Font size"},
                      view_to_check = {"text":"Small"},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to select font size",
                      view_to_find = {"text":"Small"},
                      view_to_check = {"text":"Small"},
                      critical = False)()

ui_steps.click_button(print_error = "Failed to open Font size",
                      view_to_find = {"text":"Font size"},
                      view_to_check = {"text":"Small"},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to select font size",
                      view_to_find = {"text":"Large"},
                      view_to_check = {"text":"Large"},
                      critical = False)()

ui_steps.click_button(print_error = "Failed to open Font size",
                      view_to_find = {"text":"Font size"},
                      view_to_check = {"text":"Small"},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to select font size",
                      view_to_find = {"text":"Huge"},
                      view_to_check = {"text":"Huge"},
                      critical = False)()

ui_steps.click_button(print_error = "Failed to open Font size",
                      view_to_find = {"text":"Font size"},
                      view_to_check = {"text":"Small"},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to select font size",
                      view_to_find = {"text":"Normal"},
                      view_to_check = {"text":"Normal"},
                      critical = False)()

ui_steps.press_home()()

