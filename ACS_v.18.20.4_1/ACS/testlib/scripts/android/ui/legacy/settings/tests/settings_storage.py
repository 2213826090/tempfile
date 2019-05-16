#!/usr/bin/env python

##############################################################################
#
# @filename:    settings_storage.py
#
# @description: Open the Storage menu. ET TC only checks "no error appears", I'll be assuming that test is pass
#               if the menu opens and a text is found at the begging and another one a the end of the list.
#               If texts are not found, then probably there's an error popup.
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

ui_steps.open_settings(print_error = "Error - Settings page was not "
                                     "displayed")()
ui_steps.click_button(print_error = "Failed to open Storage",
                      view_to_find = {"text":"Storage"},
                      view_to_check = {"text":"Total space"})()

ui_utils.is_text_visible(text_to_find = "Erase SD card")

ui_steps.press_home()()

