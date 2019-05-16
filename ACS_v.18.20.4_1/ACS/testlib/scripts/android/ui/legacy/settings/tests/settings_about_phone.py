#!/usr/bin/env python

##############################################################################
#
# @filename:    settings_about_phone.py
#
# @description: Testing the About area.
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

ui_steps.open_settings()()
ui_steps.open_app_from_settings(view_to_find = {"text":"About tablet"},
                                view_to_check = {"text":"Build number"})()

if version == "K":
    ui_steps.click_button(print_error = "Failed to click Intel System Software "
                                        "Auto-Update",
                          view_to_find = {"text":"Intel System Software "
                                                 "Auto-Update"},
                          view_to_check = {"textContains":"Auto-Update "
                                                          "determines the best"},
                          critical = False)()
    ui_steps.click_button(print_error = "Failed to click Check for Updates",
                          view_to_find = {"text":"Check for Updates"},
                          view_to_check = {"text":"Intel System Software "
                                                  "Auto-Update"},
                          critical = False)()

ui_steps.click_button(print_error = "Failed to open Status",
                      view_to_find = {"text":"Status"},
                      view_to_check = {"text":"Battery status"},
                      critical = False)()
ui_steps.press_back()()

ui_steps.click_button(print_error = "Failed to open Status",
                      view_to_find = {"text":"Legal information"},
                      view_to_check = {"text":"Open source licenses"},
                      critical = False)()

if version == "L":
    open_source_text = "/kernel"
elif version == "K":
    open_source_text = "/data/app"

ui_steps.click_button(print_error = "Failed to open Status",
                      view_to_find = {"text":"Open source licenses"},
                      wait_time = 220000,
                      view_to_check = {"descriptionContains":open_source_text},
                      critical = False)()
ui_steps.press_back()()

if version == "K":
    ui_steps.click_button(print_error = "Failed to open Status",
                          view_to_find = {"text":"Google legal"},
                          view_to_check = {
                              "resourceId":"android:id/alertTitle"},
                          critical = False)()
ui_steps.press_back()()

ui_steps.press_home()()

