#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Toggle several options from Security
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui.settings import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

import sys
import time
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_settings(print_error = "Error - Settings page was not displayed")()
ui_steps.click_button(print_error = "Failed to open Security",
                      view_to_find = {"text":"Security"},
                      view_to_check = {"text":"Screen lock"})()

ui_steps.click_button(print_error = "Falied to open Device administrators",
                      view_to_find = {"text":"Device administrators"},
                      view_to_check = {"text":"Android Device Manager"})()
ui_steps.press_back()()
to_be_unchecked = True
if version == "K":
    if ui_utils.is_checkbox_checked(view_to_find = {"text":"Unknown sources"}):
        to_be_unchecked = False
if version == "L":
    if ui_utils.is_switch_on(view_to_find = {"resourceId": "android:id/switchWidget",
                                             "instance": 1}):
        to_be_unchecked = False
if to_be_unchecked:
    if version == "K":
        steps.click_checkbox_button_with_ok(print_error = "Failed to uncheck "
                                                      "Unknown sources",
                                        view_to_find = {"text":
                                                        "Unknown sources"})()
        #let's leave it checked
        steps.click_checkbox_button_with_ok(print_error = "Failed to check "
                                                  "Unknown sources",
                                    view_to_find = {"text":
                                                    "Unknown sources"},
                                    critical = False,
                                    blocking = False)()
    elif version == "L":
        steps.click_switch_button_with_ok(view_to_find = {"text":
                                                            "Unknown sources"},
                                          view_to_check = {"resourceId": 
                                                           "android:id/switchWidget",
                                                           "instance": 1},
                                          state = "ON",
                                          critical = False,
                                          blocking = False)()
        steps.click_switch_button_with_ok(view_to_find = {"text":
                                                            "Unknown sources"},
                                          view_to_check = {"resourceId":
                                                           "android:id/switchWidget",
                                                           "instance": 1},
                                          state = "OFF",
                                          critical = False,
                                          blocking = False)()

if version == "K":
    steps.click_checkbox_button_with_ok(print_error = "Failed to check "
                                                       "Verify apps",
                                        view_to_find = {"text":"Verify apps"},
                                        critical = False,
                                        blocking = False)()

    steps.click_checkbox_button_with_ok(print_error = "Failed to uncheck "
                                                      "Verify apps",
                                        view_to_find = {"text":"Verify apps"},
                                        critical = False,
                                        blocking = False)()

ui_utils.is_text_visible(text_to_find = "Trusted credentials")
ui_steps.click_button(print_error = "Failed to open Trusted credentials",
                      view_to_find = {"text":"Trusted credentials"},
                      view_to_check = {"text":"System"},
                      critical = False,
                      blocking = False)()
ui_steps.press_back()()

ui_steps.click_button(print_error = "Failed to open Install from SD card",
                      blocking = False,
                      critical = False,
                      view_to_find = {"text":"Install from SD card"},
                      view_to_check = {"text":"Recent"})()

ui_steps.press_home()()

