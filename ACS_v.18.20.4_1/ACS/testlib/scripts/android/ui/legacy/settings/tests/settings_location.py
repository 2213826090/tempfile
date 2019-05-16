#!/usr/bin/env python

##############################################################################
#
# @filename:    settings_location.py
#
# @description: Check that you can turn on.
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
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_settings(print_error = "Error - Settings page was not displayed")()
ui_steps.click_button(print_error = "Failed to open Location", blocking = True,
                      view_to_find = {"text":"Location"},
                      view_to_check = {"text":"Mode"})()

ui_steps.click_switch(print_error = "Error - switching Location",
                      view_to_find = {"className":"android.widget.Switch"},
                      click_to_close_popup = {"text": "Agree"},
                      state = "ON",
                      index = 0)()

ui_steps.click_button(print_error = "Failed to open Mode",
                      view_to_find = {"text":"Mode"},
                      view_to_check = {"text":"High accuracy"})()

if version == "L":
    steps.click_radio_button_with_agree(view_to_find = {"text":"High accuracy"},
                                        critical = False,
                                        index = 0)()
    ui_steps.click_button(view_to_find = {"text":"Battery saving"},
                                        critical = False)()
    ui_steps.click_button(view_to_find = {"text":"Device only"},
                                        critical = False)()

if version == "K":
    steps.click_radio_button_with_agree(view_to_find = {"text":"High accuracy"},
                                        critical = False)()
    steps.click_radio_button_with_agree(view_to_find = {"text":"Battery saving"},
                                        critical = False)()
    steps.click_radio_button_with_agree(view_to_find = {"text":"Device only"},
                                        critical = False)()

ui_steps.press_home()()

