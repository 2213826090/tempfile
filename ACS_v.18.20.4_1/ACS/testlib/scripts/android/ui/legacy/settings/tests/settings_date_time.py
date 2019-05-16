#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Toggle several options Date & time
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui.ui_step import step as ui_step

import sys                 
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.open_settings_app(
                   print_error = "Error - Settings page was not displayed",
                   blocking = True,
                   view_to_find = {"text":"Date & time"},
                   view_to_check = {"text": "Automatic date & time"}
                          )()

if ui_utils.is_checkbox_checked({"text":"Automatic date & time"}):
    ui_steps.click_button(print_error = "Failed to open Automatic date & time",
                      blocking = True,
                      view_to_find = {"text":"Automatic date & time"})()

if version == "K":
    submit_text = "Done"
elif version == "L":
    submit_text = "OK"

ui_steps.click_button(print_error = "Failed to open Set date",
                      view_to_find = {"text":"Set date"},
                      view_to_check = {"text":submit_text},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to open close calendar",
                      view_to_find = {"text":submit_text},
                      view_to_check = {"text":"Set date"},
                      critical = False)()

ui_steps.click_button(print_error = "Failed to open Set time",
                      view_to_find = {"text":"Set time"},
                      view_to_check = {"text":submit_text},
                      critical = False)()
ui_steps.click_button(print_error = "Failed to close calendar",
                      view_to_find = {"text":submit_text},
                      view_to_check = {"text":"Set time"},
                      critical = False)()

#enable back automatic date & time
ui_steps.click_button(print_error = "Failed to check Automatic date & time",
                      blocking = True,
                      view_to_find = {"text":"Automatic date & time"},
                      critical = False)()

# check if date and time are now disabled
class confirm_disabled_buttons(ui_step):
    def do(self):
        self.date = ui_utils.is_enabled(view_to_find = {"text":"Set date"},
                                  enabled = False)
        self.time = ui_utils.is_enabled(view_to_find = {"text":"Set time"},
                                  enabled = False)
    def check_condition(self):
        return not self.date and not self.time
confirm_disabled_buttons()()

ui_steps.click_button(print_error = "Failed to open Select time zone",
                      view_to_find = {"text":"Select time zone"},
                      view_to_check = {"description":"More options"},
                      critical = False)()
ui_utils.is_text_visible(text_to_find = "Cairo") #because sometimes the list is scrolled
ui_steps.click_button(print_error = "Failed to open choose a time zone",
                      view_to_find = {"textContains":"Cairo"},
                      view_to_check = {"text":"Automatic date & time"},
                      critical = False)()

#change time format
if not ui_utils.is_checkbox_checked({"text":"Use 24-hour format"}):
    ui_steps.click_button(print_error = "Failed to check Use 24-hour format",
                      blocking = True,
                      view_to_find = {"text":"Use 24-hour format"},
                      view_to_check = {"text":"13:00"})()

class confirm_time_format(ui_step):
    def __init__(self, text_to_find, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.text_to_find = text_to_find
    def do(self):
        self.step_data = ui_utils.is_text_visible(text_to_find = self.text_to_find)
    def check_condition(self):
        return self.step_data
confirm_time_format(text_to_find = "13:00")()
# ui_utils.is_text_visible(text_to_find = "13:00")

ui_steps.click_button(print_error = "Failed to un-check Use 24-hour format",
                      blocking = True,
                      view_to_find = {"text":"Use 24-hour format"},
                      critical = False)()

# ui_utils.is_text_visible(text_to_find = "1:00 PM")
confirm_time_format(text_to_find = "1:00 PM")()


ui_steps.click_button(print_error = "Failed to open Choose date format",
                      view_to_find = {"text":"Choose date format"},
                      view_to_check = {"textContains":"Regional"},
                      critical = False)()

ui_steps.click_button(print_error = "Failed to select format",
                      view_to_find = {"resourceId":"android:id/text1",
                                      "instance":"3"},
                      view_to_check = {"text":"Choose date format"},
                      critical = False)()
ui_steps.press_back()()
ui_steps.press_back()()
ui_steps.press_home()()

