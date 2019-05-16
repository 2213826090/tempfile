#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Settings/Daydream
#
# @author:      danielx.m.ciocirlan@intel.com
#
##############################################################################
import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.base.base_step import step as base_step

class test_verification(base_step):

    """ description:
            click on an option and verify that option has been
            selected.Optional add "instance" if 2 views with
            same text

        usage:
            test_verification(view_text_to_find,instance)

        tags:
            daydream, settings
    """

    def __init__(self, view, view_index = 0, radio_index = 0, **kwargs):
        self.view = view
        self.radio_instance = radio_index
        self.view_instance = view_index
        base_step.__init__(self, **kwargs)
    def do(self):
        ui_steps.click_button(
                              print_error="Failed to click" + str(self.view),
                              view_to_find={"text": str(self.view),
                                            "instance":self.view_instance
                                           },
                              view_to_check = {"text": str(self.view)}
        )()
    def check_condition(self):
        base_step.set_passm(self, str(self.view))
        return ui_utils.is_radio_button_enabled(instance = self.radio_instance)


# Connect to device
import sys                 
from testlib.base.base_utils import get_args
                      
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

# Open Display menu
ui_steps.open_display_from_settings(view_to_check = {"text":"Daydream"})()

# Click on Daydream
ui_steps.click_button(
    print_error = "Failed to open Daydream",
    view_to_find = {"text":"Daydream"},
    view_to_check = {"text":"Clock"}
)()

# Items to select in Daydream
if version == "L":
    options = ["Clock", "Colors", "Google Photos", "News & Weather", "Photo Frame", "Photo Table"]
elif version == "K":
    options = ["Clock", "Colours", "Google Photos", "Photo Frame", "Photo Table"]

# Click on each item and make sure option is checked
i = 0
for option in options:
    test_verification(view = option, radio_index = i)()
    if version == "K" and (option == "Photo Frame" or option == "Photo Table"):
        test_verification(view = option, radio_index = i, view_index = 1)()
        i += 1
    i += 1

ui_steps.press_back()()
ui_steps.press_back()()
ui_steps.press_back()()
ui_steps.press_home()()
