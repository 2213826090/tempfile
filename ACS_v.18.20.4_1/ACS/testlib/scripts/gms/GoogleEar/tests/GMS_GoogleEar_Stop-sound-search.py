#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Ears / Cancel Google Ears search
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys
import time

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

# Connect to device
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.wake_up_device(serial = serial)()
ui_steps.unlock_device_swipe(serial = serial)()
ui_steps.press_home(serial = serial)()
adb_steps.enable_package_from_pm(serial = serial,
                                 package_name = "com.google.android.ears",
                                 app_name = "Sound Search",
                                 is_widget = True)()
#check the pm package is enabled!!!!
ui_steps.add_widget_to_homescreen(serial = serial,
                                  widget_name = "Sound Search",
                                  displayed_name = "What's this song")()

ui_steps.click_button(serial = serial,
                      view_to_find = {"textContains": "What's this song"},
                      wait_time = 3000,
                      view_to_check = {"resourceId": "com.google.android.ears:id/capture_animation_imageview"})()
time.sleep(2)
ui_steps.click_button(serial = serial,
                      view_to_find = {"resourceId": "com.google.android.ears:id/capture_animation_imageview"},
                      wait_time = 3000,
                      view_to_check = {"textContains": "What's this song"})()
ui_steps.press_home(serial = serial)()
