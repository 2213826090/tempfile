#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Display Brightness
#               Change Brightness using slider and compare with value found
#               in file on disk.
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

import sys
from testlib.scripts.android.ui.settings import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()
ui_steps.open_settings()()

ui_steps.open_app_from_settings(view_to_find = {"text": "Display"},
                                wait_time = 2000,
                                view_to_check = {"textContains":"Brightness"})()

ui_steps.click_button(view_to_find = {"textContains":"Brightness"},
                     view_to_check= {"className":
                                         "android.widget.SeekBar"})()

steps.slide_brightness(view_to_find = {"className":'android.widget.SeekBar',
                                       "instance":0}, position = 30)()

ui_steps.press_home()()
