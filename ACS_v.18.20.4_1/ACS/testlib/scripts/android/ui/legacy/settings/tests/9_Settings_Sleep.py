#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Display Sleep
#               Choose a display timeout time.
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
                                view_to_check = {"textContains":"Brightness"})()

ui_steps.click_button(view_to_find = {"text":"Sleep"},
                     view_to_check= {"text":"Cancel"})()

ui_steps.click_button(view_to_find = {"text":"10 minutes"},
                     view_to_check= {"text":"After 10 minutes of "
                                            "inactivity"})()

ui_steps.press_home()()
