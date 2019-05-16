#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Apps
#               Open Running and All apps and check the presence of one app
#                   in each list.
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

ui_steps.open_app_from_settings(view_to_find = {"text": "Apps"},
                                wait_time = 2000,
                                view_to_check = {"text":"Downloaded"})()

ui_steps.swipe(sx = 500, sy = 300, ex = 200, ey = 300, steps = 10,
               view_presence = True, view_to_check = {"text":"Settings"},
               wait_time = 5000)()

ui_steps.swipe(sx = 500, sy = 300, ex = 200, ey = 300, steps = 10,
               view_presence = True, view_to_check = {"text":"Android "
                                                             "System"},
               wait_time = 5000)()

ui_steps.press_home()()


