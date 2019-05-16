#!/usr/bin/env python

##############################################################################
#
# @filename:    .py
#
# @description: Testing notification area: open NotificationTrigger app,
#               create a notification, open the notification area,
#               swipe to dismiss the notification.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################
from testlib.scripts.android.ui.notification import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

import sys

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,  
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

steps.open_notif_area(print_error =
                      "Notification not found after opening the notification area.",
                      blocking = True)()
sx, sy = ui_utils.get_view_middle_coords({"textContains": "Your test"})

ui_steps.swipe(print_error = "Swipe error",
               sx = sx, sy = sy, ex = sx + 200, ey = sy, steps = 10,
               blocking = True)()

steps.open_notification(print_error = "Open Notification area error",
                        view_to_check = {"textContains":
                                             "Your test notification title"},
                        should_exist = False)()

ui_steps.press_home()()

