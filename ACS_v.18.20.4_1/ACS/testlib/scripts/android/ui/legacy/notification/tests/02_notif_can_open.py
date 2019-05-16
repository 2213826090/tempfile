#!/usr/bin/env python

##############################################################################
#
# @filename:    .py
#
# @description: Testing notification area: open NotificationTrigger app,
#               create a notification, open the notification area.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################


from testlib.scripts.android.ui.notification import steps
from testlib.scripts.android.ui import ui_steps
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
                      "Notification not found after opening the notification "
                      "area.",
                      blocking = True)()

ui_steps.press_home()()

