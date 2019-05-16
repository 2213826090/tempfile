#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Open left notification area from top bar
#
# @author:      danielx.m.ciocirlan@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.base.base_utils import get_args


class open_left_notification_area(ui_step):

    def __init__(self, view_to_find, **kwargs):
        self.view_to_find = view_to_find
        ui_step.__init__(self, **kwargs)
        self.set_passm("Open left notification area from top bar")
        self.set_errorm("", "Open left notification area from top bar")

    def do(self):
        layout = self.uidevice(className='android.widget.FrameLayout')
        x_coord = layout.info['bounds']['right']
        y_coord = layout.info['bounds']['bottom']
        ui_steps.swipe(sx = 1, sy = 0, ex = 1, ey = y_coord - 100)()

    def check_condition(self):
        return self.uidevice(**self.view_to_find).wait.exists(timeout = 20000)

# Connect to device
globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

open_left_notification_area(view_to_find =
                            {"resourceId": "com.android.systemui:id/battery"})()

ui_steps.press_home()()

