#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Open system utils area from top bar
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

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

class test_verification(ui_step):

    def do(self):
        self.set_passm("Open system utils area from top bar")
        self.set_errorm("", "Open system utils area from top bar")
        app = self.uidevice(className='android.widget.FrameLayout')
        x_coord = app.info['bounds']['right']
        y_coord = app.info['bounds']['bottom']
        ui_steps.swipe(sx = x_coord - 10,
                       sy = 1,
                       ex = x_coord - 10,
                       ey = y_coord - 100)()
        if version == "L":
            ui_steps.swipe(sx = x_coord - 10,
                           sy = 1,
                           ex = x_coord - 10,
                           ey = y_coord - 100)()


    def check_condition(self):
        import time
        self.uidevice(textContains = "Wi-Fi").wait.exists(timeout = 20000)
        return self.uidevice(textContains = "Wi-Fi").exists and self.uidevice(textContains = "Bluetooth").exists

ui_steps.press_home()()

test_verification()()

ui_steps.press_home()()

