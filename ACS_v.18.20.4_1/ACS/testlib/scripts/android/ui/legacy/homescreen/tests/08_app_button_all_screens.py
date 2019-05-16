#!/usr/bin/env pythonpp button present in all screens
# #############################################################################
#
# @filename:
#
# @description: HomeScreen / App button present in all screens
#
# @author:      danielx.m.ciocirlan@intel.com
#
##############################################################################

import sys
import time

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.utils.ui.uiandroid import UIDevice as ui_device

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

uidevice = ui_device()
app = uidevice(className='android.widget.FrameLayout')
x_coord = app.info['bounds']['right']
y_coord = app.info['bounds']['bottom']
ui_steps.press_home()()

for i in range(1,4):
    ui_steps.press_all_apps()()
    ui_steps.press_back()()
    ui_steps.swipe(sx = x_coord-500, sy = y_coord/2, ex = x_coord, ey = y_coord/2)()
    time.sleep(1)

ui_steps.press_home()()

for i in range(1,4):
    ui_steps.press_all_apps()()
    ui_steps.press_back()()
    ui_steps.swipe(sx = x_coord-10, sy = y_coord/2, ex = x_coord-500, ey = y_coord/2)()
    time.sleep(1)

