#!/usr/bin/env python

##############################################################################
#
# @filename:
# @description: ET/Star Peak/Script Library/L2/Interface/05 - Application view has a widget section
#               Swipe util you find the widget section.
# @author:      silviux.l.andrei@intel.com
#
##############################################################################


from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui.ui_step import step as ui_step

from testlib.base.base_utils import get_args
import sys

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

class confirm_visible_kk(ui_step):

    def do(self):
        ui_steps.press_all_apps(print_error = "Error - failed to open all apps "
                                              "screen")()

    def check_condition(self):
        return ui_utils.is_text_visible_scroll_left("Analog clock")


class confirm_widget_visible_l(ui_step):

    def do(self):
        app = self.uidevice(className='android.widget.FrameLayout')
        x_coord = app.info['bounds']['right']
        y_coord = app.info['bounds']['bottom']
        ui_steps.swipe(sx = x_coord/2, sy = y_coord/2, ex = x_coord/2, ey = y_coord/2)()
        ui_steps.click_button(view_to_find = {"textContains": "Widgets"})()

    def check_condition(self):
        return self.uidevice(text = "Analog clock").exists


if version == "L":
    confirm_widget_visible_l()()
else:
    confirm_widget_visible_kk()()

ui_steps.press_home()()

