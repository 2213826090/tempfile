#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Start app with enter key
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

class start_app_with_enter_key(ui_step):
    def __init__(self, view_text, check_text, **kwargs):
        self.view_text = view_text
        self.check_text = check_text
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.add_app_from_all_apps_to_homescreen(view_text =
                                                     self.view_text)()
        self.uidevice.press("down")
        self.uidevice.press("enter")

    def check_condition(self):
        return self.uidevice(text = self.check_text).exists

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

start_app_with_enter_key(view_text = "Calculator",
                         check_text = "sin")()

ui_steps.press_home()()

