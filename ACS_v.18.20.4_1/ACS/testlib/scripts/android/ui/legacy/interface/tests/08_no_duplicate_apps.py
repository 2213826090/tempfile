#!/usr/bin/env python

##############################################################################
#
# @filename:
# @description: ET/Star Peak/Script Library/L2/Interface/08 - Applications
#                   are no duplicated in app view
#               Check if apps in all apps have unique names.
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui.interface import utils
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

ui_steps.press_all_apps(print_error = "Error - The connection to the device "
                                      "was not established", blocking = True)()

class confirm_no_duplicate_app_names_appear_in_all_apps (ui_step):
    def do(self):
        self.unique = utils.check_apps_are_unique()
    def check_condition(self):
        return self.unique

confirm_no_duplicate_app_names_appear_in_all_apps()()

ui_steps.press_home()()

