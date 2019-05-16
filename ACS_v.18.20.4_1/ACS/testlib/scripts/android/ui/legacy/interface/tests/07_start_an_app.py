#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: ET/Star Peak/Script Library/L2/Interface/07 - Start
#                   application from app view
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

from testlib.base.base_utils import get_args
import sys

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_app_from_allapps(print_error = "Error - Application coulnd not "
                                             "be openned",
                               view_to_find = {"text":"Calculator"},
                               wait_time = 3000,
                               view_to_check = {"text": "cos"})()

ui_steps.press_home()()

