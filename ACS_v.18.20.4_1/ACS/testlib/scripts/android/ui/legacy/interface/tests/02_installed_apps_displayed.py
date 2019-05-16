#!/usr/bin/env python

##############################################################################
#
# @filename:
# @description: ET/Star Peak/Script Library/L2/Interface/02-Installed
#               apps displayed
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

import sys
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.press_all_apps(print_error = "Error - The connection to the "
                                      "device was not established")()

ui_steps.press_home()()

