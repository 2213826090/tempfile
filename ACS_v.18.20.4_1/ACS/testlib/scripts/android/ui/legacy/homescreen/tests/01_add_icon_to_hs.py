#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Add icon to homescreen
#
# @author:      danielx.m.ciocirlan@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step

# Connect to device
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home(serial = serial)()

# Add icon to home screen
ui_steps.add_app_from_all_apps_to_homescreen(serial = serial, view_text = "Calculator")()

ui_steps.press_home(serial = serial)()

