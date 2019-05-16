#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Google Bar present HomeScreen
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

# Connect to device

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

class test_verification(ui_step):

    def do(self):
        self.set_passm("Google Bar present HomeScreen")
        self.set_errorm("", "Google Bar present HomeScreen")
        ui_steps.press_home()()

    def check_condition(self):
        return self.uidevice(descriptionContains = "Search").exists

test_verification()()

ui_steps.press_home()()

