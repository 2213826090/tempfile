#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Add icon to homescreen
#
# @author:      ion-horia.petrisor@intel.com
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

adb_steps.check_serial(serial_to_check = serial)()

