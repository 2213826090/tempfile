#!/usr/bin/env python

##############################################################################
#
# @filename:    .py
#
# @description: Install File Manager which is needed by all storage 
#               tests.
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
import sys

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.add_google_account(version = version)()
ui_steps.install_playstore_app(app_name = "FileManager",
                               app_description = "File Manager (Explorer)",
                               app_install_time = 10000)()

ui_steps.press_home()()

