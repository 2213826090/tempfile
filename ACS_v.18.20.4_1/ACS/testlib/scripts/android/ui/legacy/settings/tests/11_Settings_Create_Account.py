#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Create an account
#               Add a google account to the device
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

import sys
from testlib.scripts.android.ui.settings import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args


globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.add_google_account(version = version,
                            account = "intelchat002@gmail.com",
                            password = "intel002")()

ui_steps.press_home()()
