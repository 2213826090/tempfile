#!/usr/bin/env python

##############################################################################
#
# @description: ET/../Settings/Settings Wireless & Networks more...
#               Open More for Wireless
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
ui_steps.open_settings()()

ui_steps.open_app_from_settings(view_to_find = {"textContains": "More"},
                                wait_time = 3000,
                                view_to_check = {"text":"VPN"})()

ui_steps.press_home()()

