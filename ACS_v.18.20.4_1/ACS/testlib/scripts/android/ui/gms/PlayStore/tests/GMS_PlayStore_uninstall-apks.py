#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: PlayStore / Uninstall app
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.PlayStore import playstore_steps
from testlib.scripts.gms import gms_utils

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils
args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.close_all_app_from_recent(serial = serial)()

if (gms_utils.get_google_account_number(serial = serial) == 0):
    ui_steps.add_google_account_for_L(serial = serial,version = "L",\
                 account = account,
                 password = password)()
playstore_steps.uninstall_playstore_app(serial = serial,
                                        app_name = "Spanzuratoarea",
                                        app_description = "HB SimApps",
                                        app_uninstall_time = 120)()

ui_steps.press_back(serial = serial,
                    times = 2)()

ui_steps.close_all_app_from_recent(serial = serial)()
