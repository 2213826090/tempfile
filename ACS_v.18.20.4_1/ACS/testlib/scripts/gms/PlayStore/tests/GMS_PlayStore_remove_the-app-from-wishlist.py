#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: PlayStore / Remove entry from wishlist
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.PlayStore import playstore_steps
from testlib.scripts.gms.PlayStore import playstore_utils
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
                 password = password, open_from_settings = False,\
                 from_gmail = True)()

ui_steps.open_playstore(serial = serial)()

playstore_utils.playstore_home(serial = serial)

ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "APPS"},
                      view_to_check = {"textContains": "Updated Apps"})()

playstore_steps.playstore_add_to_wishlist(serial = serial,
                                          random_app = True)()
ui_steps.press_back(serial = serial)()
playstore_steps.playstore_remove_from_wishlist(serial = serial,
                                               random_app = True)()

ui_steps.press_home(serial = serial)()
