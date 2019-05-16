#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: PlayStore / Add app to wishlist
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.playstore import playstore_steps
from testlib.scripts.gms.playstore import playstore_utils

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.open_playstore(serial = serial)()
playstore_utils.playstore_home(serial = serial)

ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "APPS"},
                      wait_time = 10000,
                      view_to_check = {"text": "New + Updated Apps"})()
playstore_steps.playstore_add_to_wishlist(serial = serial,
                                          random_app = True)()
ui_steps.press_back(serial = serial,
                    times = 2)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "GAMES"},
                      wait_time = 10000,
                      view_to_check = {"text": "Browse our go-to games"})()
playstore_steps.playstore_add_to_wishlist(serial = serial,
                                          random_app = True)()
ui_steps.press_back(serial = serial)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "BOOKS"},
                      wait_time = 10000,
                      view_to_check = {"text": "Recommended for You"})()
playstore_steps.playstore_add_to_wishlist(serial = serial,
                                          random_app = True)()
ui_steps.press_back(serial = serial,
                    times = 2)()

ui_steps.press_home(serial = serial)()
