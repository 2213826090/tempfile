#!/usr/bin/env python

##############################################################################
#
# @filename:
# @description: ET/Star Peak/Script Library/L2/Interface/04 - Application view pagination
#               Test if you can reach the second page of installed apps by swiping.
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui.interface import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
import os


# you need to install several apps in order to fill the first screen so than
# the second one appears. Might need to install more according to screen size.
# below apps can be found at cats-mirror/common/apk
"""
apps_to_install = ["Quickoffice.apk",
                   "com.twitter.android-1.apk",
                   "co.vine.android-1.apk",
                   "com.ebay.mobile-1.apk",
                   "com.facebook.katana-1.apk",]
"""
import sys
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port,
)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.press_all_apps(print_error = "Error - failed to open all apps screen",
                        blocking = True)()

apk_folder = media_path
apps_to_install = apks
if apps_to_install:
    for app in apps_to_install:
        apk_path = os.path.join(apk_folder, app)
        adb_steps.install_apk(print_error = "Error - App was not installed",
                              blocking = False,
                              apk_path = apk_path,
                              debug = True)()

ui_steps.swipe(print_error = "Swipe error",
               sx = 300, sy = 400, ex = 100, ey = 400, steps = 10,
               blocking = True,
               view_to_check =  {"textContains": "YouTube"})()

ui_steps.press_home()()

