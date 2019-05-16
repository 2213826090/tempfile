#!/usr/bin/env python

# #############################################################################
#
# @filename:    safe_mode.py
#
# @description: Boot device into Safe Mode
#
# @author:      claudiu.i.lataretu@intel.com
#
##############################################################################

from testlib.scripts.security.scripts import prerequisites
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import sys

globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
widevine_apk = args["widevine_apk"]

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234")()

#Install any application
adb_steps.install_apk(serial = serial,
                      apk_path = widevine_apk)()

ui_steps.find_app_from_allapps(serial = serial,
                               presence = True,
                               view_to_find = {"textContains": "ExoPlayer"})()

#Reboot into safe mode
adb_steps.reboot(serial = serial,
                 safe_mode = True)()

ui_steps.find_app_from_allapps(serial = serial,
                               presence = False,
                               view_to_find = {"textContains": "ExoPlayer"})()

adb_steps.reboot(serial = serial)()
