#!/usr/bin/env python

# #############################################################################
#
# @filename:    multi_user_external_storage_access.py
#
# @description: Check that new user cannot access data among initial
#               user (Owner)
#
# @author:      costin.carabas@intel.com
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
ap_name =  args["net_ap_ssid"]
ap_password = args["net_ap_password"]
widevine_apk = args["widevine_apk"]

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_wifi = True,
                         ap_name = "sp_gpt",
                         ap_password = "Starpeakqwe123!")()

ui_steps.press_home(serial = serial)()
ui_steps.disable_options_from_developer_options(serial = serial,
                                                developer_options =
                                                ["Verify apps over USB"],
                                                blocking = True)()

# Install any application
adb_steps.install_apk(serial = serial, apk_path = widevine_apk)()

# Create a new user
ui_steps.create_new_user(serial = serial,
                         set_up_user = True,
                         user_name = "New user")()

# Check if user has access to Owner's application
ui_steps.find_app_from_allapps(serial = serial,
                               presence = False,
                               view_to_find = {"textContains": "ExoPlayer"})()

### Remake initial state #####
ui_steps.switch_user(serial = serial, user_name = "Owner")()
ui_steps.remove_user(serial = serial, user_name = "New user")()
