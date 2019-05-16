#!/usr/bin/env python

# #############################################################################
#
# @filename:    fastboot_unlock_and_Widevine_DRM_Level1.py
#
# @description: Checks if the security key that was used to provision the
# device was not erased after the bootloader was unlocked) and the device was
# erased)
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
ap_name =  args["net_ap_ssid"]
ap_password = args["net_ap_password"]
widevine_apk = args["widevine_apk"]
wvkeyboxtool = args["wvkeyboxtool"]
keybox = args["keybox"]

dut_dessert = adb_utils.get_android_version(serial = serial)

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234")()

time.sleep(5)
# reboot to fastboot
adb_steps.reboot(serial = serial,
                command = "fastboot",
                ip_enabled = False,
                pin = "1234",
                blocking = True)()

fastboot_steps.change_state(serial = serial,
                            unlock_bootloader = "yes",
                            dessert = dut_dessert)()

prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_wifi = True,
                         ap_name = "sp_gpt",
                         ap_password = "Starpeakqwe123!")()

ui_steps.press_home(serial = serial)()

adb_steps.provision_sofia(serial = serial,
                wvkeyboxtool = wvkeyboxtool,
                keybox = keybox)()

ui_steps.disable_options_from_developer_options(serial = serial,
                                                developer_options =
                                                ["Verify apps over USB"])()

# Install ExoPlayer
adb_steps.install_apk(serial = serial,
                      apk_path = widevine_apk,
                      blocking = True)()

# Check if video plays
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find = {"textContains": "ExoPlayer"},
                               blocking = True)()

ui_steps.open_app_from_allapps(serial = serial,
                               view_to_find = {"textContains": "ExoPlayer"},
                               blocking = True)()

# Scroll down to Widevine video
ui_steps.scroll_up_to_view(serial = serial,
                           ey = 200,
                           view_to_check = {"textContains": "WV: HDCP not specified"})()
ui_steps.click_button(serial = serial,
                      view_to_find = {"textContains": "WV: HDCP not specified"})()
ui_steps.wait_for_view(serial = serial,
                       view_to_find = {"textContains": "playbackState=ready"},
                       timeout = 15000)()
ui_steps.press_home(serial = serial)()
