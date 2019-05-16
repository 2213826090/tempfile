#!/usr/bin/env python

######################################################################
#
# @filename:    factory_reset.py
# @description: Test factory reset.
#
# @run example:
#
#            python factory_reset.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               app=com.intel.test.apitests
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.flash import flash_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.utils.statics.android import statics

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
app = args["app"]
dut_platform = statics.Device(serial=serial)

##### test start #####
relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port)()

ui_steps.wake_up_device(serial = serial)()
ui_steps.unlock_device(serial = serial)()

# install user apk - prereq
# file in /data
adb_steps.root_connect_device(serial = serial)()
adb_steps.create_folder(serial=serial,
                        path = "/data",
                        folder = "test_factory_reset")()

# data in /data/data/app - start an app (Calculator)
ui_steps.open_app_from_allapps(serial = serial,
                      view_to_find = {"text": "Calculator"})()

adb_steps.check_folders_exist(serial = serial,
                              folder_list = ["/data/data/com.google.android.calculator/cache"])()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial)()

# reset to factory defaults
flash_steps.factory_reset(serial = serial,
                          reset_button_text = dut_platform.reset_button_text)()

# wait for adb to become online
local_steps.wait_for_adb(serial = serial,
                         timeout = 120)()

# wait for UI
adb_steps.wait_for_ui(serial = serial,
                                    timeout = 1200)()

# setup stay awake
adb_steps.command(serial = serial,
                command = "svc power stayon true")()

ui_steps.press_home(serial = serial)()

#check data was erased
adb_steps.root_connect_device(serial = serial)()

# package removed
adb_steps.check_package_installed(serial = serial,
                                  package = app,
                                  presence = False)()

# /data cleaned
adb_steps.check_folders_exist(serial = serial,
                              folder_list = ["/data/test_factory_reset"],
                              presence = False)()

# /data/data/app cleaned
adb_steps.check_folders_exist(serial = serial,
                              folder_list = ["/data/data/com.google.android.calculator/cache"],
                              presence = False)()
#### test end #####
