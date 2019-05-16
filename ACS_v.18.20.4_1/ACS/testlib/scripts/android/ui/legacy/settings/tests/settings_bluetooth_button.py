#!/usr/bin/env python

##############################################################################
#
# @filename:    settings.bluetooth_button.py
#
# @description: Enable/disable bluetooth.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.wireless.bluetooth import bluetooth_steps as bt_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device

import sys
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

uidevice = ui_device()
ui_steps.press_home()()
ui_steps.open_settings(blocking = True)()
ui_steps.open_app_from_settings(print_error = "Error - Settings page was not displayed",
                                blocking = True,
                                view_to_find = {"text":"Bluetooth"},
                                view_to_check = {"text":"Bluetooth"})()

if uidevice(text = "Off").exists:
    ui_steps.click_button(print_error = "Failed to change BT state",
                          blocking = True,
                          view_to_find = {"text":"Off"},
                          view_to_check = {"text": "On"})()

uidevice(text = "Available devices").wait.exists(timeout = 20000)
bt_steps.bt_list_displayed(print_error = "Error - counting bt devices",
                           state = "ON")()

bt_steps.bt_list_displayed(print_error = "Error - counting bt devices",
                           state = "OFF")()

ui_steps.press_home()()

