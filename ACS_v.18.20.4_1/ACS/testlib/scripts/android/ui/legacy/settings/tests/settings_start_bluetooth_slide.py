#!/usr/bin/env python

##############################################################################
#
# @filename:    settings_start_bluetooth_slide.py
#
# @description: Enable bluetooth from settings screen.
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

ui_steps.open_settings_app(view_to_find = {"text": "Bluetooth"},
                           view_to_check = {"text": "Bluetooth"}
                          )()
if ui_device(serial = serial)(text = "Off").exists:
    ui_steps.click_button(print_error = "Failed to change BT state",
                          blocking = True,
                          view_to_find = {"text":"Off"},
                          view_to_check = {"text": "On"})()


ui_steps.press_home()()

