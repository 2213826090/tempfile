#!/usr/bin/env python

##############################################################################
#
# @filename:    make_available_offline.py
#
# @description: Check filter by option for gdrive
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.drive import drive_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.drive import drive_utils
from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

offline_file_name = offline_file_name.replace("_", " ")

ui_steps.press_home(serial = serial)()

drive_steps.open_drive(serial = serial, account = account,
                        password = password, force = False)()
drive_utils.search_by(name = offline_file_name, serial = serial)
ui_steps.click_button(serial = serial,
    view_to_find = {"descriptionContains":offline_file_name},
    child_view_to_find = {"description":"Show item properties"},
    view_to_check = {"text":"Keep on device"})()

ui_steps.click_switch(serial = serial,
    view_to_find = {"text":"Keep on device"}, right_of = True)()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close item details"},
    view_to_check = {"description":"Close search"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Close search"},
    view_to_check = {"text":"My Drive"})()

time.sleep(60)

# Made available offline
# Turn Wifi off
# Enclosed in try except to make sure Wifi is turned back on if failure
try:
    wifi_steps.set_airplane_mode(serial = serial, state = "ON")()

    time.sleep(5)

    drive_steps.open_drive(serial = serial, account = account,
                        password = password, force = False)()
    drive_utils.search_by(name = offline_file_name, serial = serial)
    # try to access offline file
    ui_steps.click_button(serial = serial,
        view_to_find = {"descriptionContains":offline_file_name},
        view_to_check = {"textContains":offline_file_name,
                            "packageName":"com.google.android.apps.docs"})()

    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"Navigate up"},
        view_to_check = {"description":"Close search"})()

    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"Close search"},
        view_to_check = {"text":"My Drive"})()

    # try to access online file
    drive_utils.search_by(name = online_file_name, serial = serial)
    ui_steps.click_button(serial = serial,
        view_to_find = {"descriptionContains":online_file_name},
        view_to_check = {"description":"Navigate up"},
        view_presence = False)()
    ui_step(serial = serial).uidevice.press.back()

    ui_steps.click_button(serial = serial,
        view_to_find = {"description":"Close search"},
        view_to_check = {"text":"My Drive"})()

except Exception, e:
    wifi_steps.set_airplane_mode(serial = serial, state = "OFF")()
    raise e

# Turn wifi back on
wifi_steps.set_airplane_mode(serial = serial, state = "OFF")()

drive_steps.open_drive(serial = serial, account = account,
    password = password, force = False)()
time.sleep(10)

drive_utils.search_by(name = offline_file_name, serial = serial)
ui_steps.long_click(serial = serial,
    view_to_find = {"descriptionContains":offline_file_name},
    view_to_check = {"description":"Remove selected items on this device"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"Remove selected items on this device"},
    view_to_check = {"description":"Close search"})()

time.sleep(10)

ui_steps.long_click(serial = serial,
    view_to_find = {"descriptionContains":offline_file_name},
    view_to_check = {"description":"Keep selected items on this device"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"description":"More functions for selected items"},
    view_to_check = {"text":"Clear selection"})()

ui_steps.click_button(serial = serial,
    view_to_find = {"text":"Clear selection"},
    view_to_check = {"text":"Remove"},
    view_presence = False)()

ui_step(serial = serial).uidevice.press.back()

# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
