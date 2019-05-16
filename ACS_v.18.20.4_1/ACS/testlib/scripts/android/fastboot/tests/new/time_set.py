#!/usr/bin/env python

import os
import sys
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.utils.ui.uiandroid import UIDevice as ui_device

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

uidevice = ui_device(serial = serial)

# Test start #
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port)()

    ui_steps.open_settings(serial = serial)()
    ui_steps.click_button_with_scroll(serial = serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Date & time"})()
    ui_steps.click_switch(serial = serial, right_of = True, state = "OFF",
                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Automatic date & time"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Set date"})()

    tmp_result = False

    tmp_tries = 1000
    while tmp_tries > 0:
        if uidevice(resourceId = "android:id/prev").wait.exists(timeout = 5000):
            ui_steps.click_button(serial = serial, view_to_find = {"resourceId": "android:id/prev"})()
        else:
            tmp_result = True
            break
        tmp_tries -= 1

    if not tmp_result:
        raise Exception("The test result did not achieve the desired results")

    tmp_result = False

    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/button2",
                                          "text": "CANCEL"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Set date"})()

    tmp_tries = 1000
    while tmp_tries > 0:
        if uidevice(resourceId = "android:id/next").wait.exists(timeout = 5000):
            ui_steps.click_button(serial = serial, view_to_find = {"resourceId": "android:id/next"})()
        else:
            tmp_result = True
            break
        tmp_tries -= 1

    if not tmp_result:
        raise Exception("The test result did not achieve the desired results")

    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/button2",
                                          "text": "CANCEL"})()
    uidevice.press.home()

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port)()
# Test end #