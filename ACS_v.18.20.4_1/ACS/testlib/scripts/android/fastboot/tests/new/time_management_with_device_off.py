#!/usr/bin/env python

import os
import sys
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

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
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/prev"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"text": "1"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/button1",
                                          "text": "OK"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Set time"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"className": "android.widget.RadialTimePickerView$RadialPickerTouchHelper",
                                          "instance": "6"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"className": "android.widget.RadialTimePickerView$RadialPickerTouchHelper",
                                          "instance": "6"})()
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId": "android:id/button1",
                                          "text": "OK"})()
    ui_steps.click_switch(serial = serial, right_of = True, state = "ON",
                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Use 24-hour format"})()

    data_before_restart = None
    data_after_restart = None
    command = "adb shell date"

    r = os.popen(command)
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        line[3] = line[3].strip("\r\n")
        line[3] = line[3].split(":")
        data_before_restart = line[3]

    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               force_reboot = True,
                               delay_power_on = 120)()

    r = os.popen(command)
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        line[3] = line[3].strip("\r\n")
        line[3] = line[3].split(":")
        data_after_restart = line[3]

    if (int(data_after_restart[1]) - int(data_before_restart[1])) <= 2:
        raise Exception("The test result did not achieve the desired results")

except:
    raise

finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port)()
# Test end #