#!/usr/bin/env python
# -*- coding:utf-8 -*-
######################################################################
#
# @filename:    scan_wifi_state_iter
# @description: Tests that scan AP name,Switch wifi state iterations.
#
# @run example:
#
#            python scan_wifi_state_iter -s 0BA8F2A0
#                                          --script-args
#                                               ap_name = ddwrt
#                                               iterations = 10
#
# @author:      corneliu.stoicescu@intel.com
#
#######################################################################
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.external.uiautomator import Device
import sys
from testlib.scripts.android.adb import adb_utils
import time

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
iterations = int(args.get("iterations"))
ap_name = args.get("ap_name")
ap_lists = [ap_name, "WIFI_ADB"]
uidevice = Device(serial=serial)
product_name = adb_utils.get_product_name(serial=serial)

# turn display on, if turned off
ui_steps.wake_up_device(serial=serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial=serial, pin=wifi_defaults.wifi['pin'])()

# go to settings app
if product_name != "bxtp_abl_car":
    ui_steps.press_all_apps(serial=serial)()
    ui_steps.click_button_with_scroll(serial=serial, view_to_find={"text": "Settings"}
                                ,view_to_check={"textContains": "Wi‑Fi"})()
elif if self.device_info.dessert == "O":
    wifi_steps.open_wifi_settings(serial=serial)()
else:
    ui_steps.press_home(serial=serial)()
    ui_steps.click_button_with_scroll(serial=serial, view_to_find={"resourceId": "com.android.car.overview:id/gear_button"}
                                , view_to_check={"textContains": "Wi‑Fi"})()
ui_steps.click_button_with_scroll(serial=serial, view_to_find={"text": "Wi‑Fi"}
                                ,view_to_check={"textContains": "Wi‑Fi"})()

for iteration in range(iterations):
    # click switch On button,Switch wifi Off
    if uidevice(text="On").exists:
        ui_steps.click_button_with_scroll(serial=serial, view_to_find={"text": "On"}
                        , view_to_check={"resourceId": "android:id/empty"})()

    # click switch Off button,Switch wifi ON
    if uidevice(text="Off").exists:
        ui_steps.click_button_with_scroll(serial=serial, view_to_find={"text": "Off"}
                            , view_to_check={"text": "On"})()

    time.sleep(0.5)
for ap_list in ap_lists:
    wifi_steps.scan_and_check_ap(serial=serial, ap=ap_list, option="refresh", open_wifi="Manual")()
