#!/usr/bin/env python
#######################################################################
#
# @filename:    app_and_back.py
# @description: Hackaton script 
# @author:      ion-horia.petrisor@intel.com
#
# python app_and_back.py --serial 10.237.104.142:5555 
#                        --script-args app-to-test=Shazam
#                                      wait-time=2000
#                                      app-to-open=Calculator
#                                      host-path=/home/oane/screens
#
#######################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.file import file_steps
from testlib.base.base_utils import get_args
import sys
import time

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

host_path = args["host-path"]
app_to_test = args["app-to-test"]
wait_time = int(args["wait-time"])/1000
app_to_open = args["app-to-open"]

ui_steps.press_home()()

################## Openning app to test
ui_steps.open_app_from_allapps(view_to_find = {"text": app_to_test})()
time.sleep(wait_time)

################## Taking initial screenshot
adb_steps.take_screenshot_given_path(screenshot_file = "app_initial.png",
                                     host_path = host_path)()

################## Openning second app
ui_steps.press_home()()
ui_steps.open_app_from_allapps(view_to_find = {"textContains": app_to_open})()

################## Return to the app to test
ui_steps.press_home()()
ui_steps.open_app_from_recent_apps(app_name = app_to_test)()
time.sleep(wait_time)

################## Taking final screenshot
adb_steps.take_screenshot_given_path(screenshot_file = "app_final.png",
                                     host_path = host_path)()

################## Compare the two screenshots
file_steps.compare_images(first_file = host_path + "/app_initial.png",
                          second_file = host_path + "/app_final.png",
                          tolerance = 10)()

ui_steps.press_home()()

