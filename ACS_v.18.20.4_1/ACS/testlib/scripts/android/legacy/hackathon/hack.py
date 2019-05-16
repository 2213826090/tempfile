#!/usr/bin/env python
#######################################################################
#
# @filename:    hack.py
# @description: Hackaton script
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

from testlib.base.base_utils import get_args
import sys
import time

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

"""
################### HowTo: take_screenshot

adb_steps.take_screenshot_given_path(screenshot_file = "gigi.png",
                                     host_path = "/home/sys_spjenkins/")()

"""

"""
################### HowTo: take_screenrecord

adb_steps.take_screenrecord_given_path(screenrecord_file = "gigi.mp4",
                                       record_time = 30,
                                       host_path = "/home/sys_spjenkins/")()

"""

"""
################### HowTo: check app in recent apps

ui_steps.press_home()()
ui_steps.open_app_from_allapps(view_to_find = {"text": "Calculator"},
                               view_to_check = {"text": "sin"})()

ui_steps.press_home()()
ui_steps.app_in_recent_apps(app_name = "Calculator")()
ui_steps.press_home()()

"""

"""
################### HowTo: sleep/wakeup device

adb_steps.take_screenshot_given_path(screenshot_file = "start.png",
                                     host_path = "/home/sys_spjenkins/")()
ui_steps.put_device_into_sleep_mode()()
time.sleep(3)
adb_steps.take_screenshot_given_path(screenshot_file = "sleep.png",
                                     host_path = "/home/sys_spjenkins/")()
ui_steps.wake_up_device()()
time.sleep(3)
adb_steps.take_screenshot_given_path(screenshot_file = "wake.png",
                                     host_path = "/home/sys_spjenkins/")()

"""

"""
################### HowTo: find an app in app list

ui_steps.press_home()()
ui_steps.find_app_from_allapps(view_to_find = {"text": "Apiests"})()
ui_steps.press_home()()

"""

"""
################### HowTo: change orientation

ui_steps.press_home()()
adb_steps.take_screenshot_given_path(screenshot_file = "intitail.png",
                                     host_path = "/home/sys_spjenkins/")()
ui_steps.set_orientation(orientation = "portrait")()
time.sleep(2)
adb_steps.take_screenshot_given_path(screenshot_file = "portrait.png",
                                     host_path = "/home/sys_spjenkins/")()
ui_steps.set_orientation(orientation = "reverse-portrait")()
time.sleep(2)
adb_steps.take_screenshot_given_path(screenshot_file = "reverse-portrait.png",
                                     host_path = "/home/sys_spjenkins/")()
ui_steps.set_orientation()()
time.sleep(2)
adb_steps.take_screenshot_given_path(screenshot_file = "landscape.png",
                                     host_path = "/home/sys_spjenkins/")()
"""

