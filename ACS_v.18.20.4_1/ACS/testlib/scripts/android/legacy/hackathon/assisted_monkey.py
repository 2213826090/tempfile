#!/usr/bin/env python
#######################################################################
#
# @filename:    view_tree.py
# @description: Hackaton script
# @author:      ion-horia.petrisor@intel.com
#
# python assisted_monkey.py --serial 10.237.104.142:5555
#                           --script-args app-to-test=Shazam
#                                         wait-time=2000
#
#######################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.ui.uiandroid import UIDevice as ui_device
import sys
import time

def random_text(text):
    uidevice = ui_device()
    for letter in text:
        uidevice.press(ord(letter))

def random_clicks(rows, columns, width, height):
    row_delta = height/rows
    column_delta = width/columns
    x_center = column_delta/2
    y_center = row_delta/2
    for i in range(rows):
        for j in range(columns):
            ui_steps.click_xy(x = x_center, y = y_center)()
            time.sleep(0.5)
            y_center += row_delta
        x_center += column_delta
        y_center = row_delta/2


def generate_random_pair(min_root1, max_root1, min_root2, max_root2):
    from random import randint
    return randint(min_root1 - 1, max_root1 - 1), randint(min_root2 - 1, max_root2 - 1)

def random_swipes(no, min_size, max_size, width, height):
    for i in range(no):
        x1, y1 = generate_random_pair(1, width, 1, height)
        x2, y2 = generate_random_pair(min_size, max_size, min_size, max_size)
        x2 += x1
        y2 += y2
        ui_steps.swipe(sx = x1, sy = y1, ex = x2, ey = y2, steps = 5)()


globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

app_to_test = args["app-to-test"]
wait_time = int(args["wait-time"])/1000

ui_steps.press_home()()

################## Openning app to test
ui_steps.open_app_from_allapps(view_to_find = {"text": app_to_test})()
time.sleep(wait_time)

uidevice = ui_device()
width = uidevice.info["displayWidth"]
height = uidevice.info["displayHeight"]

random_clicks(2, 3, width, height)

random_swipes(100, 50, 350, width, height)


#random_text("bogus")

#time.sleep(10)

ui_steps.press_home()()

