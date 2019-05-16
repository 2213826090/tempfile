#!/usr/bin/env python
#######################################################################
#
# @filename:    AOSP_Android_N_Car_Application-Menu.Dialer_A_number_Back_Key.py.py
# @description: pressing the back key of dialer
# @author:      dpanday@intel.com
#
#######################################################################


import sys


# Used defined libraries
from testlib.base.base_utils import get_args
import time
from testlib.scripts.wireless.bluetooth.bt_step import Step as BtStep
from testlib.scripts.android.ui import ui_steps

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.press_dialer(serial=serial)()
ui_steps.click_button_common( view_to_find={'text':"Phone"}, second_view_to_find={"className":"android.widget.ImageButton"},serial=serial)()


time.sleep(5)
ui_steps.click_button_common( view_to_find={'text':"Phone"}, second_view_to_find={"className":"android.widget.ImageButton"},serial=serial)()

