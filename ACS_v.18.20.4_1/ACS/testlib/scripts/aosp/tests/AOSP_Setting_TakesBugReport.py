#!/usr/bin/env python

#######################################################################
#
# @filename:    AOSP_Settings_Developer_options_Enable_Bug report.py
# @description: Take bug report
# @author:      dpanday@intel.com
#######################################################################

# Build in libraries

import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
import time

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val


# Setup
ui_steps.press_home(serial=serial)()

# Run
ui_steps.enable_developer_options(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Developer options"},view_to_check={"text":"OEM unlocking"},serial=serial)()


# for full report

ui_steps.click_button_common(view_to_find={"text":"Take bug report"},view_to_check={"text":"Interactive report"},serial=serial)()

ui_steps.click_button_common(view_to_find={"text":"Full report"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"REPORT"},serial=serial)()


#for interactive report
ui_steps.click_button_common(view_to_find={"text":"Take bug report"},view_to_check={"text":"Interactive report"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Interactive report"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"REPORT"},serial=serial)()


#teardown

#moving  to home to verify if the report has been submitted

ui_steps.press_home(serial=serial)()

time.sleep(60)
ui_steps.wait_for_view_common(view_to_find={"text":"Tap to share your bug report"},serial=serial)()

#