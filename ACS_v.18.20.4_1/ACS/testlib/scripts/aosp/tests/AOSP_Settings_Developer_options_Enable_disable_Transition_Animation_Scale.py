#!/usr/bin/env python

#######################################################################
#
# @filename:    AOSP_Settings_Developer_options_Enable_disable_Transition_Animation_Scale.py
# @description: Turn ON and OFF Transition_Animation_Scale
# @author:      dpanday@intel.com
#######################################################################

# Build in libraries

import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps

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

#option 1 animation off

ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation off"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation off"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation off"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]

# option 2 Animation scale .5x


ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale .5x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale .5x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale .5x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]

#option 3 Animation scale 1x

ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale 1x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale 1x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale 1x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]

#

#option 4 Animation scale 1.5x

ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale 1.5x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale 1.5x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale 1.5x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]


#option 4 Animation scale 2x

ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale 1.5x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale 2x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale 2x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]

#option 4 Animation scale 5x


ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale 1.5x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale 5x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale 5x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]


#option 4 Animation scale 10x


ui_steps.click_button_common(view_to_find={"text":"Transition animation scale"},view_to_check={"text":"Animation scale 1.5x"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Animation scale 10x"},serial=serial)()
ui_steps.wait_for_view_common(view_to_find={"text":"Animation scale 10x"},second_view_to_find={"className":"android.widget.TextView"},position="down",retrieve_info=True,serial=serial)()["text"]


