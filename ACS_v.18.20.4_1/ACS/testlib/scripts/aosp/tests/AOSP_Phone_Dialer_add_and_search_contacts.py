#!/usr/bin/env python
#######################################################################
#
# @filename:    AOSP_Phone_Dialer_add_and_search_contact.py_
# @description: adding contact to contact list and searching them
# @author:      dpanday@intel.com
# comment : make sure bt is enable and paired with other devices
#######################################################################


import sys

# Used defined libraries
from testlib.base.base_utils import get_args
import time
from testlib.scripts.wireless.bluetooth.bt_step import Step as BtStep
from testlib.scripts.android.ui import ui_steps

# Setup
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val


ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.click_xy(x=727, y=404, serial=serial)()
# ui_steps.click_button_common(view_to_find={"text":"Contacts"}, second_view_to_find={
# "ClassName":"android.widget.TextView"},
#view_to_check = {"Text": "Contacts"}, serial = serial())


# main (add contact to the list )

ui_steps.click_button_if_exists(view_to_find={
    "resourceId": "com.android.contacts:id/floating_action_button"},
    second_view_to_find={"className": "android.widget.ImageButton"},
view_to_check = {"text": "First name"},serial=serial)()

ui_steps.edit_text(view_to_find={"text": "First name"}, value="abcdef", serial=serial)()
ui_steps.edit_text(view_to_find={"text": "Last name"}, value="ghijkl", serial=serial)()
ui_steps.edit_text(view_to_find={"text": "Phone"}, value="1 234-567-89", serial=serial)()

ui_steps.click_button_if_exists(view_to_find={"text":"SAVE"},view_to_check={"text":"1 234-567-89"},
                                serial= serial)()




# searching  the contact in the contact list

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.press_dialer(serial=serial)()

ui_steps.click_button_if_exists(view_to_find={
    "resourceId": "com.android.car.dialer:id/search"},
    second_view_to_find={"className": "android.widget.TextView"},
view_to_check = {"text": "Search contacts"},serial=serial)()


ui_steps.edit_text(view_to_find={"text": "Search contacts"}, value="abcdef ghijkl", serial=serial)()


ui_steps.click_button_common(view_to_find={
    "resourceId": "com.android.car.dialer:id/contact_name"},view_to_check = {"text": "1 234-567-89"},serial=serial)()

#teardown

ui_steps.press_home(serial=serial)()


