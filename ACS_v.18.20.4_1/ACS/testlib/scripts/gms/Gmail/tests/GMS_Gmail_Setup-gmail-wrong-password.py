#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Gmail / Sign in and sync
#
# @author:      andrei.barjovanu@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.Gmail import gmail_steps
from testlib.scripts.gms import gms_utils
from testlib.scripts.android.ui import ui_utils

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils
from testlib.utils.ui import uiandroid

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})


uidevice = uiandroid.UIDevice(serial = serial)

ui_steps.close_all_app_from_recent()()


if (gms_utils.get_google_account_number(serial = serial) > 0):
    ui_steps.remove_all_google_accounts(serial = serial)()

ui_steps.open_app_from_allapps(serial = serial,\
                        view_to_find = {"text": "Gmail"})()

if uidevice(text = "Got it").wait.exists(timeout = 5000):
    ui_steps.click_button(serial = serial,\
                    view_to_find = {"text":"Got it"},
                    view_to_check = {"text": "Add an email address"})()

if uidevice(text = "Skip").wait.exists(timeout = 5000):
    ui_steps.click_button(serial = serial,\
                    view_to_find = {"text":"Skip"},
                    view_to_check = {"text": "Add an email address"})()

ui_steps.click_button(serial = serial,\
                    view_to_find = {"text":"Add an email address"},
                    view_to_check = {"text": "Google"})()

while not uidevice(resourceId="com.google.android.gm:id/google_option").\
        child(resourceId="com.google.android.gm:id/radio_button").checked:
    ui_steps.click_button(serial = serial,\
                        view_to_find = {"text":"Google"})()

ui_steps.click_button(serial = serial,
                    view_to_find = {"text":"Next"})()

uidevice(text="Checking info").wait.gone(timeout = 30000)

ui_steps.edit_text(serial = serial,\
                  view_to_find = {"className":"android.widget.EditText"},\
                  value = account, is_password=True)()


uidevice(text = account).wait.exists(timeout = 7000)
#press enter keycode
uidevice.press(66)

ui_steps.click_button(serial = serial,\
    view_to_find = {"descriptionContains":"Password"},\
    view_to_check = {"descriptionContains":"Forgot password?"})()

ui_steps.edit_text(serial = serial, view_to_find = {"className":\
    "android.widget.EditText"},value = "wrongpass", is_password = True)()

#press enter keycode
uidevice.press(66)

uidevice(descriptionContains  =\
    "The email and password you entered don't match").wait.exists(timeout = 7000)
ui_steps.check_object_count(view_to_find = \
    {"descriptionContains":"The email and password you entered don't match"})()

ui_steps.close_all_app_from_recent()()
