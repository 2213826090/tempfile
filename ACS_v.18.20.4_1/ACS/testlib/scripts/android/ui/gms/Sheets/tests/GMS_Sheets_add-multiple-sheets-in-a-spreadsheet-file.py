#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Sheets / Added multiple sheets in a spreadsheet
#
# @author:      andrei.barjovanu@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.Gmail import gmail_steps
from testlib.scripts.gms import gms_utils

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})


uidevice = ui_device(serial = serial)

ui_steps.close_app_from_recent(serial = serial,\
                view_to_find = {"text": "Sheets"})()


if (gms_utils.get_google_account_number(serial = serial) == 0):
    ui_steps.add_google_account_for_L(serial = serial,version = "L",\
                 account = account,
                 password = password)()
if (gms_utils.get_google_account_number(serial = serial) > 1):
    ui_steps.remove_all_google_accounts(serial = serial)()
    ui_steps.add_google_account_for_L(serial = serial,version = "L",\
                 account = account,
                 password = password)()


ui_steps.open_app_from_allapps(serial = serial,\
                    view_to_find = {"text": "Sheets"})()

if uidevice(text = "Spreadsheets on the go").exists:
    ui_steps.click_button_if_exists(serial = serial,\
                view_to_find = {"text":"Skip"},\
                wait_time = 5000)()

if uidevice(descriptionContains = "View as List").exists:
    ui_steps.click_button_if_exists(serial = serial,\
                view_to_find = {"descriptionContains":"View as List"},\
                wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial,\
            view_to_find = {"descriptionContains":"Create new spreadsheet"},\
            view_to_check ={"text":"Untitled spreadsheet"},\
            wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial,\
            view_to_find = {"descriptionContains":"Add new sheet"},\
            view_to_check ={"text":"Untitled spreadsheet"},\
            wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial,\
            view_to_find = {"descriptionContains":"Add new sheet"},\
            view_to_check ={"text":"Untitled spreadsheet"},\
            wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial,\
            view_to_find = {"text":"Sheet1"},\
            view_to_check ={"uidevice(text = \"Sheet1\").selected": "true"},\
            wait_time = 5000)()


ui_steps.click_button_if_exists(serial = serial,\
    view_to_find = {"descriptionContains":"All Sheets Menu"},\
    view_to_check ={"resourceId":\
    "com.google.android.apps.docs.editors.sheets:id/dialog_box_content_frame"},\
    wait_time = 5000)()


ui_steps.click_button_if_exists(serial = serial,\
    view_to_find = {"descriptionContains":"Sheet2"},\
    view_to_check ={"uidevice(text = \"Sheet2\").selected": "true"},\
    wait_time = 5000)()

ui_steps.click_button_if_exists(serial = serial,\
    view_to_find = {"descriptionContains":"Navigate up"},\
    view_to_check ={"descriptionContains": "Open navigation drawer"},\
    wait_time = 5000)()

ui_steps.wait_for_view(serial = serial,\
            view_to_find = {"text":"Untitled spreadsheet"},\
            timeout = 40)
ui_steps.long_click(serial = serial,\
                view_to_find = {"text":"Untitled spreadsheet"},
                view_to_check = {"text": "Remove"})()

ui_steps.click_button_if_exists(serial = serial,\
    view_to_find = {"text":"Remove"},\
    view_to_check ={"text": "Do you really want to remove this file?"},\
    wait_time = 5000)()
ui_steps.click_button_if_exists(serial = serial,\
    view_to_find = {"text":"Remove"},\
    view_to_check ={"descriptionContains": "Open navigation drawer"},\
    wait_time = 5000)()

ui_steps.close_app_from_recent(serial = serial,\
        view_to_find={"text": "Sheets"})()
