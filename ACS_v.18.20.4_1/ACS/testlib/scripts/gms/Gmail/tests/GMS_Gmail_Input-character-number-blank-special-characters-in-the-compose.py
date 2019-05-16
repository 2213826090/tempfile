#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Gmail / Sending email using Gmail
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
from testlib.utils.ui import uiandroid

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.root_connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

uidevice = uiandroid.UIDevice(serial = serial)

ui_steps.close_app_from_recent(serial = serial,\
                view_to_find = {"text": "Gmail"})()

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

ui_steps.click_button(serial = serial,view_to_find = {"text":"Google"},
                    view_to_check = {"text": "Add account"})()


ui_steps.click_button(serial = serial,view_to_find = {"text":"OK"},\
                    view_to_check = {"descriptionContains": "Sign in to"},\
                    timeout = 60)()


ui_steps.add_google_account_for_L(serial = serial,version = "L",\
                 account = account,
                 password = password, open_from_settings = False,\
                 from_gmail = True)()

gmail_steps.compose_mail(serial = serial,\
            account_to = account_to,
            subject = "Compose mail Test",
            body_mail = "*_ bo$$ DE 1234!")()

gmail_steps.add_the_second_account_fromGmail(account = account_to, \
                                            password = password_to)()

gmail_steps.switch_gmail_accounts(serial = serial,\
                account_to_switch=account_to, \
                password = password_to)()

gmail_steps.gmail_open_mail(serial = serial, account  = account_to, \
                subject="Compose mail Test",\
                body_mail="*_ bo$$ DE 1234!",delete=True)()

ui_steps.close_all_app_from_recent()()


