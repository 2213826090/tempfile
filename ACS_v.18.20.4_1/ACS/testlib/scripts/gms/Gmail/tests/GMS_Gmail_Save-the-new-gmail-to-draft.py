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

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.close_app_from_recent(serial = serial,view_to_find=
            {"text": "Gmail"})()

if (gms_utils.get_google_account_number(serial = serial) == 0):
    ui_steps.add_google_account_for_L(serial = serial,version = "L",\
                 account = account,
                 password = password, open_from_settings = True,\
                 from_gmail = False)()

ui_steps.open_app_from_allapps(serial = serial,\
                        view_to_find = {"text": "Gmail"})()


gmail_steps.compose_mail(serial = serial,\
            account_to = account_to,
            subject = "Compose mail Test",
            body_mail = "this is a test mail",
            save_as_draft = True, send_mail = False)()

ui_steps.close_all_app_from_recent(serial = serial)()
