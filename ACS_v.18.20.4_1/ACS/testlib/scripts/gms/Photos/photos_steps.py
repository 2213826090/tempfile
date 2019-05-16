#!/usr/bin/env python

##############################################################################
#
# @filename:    photos_steps.py
#
# @description: Google Photos app test steps
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import time

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.gms import gms_utils


class add_account(ui_step):
    def __init__(self, account = "auto3test@gmail.com",
                    password = "testing12345", force = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.account = account
        self.password = password
        self.force = force
    def do(self):
        if self.force:
            # sending MEDIA_MOUNTED intent to have
            # video files available
            adb_steps.command(serial = self.serial, command = "am broadcast -a\
                android.intent.action.MEDIA_MOUNTED -d file:///storage/sdcard0/Movies",
                                timeout = 10)()
            # Make sure user is signed in before opening app
            WHERE = 'name = "{0}"'.format(self.account)
            account_exists = ui_utils.google_account_exists(serial = self.serial, where = WHERE)
            total_account_no = gms_utils.get_google_account_number(serial = self.serial)

            if account_exists:
                account_synced = ui_steps.sync_google_account(serial = self.serial,
                                        account = self.account,
                                        password = self.password)()

            if (total_account_no >= 2) or (not account_exists) or (account_exists and not account_synced):
                ui_steps.remove_all_google_accounts(serial = self.serial)()
                ui_steps.add_google_account_for_L(serial = self.serial,
                                            account = self.account,
                                            password = self.password,
                                            prefer_sync = True)()
            #~ ui_steps.remove_all_google_accounts(serial = self.serial)()
            #~ ui_steps.add_google_account_for_L(serial = self.serial,
                       #~ account = self.account, password = self.password)()
            time.sleep(5)
            ui_steps.press_home(serial = self.serial)()
