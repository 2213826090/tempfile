#!/usr/bin/env python

##############################################################################
#
# @filename:    drive_steps.py
#
# @description: Google drive app test steps
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_step import FailedError
from testlib.scripts.gms import gms_utils
from testlib.scripts.gms.drive import drive_utils

class open_drive(ui_step):
    def __init__(self, account = None,
                    password = None, force = True, **kwargs):
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
            time.sleep(2)

        # Close app before openning it to ensure first screen consistency
        ui_steps.close_app_from_recent(serial = self.serial,
                                       view_to_find={"text": "Drive"})()
        self.uidevice.wait.idle()
        ui_steps.open_app_from_allapps(serial = self.serial,
            view_to_find = {"text": "Drive"},
            view_to_check = {"packageName":"com.google.android.apps.docs"})()
        # if opened for first time
        self.uidevice.wait.idle()
        ui_steps.click_button_if_exists(serial = self.serial,
                                        view_to_find = {"text": "Skip"})()

class upload_file(ui_step):
    def __init__(self, file_name = None, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.file_name = file_name

    def do(self):
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"description":"Open navigation drawer"},
            view_to_check = {"text":"Uploads"})()

        ui_steps.click_button(serial = self.serial, view_to_find = {"text":"Uploads"},
            view_to_check = {"description":"Create"})()

        ui_steps.click_button(serial = self.serial,
            view_to_find = {"description":"Create"},
            view_to_check = {"text":"Upload"})()

        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text":"Upload"},
            view_to_check = {"packageName":"com.android.documentsui"})()

        ui_steps.click_button_if_exists(serial = self.serial,
            view_to_find = {"description":"Show roots"})()

        ui_steps.click_button(serial = self.serial, view_to_find = {"text":"Videos"},
            view_to_check = {"text":"Movies"})()

        ui_steps.show_as_list(serial = self.serial)()

        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text":"Movies"})()

        self.uidevice(resourceIdMatches = ".*toolbar").\
            child(text="Movies").wait.exists(timeout = 20000)

        ui_steps.show_as_list(serial = self.serial)()

        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text":self.file_name},
            view_to_check = {"packageName":"com.google.android.apps.docs"})()

        time.sleep(60)

        # Go back to my drive and check the file is there
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"description":"Open navigation drawer"},
            view_to_check = {"text":"My Drive"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text":"My Drive"},
            view_to_check = {"description":"Open navigation drawer"})()
        drive_utils.search_by(name = self.file_name[:-4],
                            serial = self.serial)
        # time.sleep(2)
        ui_steps.wait_for_view(serial = self.serial,
            view_to_find = {"descriptionContains":self.file_name[:-4]})()
        drive_utils.exit_search(serial = self.serial)

class remove_file(ui_step):
    def __init__(self, file_name = None, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.file_name = file_name

    def do(self):
        drive_utils.search_by(name = self.file_name[:-4],
                            serial = self.serial)
        if self.uidevice(descriptionContains = self.file_name[:-4]).exists:
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"description":"More options"},
                view_to_check = {"text":"Select all"})()

            ui_steps.click_button(serial = self.serial,
                view_to_find = {"text":"Select all"},
                view_to_check = {"description":"More functions for selected items"})()
            if self.uidevice(description = "Remove selected items").exists:
                ui_steps.click_button(serial = self.serial,
                view_to_find = {"description":"Remove selected items"},
                view_to_check = {"descriptionContains":self.file_name[:-4]},
                    view_presence = False)()
            else:
                ui_steps.click_button(serial = self.serial,
                    view_to_find = {"description":"More functions for selected items"},
                    view_to_check = {"text":"Remove"})()
                ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"Remove"},
                    view_to_check = {"descriptionContains":self.file_name[:-4]},
                    view_presence = False)()
        drive_utils.exit_search(serial = self.serial)

class check_files_type(ui_step):
    def __init__(self, file_type = "PDF", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.file_type = file_type
        self.files_count = 0
        self.correct_type_files_count = 0

    def do(self):
        self.files_count = self.uidevice(description = "Show item properties").count
        for i in range(self.files_count):
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"description":"Show item properties", "instance":i},
                view_to_check = {"text":"Type"})()
            if self.uidevice(text = "Type").right(textContains = self.file_type).exists:
                self.correct_type_files_count += 1

            ui_steps.click_button(serial = self.serial,
                view_to_find = {"description":"Close item details"},
                view_to_check = {"text":"Filter: " + self.file_type})()
            time.sleep(1)

    def check_condition(self):
        return self.correct_type_files_count == self.files_count
