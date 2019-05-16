#!/usr/bin/env python

##############################################################################
#
# @filename:    youtube_steps.py
#
# @description: YouTube app test steps
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb import adb_steps

class open_youtube(ui_step):
    def do(self):
        # sending MEDIA_MOUNTED intent to have
        # video files available for Youtube app
        adb_steps.command(serial = self.serial, command = "am broadcast -a\
 android.intent.action.MEDIA_MOUNTED -d file:///storage/sdcard0/Movies",
                            timeout = 10)()
        # Close app before openning it to ensure first screen consistency
        ui_steps.close_app_from_recent(serial = self.serial,
                                       view_to_find={"text": "YouTube"})()
        time.sleep(1)
        ui_steps.open_app_from_allapps(serial = self.serial,
            view_to_find = {"text": "YouTube"},
            view_to_check = {"packageName":"com.google.android.youtube"})()
        # if opened for first time
        time.sleep(1)
        ui_steps.click_button_if_exists(serial = self.serial,
                                        view_to_find = {"text": "Skip"})()
        time.sleep(5)
        ui_steps.click_button_if_exists(serial = self.serial,
                                        view_to_find = {"text":"OK"})()

class search_video(ui_step):
    def __init__(self, search_for = "The Hobbit", items_count = 5,
                    comparator = "=", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.search_for = search_for
        self.items_count = items_count
        self.comparator = comparator
    def do(self):
        # click Search
        ui_steps.click_button(serial = self.serial,
                        view_to_find = {"description":"Search"},
                        view_to_check = {"textContains":"Search YouTube"})()

        time.sleep(2)
        # Input search string
        ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"textContains":"Search YouTube"},
                            value = self.search_for)()

        adb_steps.command(serial = self.serial, timeout = 10,
                            command = "input keyevent KEYCODE_ENTER")()

        time.sleep(5)
        # Check
        ui_steps.check_object_count(serial = self.serial,
            view_to_find = {"textContains":self.search_for},
            count = self.items_count, comparator = self.comparator)()


class sign_out(ui_step):
    def do(self):
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"description":"More options"},
                    view_to_check = {"text":"Sign out"})()
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"Sign out"})()
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"description":"More options"},
                    view_to_check = {"text":"Sign in"})()
        self.uidevice.press.back()

class sign_in(ui_step):
    def __init__(self, account = None,
                    password = None, force = True,
                    existing = False, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.force = force
        self.account = account
        self.password = password
        self.existing = existing
    def do(self):
        if self.force:
            ui_steps.remove_all_google_accounts(serial = self.serial)()
            open_youtube(serial = self.serial)()

        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"description":"More options"},
                    view_to_check = {"text":"Settings"})()
        if self.uidevice(text = "Sign out").exists:
            self.uidevice.press.back()
            if not self.force:
                return
            sign_out(serial = self.serial)()

        if self.existing:
            ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"Sign in"},
                    view_to_check = {"text":self.account})()
            ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":self.account},
                    view_to_check = {"text":"OK"})()
            ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"OK"},
                    view_to_check = {"packageName":"com.google.android.youtube"})()
        else:
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"text":"Sign in"},
                        view_to_check = {"description":"Add your account"})()
            ui_steps.add_google_account_for_L(serial = self.serial,
                        account = self.account, password = self.password,
                        open_from_settings = False)()

        time.sleep(2)
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"description":"More options"},
                    view_to_check = {"text":"Sign out"})()
        self.uidevice.press.back()
