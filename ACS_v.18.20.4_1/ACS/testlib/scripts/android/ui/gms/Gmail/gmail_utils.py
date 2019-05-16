#!/usr/bin/env python

##############################################################################
#
# @filename:    Gmail_steps.py
# @description: UI Gmail test steps
# @author:      andrei.barjovanu@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
import random


class compose_mail(ui_step):

    """description:
            send a test mail

        usage:
            gmail_steps.compose_mail(account_to = "<account>@gmail.com",
                                      subject = "Compose mail Test",
                                      body_mail = "this is a test mail")()

        tags:
            ui, gmail, mail, account, compose
    """
    def __init__(self,
                 account_to,
                 subject = "Compose mail Test",
                 body_mail = "this is a test mail",
                 **kwargs):
        self.account_to = account_to
        self.subject = subject
        self.body_mail = body_mail
        ui_step.__init__(self, **kwargs)

    def do (self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                view_to_find = {"text": "Gmail"})()
        ui_steps.click_button_if_exists(serial = self.serial,\
                view_to_find = {"resourceId":\
                "com.google.android.gm:id/compose_button"},wait_time = 2000)()

        ui_steps.edit_text(serial = self.serial,view_to_find = {"resourceId":
                    "com.google.android.gm:id/to"},
                    value = self.account_to)()

        ui_steps.click_button_if_exists(serial = self.serial,view_to_find = {"text":
            "Compose"},wait_time = 5000)()
        ui_steps.edit_text(serial = self.serial,view_to_find = {"resourceId":
                    "com.google.android.gm:id/subject"},
                    value = self.subject)()

        ui_steps.edit_text(serial = self.serial,view_to_find = {"resourceId":
                    "com.google.android.gm:id/body"},
                    value =  self.body_mail)()

        ui_steps.click_button_if_exists(serial = self.serial,\
            view_to_find = {"resourceId":"com.google.android.gm:id/send"})()

    def check_condition(self):
        ui_steps.click_button_if_exists(serial = self.serial,\
            view_to_find = {"descriptionContains":\
            "Open navigation drawer"},wait_time = 5000)()

        ui_steps.click_button_if_exists(serial = self.serial,\
            view_to_find = {"text":"Sent"},wait_time = 5000)()
        stat = self.uidevice(descriptionContains = self.body_mail).exists
        ui_steps.close_app_from_recent(serial = self.serial,view_to_find=\
            {"text": "Gmail"})()
        return stat

class gmail_open_mail(ui_step):

    """description:
            chech a email from inbox

        usage:
            gmail_steps.gmail_open_mail(account = "<account>@gmail.com",
                                      subject = "Compose mail Test",
                                      body_mail = "this is a test mail",
                                      delete = True)()

        tags:
            ui, gmail, mail, account, compose,delete mail
    """
    def __init__(self,
                 account,
                 subject = None,
                 body_mail = None,
                 delete = False,
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.account = account
        self.subject = subject
        self.body_mail = body_mail
        self.delete = delete

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                view_to_find = {"text": "Gmail"})()
        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : "Open navigation drawer"},wait_time = 5000)()

        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"text" : "Primary"},wait_time = 5000)()
        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : self.subject},wait_time = 5000)()

    def check_condition(self):
        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : self.subject},wait_time = 5000)()
        stat = self.uidevice(textContains = self.subject).exists
        if self.delete:
            ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : "Delete"},wait_time = 5000)()
        return stat
