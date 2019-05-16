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
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms import gms_utils
import time
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
                 send_mail = True,
                 save_as_draft = False,
                 **kwargs):
        self.account_to = account_to
        self.subject = subject
        self.body_mail = body_mail
        self.send_mail = send_mail
        self.save_as_draft = save_as_draft
        ui_step.__init__(self, **kwargs)

    def do (self):
        ui_steps.open_app_from_allapps(serial = self.serial,\
                        view_to_find = {"text": "Gmail"})()
        ui_steps.click_button_if_exists(serial = self.serial,\
                    view_to_find =\
                    {"resourceId":"com.google.android.gm:id/compose_button"},\
                    wait_time = 2000)()
        #sometimes you have to wait for sync. A Wait for sync message appears:
        if self.uidevice(text="Waiting for sync").wait.exists(timeout = 10000):
            #then wait for it to disappear
            self.uidevice(text="Waiting for sync").wait.gone(timeout = 90000)

        ui_steps.edit_text(serial = self.serial,\
            view_to_find = {"resourceId":"com.google.android.gm:id/to"},
            value = self.account_to)()

        ui_steps.click_button_if_exists(serial = self.serial,\
            view_to_find = {"text":"Compose"},wait_time = 5000)()
        ui_steps.edit_text(serial = self.serial,view_to_find = {"resourceId":\
                    "com.google.android.gm:id/subject"},\
                    value = self.subject)()

        ui_steps.edit_text(serial = self.serial,view_to_find = {"resourceId":\
                    "com.google.android.gm:id/body"},
                    value =  self.body_mail)()
        if(self.send_mail):
            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"resourceId":"com.google.android.gm:id/send"})()

        if(self.save_as_draft):
            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"descriptionContains":"More options"})()
            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"text":"Save draft"})()

    def check_condition(self):
        if(self.save_as_draft):
            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"descriptionContains":"Navigate up"})()

        ui_steps.click_button_if_exists(serial = self.serial,\
            view_to_find = {"descriptionContains":"Open navigation drawer"},\
            wait_time = 5000)()
        if(self.send_mail):
            ui_steps.click_button_if_exists(serial = self.serial,\
                view_to_find = {"text":"Sent"},wait_time = 5000)()
            stat = ui_steps.wait_for_view(serial = self.serial,\
                view_to_find = {"descriptionContains" : self.body_mail},\
                timeout =20)

        if(self.save_as_draft):
            ui_steps.click_button_if_exists(serial = self.serial,\
                view_to_find = {"text":"Drafts"},wait_time = 5000)()

            ui_steps.click_button_if_exists(serial = self.serial,\
                view_to_find = {"descriptionContains":self.body_mail},\
                wait_time = 5000)()

            ui_steps.click_button_if_exists(serial = self.serial,\
                view_to_find = {"descriptionContains":"Edit"},\
                wait_time = 5000)()

            ui_steps.edit_text(serial = self.serial,\
                    view_to_find ={"resourceId":\
                    "com.google.android.gm:id/composearea_tap_trap_bottom"},\
                    value = " new text editted")()

            stat = self.uidevice(textContains = "new text editted").exists

            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"descriptionContains":"More options"})()

            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"text":"Discard"})()

            ui_steps.click_button_if_exists(serial = self.serial,\
               view_to_find = {"text":"Discard"})()

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
            {"descriptionContains" : "Open navigation drawer"},\
            wait_time = 5000)()

        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"text" : "Primary"},wait_time = 5000)()

        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : self.subject},wait_time = 90000)()

    def check_condition(self):

        stat = ui_steps.wait_for_view(view_to_find =\
            {"descriptionContains" : self.body_mail}, timeout = 30,\
            serial = self.serial)()

        if self.delete:
            ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : "Delete"},wait_time = 5000)()
        return stat

class add_the_second_account_fromGmail(ui_step,adb_step):

    """description:
            add a second account from Gmail app

        usage:
            gmail_steps.add_the_second_account_fromGmail(account = "<ACCOUNT>@gmail.com")()

        tags:
            ui, gmail, mail, account, gmail account
    """

    def __init__(self,
                 account = None,
                 password = None,
                 subject = None,
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.account = account
        self.subject = subject
        self.password = password
        self.acc_no = gms_utils.get_google_account_number(serial = self.serial)

    def do(self):

        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,\
                view_to_find = {"text": "Gmail"})()
        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : "Open navigation drawer"},\
            wait_time = 5000)()

        ui_steps.click_button(view_to_find = {"resourceId":
                    "com.google.android.gm:id/account_list_button"},
                    view_to_check = {"text":"Add account"})()

        ui_steps.click_button(view_to_find = {"text":"Add account"})()
        while not self.uidevice(resourceId="com.google.android.gm:id/google_option").\
            child(resourceId="com.google.android.gm:id/radio_button").checked:
            ui_steps.click_button(serial = self.serial,\
            view_to_find = {"text":"Google"})()

        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"Next"})()

        self.uidevice(text="Checking info").wait.gone(timeout = 30000)

        ui_steps.click_button(serial = self.serial,
                     view_to_find = {"descriptionContains":"Enter your email"},
                     view_to_check = {"descriptionContains":"Sign in"})()

        ui_steps.edit_text(serial = self.serial,
                  view_to_find = {"className":
                              "android.widget.EditText"},
                                  value = self.account,
                                  is_password = True)()
        #press enter keycode
        self.uidevice.press(66)
        self.uidevice(text = self.account).wait.exists(timeout = 5000)

        ui_steps.edit_text(serial = self.serial,
              view_to_find = {"className":
                          "android.widget.EditText"},
                              value = self.password,
                              is_password = True)()
        #press enter keycode
        self.uidevice.press(66)

        ui_steps.wait_for_view(view_to_find = {"description":"ACCEPT"}, timeout = 30)()
        for f in range(1,10):
                if not self.uidevice(description = "ACCEPT").exists:
                    break
                else:
                    # get object coords
                    x, y = [c for c in ui_utils.get_center_coords(self.uidevice(
                        description = "ACCEPT").info["bounds"])]
                    cmd = "input tap {0} {1}".format(x,y)
                    self.adb_connection.run_cmd(cmd, timeout = 6)
                    self.uidevice(text = "Next").wait.exists(timeout = 20000)


        self.uidevice(text = "Sync your account").wait.exists(timeout = 40000)

        #~ ui_steps.click_button(serial = self.serial,
                 #~ view_to_find = {"text":"Next"},
                 #~ view_to_check = {"text":"Primary"})()


    def check_condition(self):
        new_acc_no = gms_utils.get_google_account_number(serial = self.serial)
        return new_acc_no == self.acc_no + 1

class switch_gmail_accounts(ui_step):

    """description:
            Switch between gmail accounts

        usage:
            gmail_steps.switch_gmail_accounts(serial = serial,)()

        tags:
            ui, gmail, mail, account, gmail account
    """

    def __init__(self,
                 account_to_switch = None,
                 password = None,
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.account_to_switch = account_to_switch
        self.password = password

    def do(self):

        ui_steps.press_home(serial = self.serial)()
        ui_steps.close_app_from_recent(serial = self.serial,view_to_find=\
            {"text": "Gmail"})()
        ui_steps.open_app_from_allapps(serial = self.serial,\
                view_to_find = {"text": "Gmail"})()
        ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
            {"descriptionContains" : "Open navigation drawer"},\
            wait_time = 5000)()
        if ui_steps.wait_for_view(view_to_find = {"descriptionContains":\
            "Switch to "+self.account_to_switch},timeout = 20, serial = self.serial):
                ui_steps.click_button_if_exists(serial = self.serial,view_to_find =\
                {"descriptionContains" : "Switch to "+self.account_to_switch},\
                wait_time = 5000)()

    def check_condition(self):
        return ui_steps.wait_for_view(view_to_find = {"text":\
            self.account_to_switch},timeout = 20, serial = self.serial)
