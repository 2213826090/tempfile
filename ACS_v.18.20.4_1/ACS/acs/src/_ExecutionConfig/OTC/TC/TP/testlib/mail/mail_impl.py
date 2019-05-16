#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

import time
import os
from nose.tools import assert_equals
from testlib.util.common import g_common_obj


class EmailImpl:
    """
    Implements Mail app actions.
    """

    mail_pkg_name = "com.android.email"
    mail_activity_name = "com.android.email.activity.Welcome"


    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    @staticmethod
    def set_sync_watcher():
        """
        Set sync watcher to sync account
        """
        g_common_obj.get_device().watcher("SKIP_SYNC").when(
            text="Sync now").click(text="Sync now")

    @staticmethod
    def remove_sync_watcher():
        """
        Remove sync watcher
        """
        g_common_obj.get_device().watcher("SKIP_SYNC").remove()


    @staticmethod
    def launch_by_am():
        """
        launch by am
        """
        g_common_obj.launch_app_am(EmailImpl.mail_pkg_name, \
            EmailImpl.mail_activity_name)
        print "[INFO] launch mail app successfully"

    def startApp(self):
        """
        Skip Email first launch screen
        """
        g_common_obj.launch_app_from_home_sc("Email")
        assert self.d(packageName = EmailImpl.mail_pkg_name).wait.exists()
        if self.d(textContains="Getting").exists:
            self.d(textContains="Getting").wait.gone(timeout=600000)
        assert not self.d(textContains="Getting").exists, \
        "ERROR: SYNC for more than 10 mins"
        if self.d(resourceId="com.android.email:id/account_email").exists:
            self.d(resourceId="com.android.email:id/account_email").click()
            g_common_obj.adb_cmd_capture_msg(
                "input text " + self.cfg.get("email_account"))
            time.sleep(2)
            self.d(resourceId="com.android.email:id/next").click()
            self.d(resourceId="com.android.email:id/regular_password").\
            wait.exists()
            self.d(resourceId="com.android.email:id/regular_password").click()
            time.sleep(2)
            g_common_obj.adb_cmd_capture_msg(
                "input text " + self.cfg.get("email_pwd"))
            time.sleep(2)
            self.d(resourceId="com.android.email:id/next").click()
            assert self.d(text="Account options").wait.exists(timeout=60000)
            for _ in (0, 10):
                if self.d(
                    resourceId="com.android.email:id/next")\
                .wait.exists(timeout=60000):
                    self.d(resourceId="com.android.email:id/next").click()
                else:
                    break
            assert self.d(text="Inbox").wait.exists(timeout=60000)

        g_common_obj.back_home()

    # select an acount to send email
    def select_account(self, account):

        if self.d(text=account).exists==False:
            self.d(description="Open navigation drawer").click()
        if self.d(description="Show accounts").exists:
            self.d(description="Show accounts").click()
        tmp = "Switch to %s" % account
        print tmp
        if self.d(description=tmp).exists==True:
            self.d(description=tmp).click()
        else:
            assert self.d(text=account).exists, \
            "[ERROR]: The sender account %s does not login" % account
        self.d(description="Close navigation drawer").click()
        print "[INFO] select account %s successfully" % account

    # make a screenshot as attachment to send
    def make_a_screenshot(self, name):
        """
        screen shot
        """
        g_common_obj.adb_cmd("screencap /sdcard/Pictures/%s" % name)
        print "[INFO] screenshot successfully"

    # add a new contact in the contact app
    def compose_a_new_email(self, to, subject, content):
        """
        compose a new email
        """
        self.d(
            resourceId="com.android.email:id/compose_button")\
        .click.wait(timeout=60000)
        time.sleep(2)
        self.d(resourceId="com.android.email:id/subject").set_text(subject)
        self.d(resourceId="com.android.email:id/body").set_text(content)
        self.d(resourceId="com.android.email:id/to").set_text(to)
        time.sleep(2)

    def screenshot_and_attach(self, base_path, file_name):
        """
        screen shoot and attached
        """
        #push png file into /sdcard/Picture
        print "[INFO] adb shell screencap %s%s" % (base_path, file_name)
        g_common_obj.adb_cmd("screencap %s%s" % (base_path, file_name))
        g_common_obj.adb_cmd("am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://sdcard ")
        #attache the png file
        #self.d(description="More options").click()
        self.d(description="Attach file").click()
        self.d(text="Attach file").click()
        #self.d(description="More options").click()
        self.d(text="Photos").click()
        #self.d(textContains=file_name).click()
        self.d(className="android.view.View", packageName="com.google.android.apps.plus", clickable="true").click()
        self.d(text="SELECT").click()
        time.sleep(5)
        assert self.d(text="Photos").exists==False, "[INFO] attach file failed"

    def send_email(self):
        self.d(resourceId="com.android.email:id/send").click()
        time.sleep(10)
        assert self.d(resourceId="com.android.email:id/send").exists==False, "[INFO] send email failed"
        print "[INFO] send email successfully"

    def delete_screenshot_file(self, base_path, file_name):
        #delet png file into /sdcard/Picture
        print "[INFO] adb shell rm %s%s" % (base_path, file_name)
        g_common_obj.adb_cmd("rm %s%s" % (base_path, file_name))
        g_common_obj.adb_cmd("am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://sdcard ")


class MailImpl:
    """
    Implements Mail app actions.
    """

    #mail_pkg_name = "com.android.email"
    #mail_activity_name = ".activity.Welcome"
    #mail_pkg_name="com.google.android.email"
    #mail_activity_name="com.android.email.activity.Welcome"
    mail_pkg_name="com.google.android.gm"
    mail_activity_name="com.google.android.gm.ConversationListActivityGmail"
    vcf_file_name = ""

    def __init__ (self, wifi_cf={}):
        self.d = g_common_obj.get_device()

    @staticmethod
    def set_sync_watcher():
        """
        Set sync watcher to sync account
        """
        g_common_obj.get_device().watcher("SKIP_SYNC").when(
            text="Sync now").click(text="Sync now")

    @staticmethod
    def remove_sync_watcher():
        """
        Remove sync watcher
        """
        g_common_obj.get_device().watcher("SKIP_SYNC").remove()

    # launch contact app by am
    def launch_by_am(self):
        """
        launch by am
        """
        g_common_obj.launch_app_am(MailImpl.mail_pkg_name, \
            MailImpl.mail_activity_name)
        print "[INFO] launch mail app successfully"

    @staticmethod
    def startApp():
        """
        Skip Gmail first launch screen
        """
        g_common_obj.launch_app_from_home_sc("Gmail")
        time.sleep(10)
        d = g_common_obj.get_device()
        while d(
            resourceId="com.google.android.gm:id/welcome_tour_got_it").exists:
            d(
                resourceId="com.google.android.gm:id/welcome_tour_got_it").click()
            d.wait.update()
        while d(resourceId="com.google.android.gm:id/action_done").exists:
            d(resourceId="com.google.android.gm:id/action_done").click()
            d.wait.update()

        g_common_obj.back_home()

    # select an acount to send email
    def select_account(self, account):

        if self.d(text=account).exists==False:
            self.d(description="Open navigation drawer").click()
        if self.d(description="Show accounts").exists:
            self.d(description="Show accounts").click()
        tmp="Switch to %s" % account
        print tmp
        if self.d(description=tmp).exists==True:
            self.d(description=tmp).click()
        else:
            assert self.d(text=account).exists, "[ERROR]: The sender account %s does not previously setup login" % account
        self.d(description="Close navigation drawer").click()
        print "[INFO] select account %s successfully" % account

    # make a screenshot as attachment to send
    def make_a_screenshot(self, name):
        g_common_obj.adb_cmd("screencap /sdcard/Pictures/%s" % name)
        print "[INFO] screenshot successfully"

    # add a new contact in the contact app
    def compose_a_new_email(self,to,subject, content):
        self.d(resourceId="com.google.android.gm:id/compose_button").click()
        time.sleep(2)
        self.d(resourceId="com.google.android.gm:id/subject").set_text(subject)
        self.d(resourceId="com.google.android.gm:id/body").set_text(content)
        self.d(resourceId="com.google.android.gm:id/to").set_text(to)
        time.sleep(2)

    def screenshot_and_attach(self, base_path, file_name):
        #push png file into /sdcard/Picture
        print "[INFO] adb shell screencap %s%s" % (base_path, file_name)
        g_common_obj.adb_cmd("screencap %s%s" % (base_path, file_name))
        g_common_obj.adb_cmd("am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://sdcard ")
        #attache the png file
        #self.d(description="More options").click()
        self.d(description="Attach file").click()
        self.d(text="Attach file").click()
        #self.d(description="More options").click()
        self.d(text="Photos").click()
        #self.d(textContains=file_name).click()
        self.d(className="android.view.View", packageName="com.google.android.apps.plus", clickable="true").click()
        self.d(text="SELECT").click()
        time.sleep(5)
        assert self.d(text="Photos").exists==False, "[INFO] attach file failed"

    def send_email(self):
        self.d(resourceId="com.google.android.gm:id/send").click()
        time.sleep(10)
        assert self.d(resourceId="com.google.android.gm:id/send").exists==False, "[INFO] send email failed"
        print "[INFO] send email successfully"

    def delete_screenshot_file(self, base_path, file_name):
        #delet png file into /sdcard/Picture
        print "[INFO] adb shell rm %s%s" % (base_path, file_name)
        g_common_obj.adb_cmd("rm %s%s" % (base_path, file_name))
        g_common_obj.adb_cmd("am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://sdcard ")