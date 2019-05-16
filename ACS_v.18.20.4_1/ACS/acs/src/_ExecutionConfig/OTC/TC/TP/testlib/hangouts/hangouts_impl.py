#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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

"""
@summary: This file implements for hangouts
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj

class HangoutsImpl:
    """
    Hangouts Test Impl Class
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_people(self):
            """ UI button settings """
            return self.d(descriptionContains="People")

        @property
        def btn_conver(self):
            """ UI button settings """
            return self.d(descriptionContains="Conversations")

        @property
        def btn_tabhost(self):
            """ UI button settings """
            return self.d(resourceId="android:id/tabhost")

        @property
        def btn_account(self):
            """ UI button account """
            return self.d(
                resourceId="com.google.android.apps.hangouts:id/account_display_name")
        @property
        def btn_skip(self):
            """
            Skip button
            """
            return self.d(
                resourceId="com.google.android.apps.hangouts:id/call_promo_no_thanks")
        @property
        def btn_ok(self):
            return self.d(textContains="Okay")

    packagename = "com.google.android.talk"
    activityname = "com.google.android.talk.SigningInActivity"


    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self._locator = HangoutsImpl.Locator(self.d)
        self.cfg = cfg

    def launch_from_am(self):
        """
            Launch Browser from am
        """
        print "[INFO] Launch Chrome from am"
        g_common_obj.launch_app_am(HangoutsImpl.packagename, \
            HangoutsImpl.activityname)
        time.sleep(2)
        if self._locator.btn_ok.exists:
            self._locator.btn_ok.click()
            time.sleep(2)

    @staticmethod
    def startApp():
        """
        Skip Google Plus first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Hangouts")
        time.sleep(10)
        if d(resourceId="android:id/up").exists:
            g_common_obj.back_home()
            return
        while d(resourceId="com.google.android.apps.hangouts:id/call_promo_no_thanks").exists:
            d(resourceId="com.google.android.apps.hangouts:id/call_promo_no_thanks").click()
            d.wait.update()
        while d(textContains="Skip").exists:
            d(textContains="Skip").click()
        g_common_obj.back_home()

    def basic_test(self):
        """
        Basic funtion test
        """
        self.d.watcher("SKIP").when(text="SKIP").click(text="SKIP")
        self.launch_from_am()
        if self._locator.btn_account.exists:
            self._locator.btn_tabhost.swipe.left()
        if not self._locator.btn_people.exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.talk").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/hangouts.png")
            self.d.press.back()
            return
        self._locator.btn_people.click()
        if not self._locator.btn_conver.exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.talk").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/hangouts.png")
            self.d.press.back()
            return
        self._locator.btn_conver.click()
        self.d.press.back()