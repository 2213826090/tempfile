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
@summary: module for Search application
@since: 08/01/2014
@author: Grace yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj
import random

class SearchImpl(object):
    """
        class for Search application Home UI
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_search_bar(self):
            """ UI button google search """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/search_drop_target_bar")

        @property
        def btn_search_box(self):
            """ UI button google search """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/search_box")

        @property
        def btn_search_text(self):
            """ UI button google search """
            return self.btn_search_box.info.get("text").decode('gbk')

        @property
        def btn_search_plate(self):
            """ UI button google search """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/search_plate")

        @property
        def btn_get_google_now(self):
            """ UI button Get Google Now """
            return self.d(textContains="Get Google Now")

        @property
        def btn_select_account(self):
            """ UI button select account """
            return self.d(resourceId="com.android.contacts:id/account_list")

        @property
        def btn_account(self):
            """ UI button select account """
            return self.d(\
                className="android.widget.LinearLayout", index="0")

        @property
        def btn_account_sync(self):
            """ UI button account sync """
            return self.d(textContains="Your new contact will be synchronized")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(textContains="OK")

    packagename = "com.google.android.googlequicksearchbox"
    activityname = "com.google.android.velvet.ui.VelvetActivity"

    def __init__ (self, cfg):

        self.d = g_common_obj.get_device()
        self._locator = SearchImpl.Locator(self.d)
        self.cfg = cfg

    def launch_from_am(self):
        """
            Launch search from am
        """
        print "[INFO] Launch Search from am"
        g_common_obj.launch_app_am(SearchImpl.packagename, \
            SearchImpl.activityname)
        time.sleep(2)
        assert self._locator.btn_search_box.exists, "[FAILURE] Launch fail"

    @staticmethod
    def stop_from_am():
        """
            Stop search from am
        """
        print "[INFO] Stop search from am"
        g_common_obj.stop_app_am(SearchImpl.packagename)

    def search(self, text):
        """
        @summary: search text in google search
        """
        self.d.press.home()
        assert self._locator.btn_search_bar.exists, \
        "[FAILURE] Search bar is not found"
        self._locator.btn_search_bar.click()
        self._locator.btn_search_box.set_text(text)
        time.sleep(2)

    def check_search(self, icon):
        """
        @summary: check search
        """
        time.sleep(5)
        error_mesg = "[FAILURE] The %s is not found in search" % icon
        assert self.d(textContains=icon).exists, error_mesg
        print "[INFO] [%s] check pass" % icon

    def launch_app_from_search(self, app, check):
        """
        @summary: Launch app from search list
        """
        assert self.d(textContains=app).exists, \
        "[FAILURE] The app is not found in search"
        self.d(textContains=app, instance="0").click()
        print "[INFO] %s check pass" % app
        time.sleep(5)
        self.check_focus_window(check)
        print "[INFO] Check launch %s from search result pass!" % app

    @staticmethod
    def check_focus_window(windows):
        """
        @summary: check focus windows
        """
        cmd = "dumpsys window|grep -i mCurrentFocus"
        pipe = g_common_obj.adb_cmd_capture_msg(cmd)
        print "[INFO] Focus window is %s" % pipe
        assert not "Error" in pipe, "[FAILURE] Error pop up!"
        assert windows in pipe, "[FAILURE] Check focus window error!"

    @staticmethod
    def check_low_battery(battery_level):
        """
        @summary: Get battery level
        """
        cmd = "dumpsys battery|grep level"
        level = g_common_obj.adb_cmd_capture_msg(cmd).split()
        print "[INFO]: Current battery level is %s" % level[1]
        assert int(level[1]) <= int(battery_level), \
        "[FAILURE] Device is not in low battery status!Skip the case."

    def test_virtual_key(self):
        """
        test virtual keyboard
        """
        self.d.press.home()
        assert self._locator.btn_search_bar.exists, \
        "[FAILURE] Search bar is not found"
        self._locator.btn_search_bar.click()
        self._locator.btn_search_box.set_text("x")
        for _ in range (0, 10):
            x_location = random.randint(200, 600)
            y_location = random.randint(900, 1000)
            self.d.click(x_location, y_location)
        text = self._locator.btn_search_text
        print "[INTO] Input text is %s " % text

        assert not text is "x"