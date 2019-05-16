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

class SlidesImpl:
    """
    Slides Test Impl Class
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_add(self):
            """ UI button settings """
            return self.d(descriptionContains="New")

        @property
        def btn_overflow(self):
            """ UI button account """
            return self.d(textContains="Untitled")

        def btn_next(self):
            """
            Next button
            """
            return self.d(resourceId="com.google.android.apps.docs.editors.slides:id/next")

        def btn_done(self):
            """
            Done button
            """
            return self.d(resourceId="com.google.android.apps.docs.editors.slides:id/done")

    packagename = "com.google.android.apps.docs.editors.slides"
    activityname = "com.google.android.apps.docs.app.DocListActivity"


    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self._locator = SlidesImpl.Locator(self.d)
        self.cfg = cfg

    @staticmethod
    def launch_from_am():
        """
            Launch Slides from am
        """
        print "[INFO] Launch Slides from am"
        g_common_obj.launch_app_am(SlidesImpl.packagename, \
            SlidesImpl.activityname)
        time.sleep(2)

    @staticmethod
    def startApp():
        """
        Skip Slides first launch screen
        """
        g_common_obj.launch_app_from_home_sc("Slides")
        d = g_common_obj.get_device()
        time.sleep(10)
        if d(descriptionContains="New").exists:
            g_common_obj.back_home()
            return
        while d(
            resourceId="com.google.android.apps.docs.editors.slides:id/next").\
        wait.exists(timeout=3000):
            d(resourceId="com.google.android.apps.docs.editors.slides:id/next").click()
            d.wait.update()
        if d(resourceId="com.google.android.apps.docs.editors.slides:id/done").\
        wait.exists(timeout=60000):
            d(resourceId="com.google.android.apps.docs.editors.slides:id/done").click()
        g_common_obj.back_home()

    def basic_test(self):
        """
        Basic funtion test
        """
        g_common_obj.launch_app_from_home_sc("Slides")
        if not self._locator.btn_add.exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.apps.docs.editors.slides").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/slides.png")
            self.d.press.back()
            return
        self._locator.btn_add.click()
        time.sleep(10)
        if not self._locator.btn_overflow.exists:
            print "[WARNING:UI is changed! Just test add function]"
            assert self.d(packageName="com.google.android.apps.docs.editors.slides").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/slides.png")
            self.d.press.back()
            return
        assert self._locator.btn_overflow.exists
        g_common_obj.back_home()
