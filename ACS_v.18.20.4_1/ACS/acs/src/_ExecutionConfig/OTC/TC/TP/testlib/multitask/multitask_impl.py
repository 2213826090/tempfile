# coding: UTF-8
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
@summary: The test for multi task
@since: 07/02/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj

class MultiTaskImpl:
    """
    Multi-task functions.
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_apps(self):
            """ UI button apps """
            return self.d(description="Apps")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/inactive")

    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self._locator = MultiTaskImpl.Locator(self.d)
        self.cfg = cfg
        self.app = self.cfg.keys()
        self.app_package = self.cfg.values()

    def app_switch(self):
        """
        @summary: launch some app in home screen
        @return: None
        """
        for i in range (0, len(self.app)):
            print self.app[i]
            package = self.cfg.get(self.app[i]).split('/')[0]
            self.launch_app(self.app[i])
            time.sleep(5)
            self.check_error()
            assert self.focus_window(package), \
            "[FAILURE] Launch %s failed!" % self.app[i]

    def app_switch_fix(self):
        """
        @summary: launch some app by adb command
        @return: None
        """
        for i in range (0, len(self.app_package)):
            print self.app_package[i]
            package = self.app_package[i].split('/')
            g_common_obj.launch_app_am(package[0], package[1])
            time.sleep(5)
            self.check_error()
            assert self.focus_window(package[0]), \
            "[FAILURE] Launch %s failed!" % self.app[i]
            g_common_obj.stop_app_am(package[0])
            self.check_error()
            time.sleep(1)
            self.d.press.home()

    def launch_app(self, app):
        """
        @summary: press home and launch app by click
        @parameter:
            app : the name of app
        @return: None
        """
        self.launch_app_from_home_sc(app)

    def check_error(self):
        """
        @summary: check if error dialog popup
        @return: None
        """
        error_check = self.d(textContains="error").exists|\
        self.d(textContains="Can't").exists|\
        self.d(textContains=" stop").exists
        assert not error_check, "error pop up!"

    @staticmethod
    def focus_window(package):
        """
        @summary: check focus package
        """
        cmd = "dumpsys window|grep mCurrentFocus"
        message = g_common_obj.adb_cmd_capture_msg(cmd)
        print "[INFO] Current Focus window is %s" % message
        if package in message:
            return True
        return False

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self.d.press.home()
        self._locator.btn_apps.click()
        time.sleep(2)
        count = int(self._locator.btn_inactive.count)
        for i in range(0, count*2):
            time.sleep(2)
            if self.d(textContains=appname).exists:
                self.d(textContains=appname).click()
                iffind = True
                break
            if i < count:
                self.d(scrollable=True).scroll.horiz()
            else:
                self.d(scrollable=True).scroll.horiz.backward()
        assert iffind == True
