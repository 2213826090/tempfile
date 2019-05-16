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
@summary: This file implements for install/uninstall apk
@since: 06/09/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import os
import time
from testlib.util.common import g_common_obj

class ApkInstallUninstallImpl:
    """
    @summary: The basic function to install/uninstall apk
    """

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_apps_customize(self):
            """ UI button apps customize pane content """
            return self.d(resourceId=\
                 "com.google.android.googlequicksearchbox:id/apps_customize_pane_content",\
                 scrollable="true")

        @property
        def btn_apps(self):
            """ UI button apps """
            return self.d(description="Apps")

        @property
        def btn_apps_list(self):
            """ UI button apps list """
            return self.d(resourceId="android:id/list")

        def app_in_list(self, apkname):
            """ UI button app in app list """
            return self.btn_apps_list.child_by_text(apkname, \
            resourceId="com.android.settings:id/app_name", \
            allow_scroll_search=True)

        @property
        def btn_uninstall(self):
            """ UI button uninstall """
            return self.d(text="Uninstall", \
            resourceId="com.android.settings:id/right_button")

        @property
        def btn_ok(self):
            """ UI button OK """
            return self.d(text="OK")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/inactive")

    apksetting_package = "com.android.settings"
    apksetting_activity = "com.android.settings.applications.ManageApplications"
    def __init__ (self, cfg):
        self._device = g_common_obj.get_device()
        self._locator = ApkInstallUninstallImpl.Locator(self._device)
        self.cfg = cfg

    def apk_check(self, apk_name):
        """
        @summary: verify if the apk is existed
        @parameter:
            apk_packagename : the package name of apk
        @return: True or False
        """
        #check if the apk_name is existed
        try:
            if apk_name is None:
                print("ERROR: The apk name is none! Please check it!")
                message = "The apk name should not be None!"
                raise ValueError
        except ValueError:
            print message
        count = int(self._locator.btn_inactive.count)
        self._device.press.home()
        self._locator.btn_apps.click()
        time.sleep(2)  
        for i in range (0, count*2):
            time.sleep(2)
            if self._device(text=apk_name).exists:
                print("INFO:The apk has been installed!")
                return True

            if i < count:
                self._locator.btn_apps_customize.scroll.horiz()
            else:
                self._locator.btn_apps_customize.scroll.horiz.backward()

        print("INFO:The apk is not installed!")
        return False

    def apk_install(self, file_name, apk_name, website=None):
        """
        @summary: install the apk by adb command
        @parameter:
            file_name : the name of apk which you want to install
            apk_name : the name of apk display in apps list
            website : the website to download the apk
        @return: True or False
        """
        #check if the file is existed
        try:
            if file_name is None:
                print("ERROR: The file is none! Please check it!")
                message = "The file should not be None!"
                raise ValueError
            if apk_name is None:
                print("ERROR: The apk name is none! Please check it!")
                message = "The apk name should not be None!"
                raise ValueError
        except ValueError:
            print message

        resource = "/tmp/resource_apk/" + file_name
        check_file = os.path.exists(resource)
        if not check_file:
            print("The file is not existed: %s" % resource)
            try:
                if not self.apk_download(website):
                    print("ERROR: Download error!")
                    raise OSError
            except OSError:
                print "Download Error!"

        if self.apk_check(apk_name):
            print "WARNING:APK has been installed.Skip this!"
            return True

        #install apk
        cmd = "adb install %s; echo $?" % resource
        try:
            pipe = os.popen(cmd).read().rstrip()
            time.sleep(10)
            #print "[@@Grace_Debug]:Adb install"
            #print "[@@Grace_Debug]:", pipe, type(pipe)
            if "0" != pipe[-1]:
                print("ERROR: Adb install failed!")
                raise OSError
        except OSError:
            print "Install apk failed"
        #check if the apk is installed
        assert self.apk_check(apk_name), "ERROR: The apk is not in Apps"
        print("SUCCESS: Install APK Successfully!")
        return True

    def apk_uninstall(self, apk_name):
        """
        @summary: uninstall the apk by uiautomator
        @parameter:
            apk_name : the name of apk which you want to uninstall
        @return: True or False
        """

        self._device.press.home()
        #launch Settings/Apps
        # adb shell am start -n  com.android.settings/com.android.settings.
        #applications.ManageApplications
        g_common_obj.launch_app_am(\
            ApkInstallUninstallImpl.apksetting_package, \
            ApkInstallUninstallImpl.apksetting_activity)
        time.sleep(2)
        assert self._locator.btn_apps_list.exists, "Launch apps failed!"
        #press apk_name
        assert self._locator.app_in_list(apk_name) != None, \
        "ERROR: The apk is not found in Apps list!"

        self._locator.app_in_list(apk_name).click()
        time.sleep(1)
        #press uninstall button
        self._locator.btn_uninstall.click()
        time.sleep(2)
        #press ok button on uninstall dialog
        self._locator.btn_ok.click()
        time.sleep(1)
        #check if the apk package is deleted
        assert not self.apk_check(apk_name), "ERROR: The apk is still in Apps"
        print("SUCCESS: Uninsall Successfully!")
        return True

    @staticmethod
    def apk_download(apk_website):
        """
        @summary: download the apk from internet by wget command
        @parameter:
            apk_website : the website to download the apk
        @return: True or False
        """
        #verify the website
        if apk_website is None:
            print("ERROR: The website is None!")
            raise
        cmd = "mkdir /tmp/resource_apk;cd /tmp/resource_apk;wget -c %s"\
         % apk_website
        pipe = os.system(cmd)
        if pipe != 0:
            print("ERROR:Download file error!\
                Please check the website [%s]" % apk_website)
            return False
        print("SUCCESS:Download finish!")
        return True

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self._device.press.home()
        self._locator.btn_apps.click()
        time.sleep(2)
        count = int(self._locator.btn_inactive.count)
        for i in range(0, count*2):
            time.sleep(2)
            if self._device(text=appname).exists:
                self._device(text=appname).click()
                iffind = True
                break
            if i < count:
                self._device(scrollable=True).scroll.horiz()
            else:
                self._device(scrollable=True).scroll.horiz.backward()
        assert iffind == True