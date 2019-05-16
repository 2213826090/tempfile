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

"""
@summary: module for App manipulate
@since: 10/15/2014
@author: samlanx (samx.lan@intel.com)
"""

import os

from testlib.util.common import g_common_obj

class ApkFilter(object):

    def filter(self, apkfile):
        """
            @summary: filter apk
            @param apkfile: absolute path of apk file
            @return: True -- install the apk, False -- skip install the apk
        """
        if os.path.isfile(apkfile) and os.path.splitext(apkfile)[1] == ".apk":
            return True
        return False

class ARMApkFilter(ApkFilter):

    def filter(self, apkfile):
        if super(ARMApkFilter, self).filter(apkfile):
            cmd = 'unzip -l %s | grep "lib/armeabi"; echo $?' %apkfile
            res_list = os.popen(cmd).readlines()
            ret = int(res_list[-1].strip())
            if ret != 0:
                print "[INFO] %s is not ARM apk" % apkfile
            return ret == 0
        return False

armApkFilter = ARMApkFilter()

class AppImpl(object):
    '''
        class for App install/uninstall
    '''

    def __init__(self, device):
        self.d = device
        self.new_apps = []

    def install_apps(self, apkfolder, apkfilter=None):
        """
            @summary: Install apps whose apk under one folder
            @param apkfolder: folder placed apks install file of apps
            @param apkfilter: filter to choose apk,
                if filter turn True, install the apk, otherwise skip install
            @return: install successed apps
        """
        print "[INFO] Install Apps under folder: %s" % apkfolder
        suc_cnt = 0
        fail_cnt = 0
        for apkfile in os.listdir(apkfolder):
            apk_path = os.path.join(apkfolder, apkfile)
            if apkfilter.filter(apk_path):
                install_cmd = 'adb install -r "%s"' % apk_path
                pipe = os.popen(install_cmd).read()
                if "Success" in pipe:
                    suc_cnt += 1
                    print "[INFO] Install App success: %s" % apkfile
                else:
                    fail_cnt += 1
                    print "[ERROR] Install App fail: %s" % apkfile
        print "[INFO] Try to install %d apps, Success: %d, Fail: %d" % (suc_cnt+fail_cnt, suc_cnt, fail_cnt)
        return self.get_user_apps()

    def uninstall_apps(self, apps=None):
        """
            @summary: Uninstall apps
            @param apps: list of apps' package name,
                if apps=None, will get all user installed apps on device
        """
        print "[INFO] Uninstall apps"
        fail_cnt = 0
        if not apps:
            apps = self.get_user_apps()
        for app in apps:
            app_package = app.strip().split(':')[1]
            uninstall_cmd = 'adb uninstall %s' % app_package
            pipe = os.popen(uninstall_cmd).read()
            if "Success" in pipe:
                print "[INFO] Uninstall App success: %s" % app_package
            else:
                fail_cnt += 1
                print "[ERROR] Uninstall App fail: %s" % app_package
        assert fail_cnt == 0, "%d apps uninstall fail" % fail_cnt

    def get_user_apps(self):
        """
            @summary: Get user installed apps
        """
        cmd = 'pm list packages -3; echo $?'
        apps = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')[0:-1]
        return apps

    def launch_app_by_am(self, packagename, activityname):
        """
            @summary: launch app by am
        """
        g_common_obj.launch_app_am(packagename, activityname)
        print "[INFO] launch package: %s, activity: %s successfully" % (packagename, activityname)

    def stop_app_by_am(self, packagename):
        """
            @summary: stop app by am
        """
        g_common_obj.stop_app_am(packagename)
        print "[INFO] stop package: %s successfully" % packagename

    def check_app_launched_successfully(self, packagename, activityname):
        """
            @summary: check the app is launched successfully
        """
        app="%s/%s" %(packagename, activityname)
        cmd="dumpsys window|grep mCurrentFocus"
        currentfocus=g_common_obj.adb_cmd_capture_msg(cmd).strip()
        currentfocus_result=currentfocus.find(app)
        
        #handle the alternative windows for chrome
        if packagename=="com.android.chrome":
            app="com.android.chrome/com.google.android.apps.chrome"
            currentfocus=g_common_obj.adb_cmd_capture_msg(cmd).strip()
            currentfocus_result=currentfocus.find(app)
        #handle the alternative windows for photos
        elif packagename=="com.google.android.apps.photos":
            app="google.android.apps.plus/com.google.android.apps.photos.phone.PhotosHomeActivity"
            currentfocus=g_common_obj.adb_cmd_capture_msg(cmd).strip()
            currentfocus_result=currentfocus.find(app)
            if currentfocus_result==-1:
                app="com.google.android.apps.plus/.phone.PhotosIntroActivity"
                currentfocus=g_common_obj.adb_cmd_capture_msg(cmd).strip()
                currentfocus_result=currentfocus.find(app)
        #handle the alternative windows for music
        elif packagename=="com.google.android.music":
            if currentfocus_result==-1:
                app="com.google.android.music/.tutorial.TutorialSelectAccountActivity"
                currentfocus=g_common_obj.adb_cmd_capture_msg(cmd).strip()
                currentfocus_result=currentfocus.find(app)
        #########################
        errormessage= "[Error]: the app %s does not launch successfully" % app
        assert currentfocus_result!=-1, errormessage
        print "[INFO]: launch app successfully"
