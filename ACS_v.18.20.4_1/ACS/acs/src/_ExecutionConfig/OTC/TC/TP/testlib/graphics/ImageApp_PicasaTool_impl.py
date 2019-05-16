# Copyright (C) 2015  Zhao, XiangyiX <xiangyix.zhao@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for ImageApp PicSay
@since: 06/10/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.compare_pic_impl import compare_pic

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def sign_in(self):
        return self._device(textContains="Sign In")

    @property
    def select_account(self):
        return self._device(resourceId="android:id/text1", textContains="@gmail.com")

    @property
    def english(self):
        return self._device(resourceId="larry.zou.colorfullife:id/first_button", textContains="English")

    @property
    def ok(self):
        return self._device(text="OK")

    @property
    def upload(self):
        return self._device(
            resourceId="larry.zou.colorfullife:id/menu_upload")

    @property
    def choose_pic(self):
        return self._device(resourceId=\
            "larry.zou.colorfullife:id/v_thumbnail_container")

    @property
    def edit_btn(self):
        return self._device(resourceId=\
            "larry.zou.colorfullife:id/menuEdit")

    @property
    def copy_and_edit(self):
        return self._device(className=\
            "android.widget.ListView")\
            .child(className="android.widget.LinearLayout")

    @property
    def enhance_btn(self):
        return self._device(resourceId=\
            "larry.zou.colorfullife:id/aviary_tools_listview")\
            .child(className="android.widget.LinearLayout", index=0)

    @property
    def hidef_btn(self):
        return self._device(resourceId=\
            "larry.zou.colorfullife:id/aviary_content_view")\
            .child(className="com.aviary.android.feather.sdk.widget.AviaryHighlightImageButton", index=0)

    @property
    def apply(self):
        return self._device(resourceId=\
            "larry.zou.colorfullife:id/navbar_button2")

    @property
    def done(self):
        return self._device(text="Done")

class PicasaTool:
    '''
    classdocs
    '''

    pkg_name = "larry.zou.colorfullife"
    activity_name = "com.colure.pictool.ui.Main_"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        config_handle = ConfigHandle()
        self.id = config_handle.read_configuration('google_account', 'user_name', '/etc/oat/', 'sys.conf')
        self.pwd = config_handle.read_configuration('google_account', 'password', '/etc/oat/', 'sys.conf')

    def launch_app_am(self):
        """ Launch PicasaTool via adb am command
        """
        print "Launch PicasaTool by adb am"
        g_common_obj.launch_app_am(\
            PicasaTool.pkg_name, PicasaTool.activity_name)
        time.sleep(2)
        if self._device(resourceId="larry.zou.colorfullife:id/v_txt").exists:
            self._device(text="Cancel").click.wait()
            time.sleep(2)
        self._locator.sign_in.click.wait()
        time.sleep(2)
        assert self._locator.select_account.exists, "Please sign in a google account!"
        self._locator.select_account.click.wait()
        time.sleep(2)
        for _ in range(5) :
            time.sleep(12)
            if self._device(text="Picasa Tool would like to:").exists:
                if self._device(resourceId="com.google.android.gms:id/accept_button", textContains="Allow").exists:
                    self._device(resourceId="com.google.android.gms:id/accept_button", textContains="Allow").click.wait()
                if self._device(textContains="OK").exists:
                    self._device(textContains="OK").click.wait()
                time.sleep(2)
            if self._device(text="Picasa Web Albums").exists:
                self._device(textContains="OK").click.wait()
            if self._device(description="Please sign in").exists:
                self._device(description="NEXT").click.wait()
                time.sleep(6)
                username = self.id
                assert self._device(description=username).exists, "Please use pretest google account!"
                self._device(className="android.widget.EditText").set_text(self.pwd)
                time.sleep(2)
                self._device(description="NEXT").click.wait()
                time.sleep(6)
            if self._device(resourceId="com.google.android.gms:id/app_name").exists:
                self._device(textContains="OK").click.wait()
                time.sleep(6)
            if self._device(text="You have no Picasa web albums. Upload your first photo today!").exists:
                self._device(resourceId="larry.zou.colorfullife:id/second_button", textContains="Cancel").click.wait()
                time.sleep(2)
            if self._device(text="Picasa Tool 7.6.3.2").exists:
                if self._locator.english.exists:
                    self._locator.english.click.wait()
                    time.sleep(2)
                if self._locator.ok.exists:
                    self._locator.ok.click.wait()
                    time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop PicasaTool via adb am command
        """
        print "Stop PicasaTool by adb am"
        g_common_obj.stop_app_am(PicasaTool.pkg_name)

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def choose_pic(self):
        """ choose picture to edit
        """
        assert self._locator.upload.exists, "set up PicasaTool fail!"
        self._locator.upload.click()
        time.sleep(2)
        if self._locator.ok.exists:
            self._locator.ok.click()
            time.sleep(1)
        self._locator.choose_pic.click()
        time.sleep(2)

    def edit_pic(self):
        """ edit picture
        """
        self._locator.edit_btn.click()
        time.sleep(4)
        self._locator.copy_and_edit.click()
        time.sleep(2)
        print "catch screenshot before_effect.png"
        self._device.screenshot("before.png")
        time.sleep(2)
        self._locator.enhance_btn.click()
        time.sleep(2)
        self._locator.hidef_btn.click()
        time.sleep(2)
        self._locator.apply.click()
        time.sleep(2)
        print "catch screenshot after_effect.png"
        self._device.screenshot("after.png")
        time.sleep(1)
        self._locator.done.click()
        time.sleep(2)

    def check_pic(self):
        """ check edit picture effect via compare 2 screenshot and copy image exists
        """
        path = os.getcwd()
        print "current path is " + path
        time.sleep(1)
        rms = compare_pic.compare_pic(path + "/before.png", path + "/after.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms >= 20, "The screenshot is same with before add effect"

    def uninstall_app(self):
        """ uninstall PicasaTool
        """
        print "uninstall PicasaTool"
        cmd = 'uninstall %s' % PicasaTool.pkg_name
        g_common_obj.adb_cmd_common(cmd)
