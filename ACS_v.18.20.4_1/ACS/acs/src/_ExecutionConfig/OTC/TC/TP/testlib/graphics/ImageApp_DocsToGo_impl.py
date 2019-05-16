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
@summary: Class for ImageApp DocsToGo
@since: 09/08/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import get_screenshot_region
from testlib.common.base import getTmpDir
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def create_file(self):
        return self._device(resourceId="com.dataviz.docstogo:id/create_file_button")

    @property
    def word_btn(self):
        return self._device(text="MS Word")

    @property
    def more_btn(self):
        return self._device(resourceId="com.dataviz.docstogo:id/button_more_e")

    @property
    def file_btn(self):
        return self._device(text="File")

    @property
    def edit_btn(self):
        return self._device(text="Edit")

    @property
    def format_btn(self):
        return self._device(text="Format")

    @property
    def select_all_btn(self):
        return self._device(text="Select All")

    @property
    def bold_btn(self):
        return self._device(text="Bold")

    @property
    def underline_btn(self):
        return self._device(text="Underline")

    @property
    def save_btn(self):
        return self._device(text="Save")

    @property
    def return_btn(self):
        return self._device(resourceId="com.dataviz.docstogo:id/button_back_e")

    @property
    def back_btn(self):
        return self._device(resourceId="com.dataviz.docstogo:id/back_button")

class DocsToGo:
    '''
    classdocs
    '''

    pkg_name = "com.dataviz.docstogo"
    activity_name = "com.dataviz.dxtg.common.launcher.android.LauncherActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.tmpdir = getTmpDir()
        self.systemui = SystemUiExtendImpl()

    def launch_app_am(self):
        """ Launch DocsToGo via adb am command
        """
        print "Launch DocsToGo by adb am"
        g_common_obj.launch_app_am(\
            DocsToGo.pkg_name, DocsToGo.activity_name)
        time.sleep(3)
        if self._device(text="Welcome!").exists:
            self._device(text="Next").click()
            time.sleep(1)
            self._device(text="Accept").click()
            time.sleep(1)
            self._device(text="Later").click()
            time.sleep(1)

    @staticmethod
    def stop_app_am():
        """ Stop DocsToGo via adb am command
        """
        print "Stop DocsToGo by adb am"
        g_common_obj.stop_app_am(DocsToGo.pkg_name)

    def clean_workaround(self, docx):
        """ Clean the workaround to avoid other file affect.
        """
        print "delete DocsToGo created documents folder"
        cmd = 'rm -rf sdcard/documents'
        g_common_obj.adb_cmd(cmd)
        print "delete the docx pushed"
        cmd = "rm -rf /sdcard/%s" % docx
        g_common_obj.adb_cmd(cmd)

    def edit_docx(self):
        """ Edit docx in DocsToGo.
        """
        if not self._locator.create_file.exists:
            self.check_ad()
        self._locator.create_file.click.wait()
        time.sleep(1)
        self._locator.word_btn.click.wait()
        time.sleep(2)
        cmd = "input text test"
        g_common_obj.adb_cmd(cmd)
        print "Excute:Select all"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.edit_btn.click.wait()
        time.sleep(1)
        self._locator.select_all_btn.click.wait()
        time.sleep(1)
        print "Excute:Add Format Bold"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.format_btn.click.wait()
        time.sleep(1)
        self._locator.bold_btn.click.wait()
        time.sleep(1)
        print "Excute:Add Format Underline"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.format_btn.click.wait()
        time.sleep(1)
        self._locator.underline_btn.click.wait()
        time.sleep(1)
        print "Excute:Save docx"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.file_btn.click.wait()
        time.sleep(1)
        self._locator.save_btn.click.wait()
        time.sleep(1)
        self._locator.save_btn.click.wait()
        time.sleep(1)
        if self._device(text="File already exists, overwrite?").exists:
            self._device(text="Yes").click.wait()
            time.sleep(1)
        self._locator.return_btn.click.wait()
        time.sleep(2)
        self.check_ad()

    def check_docx(self):
        """ get screenshot and check cocx which template docx by screenshot in DocsToGo.
        """
        assert self._device(text="Untitled.docx").exists, "Can not find the edited file Untitled.docx"
        self._device(text="Untitled.docx").click.wait()
        time.sleep(2)
        print "Excute:Select all"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.edit_btn.click.wait()
        time.sleep(1)
        self._locator.select_all_btn.click.wait()
        time.sleep(2)
        self.get_docx_screenshot("before.png")
        self._locator.return_btn.click.wait()
        time.sleep(2)
        self.check_ad()
        self._locator.back_btn.click.wait()
        time.sleep(1)
        if self._locator.back_btn.exists:
            self._locator.back_btn.click.wait()
            time.sleep(1)
        if not self._device(text="template.docx").exists:
            self._device(scrollable=True).scroll.to(text="template.docx")
        time.sleep(1)
        assert self._device(text="template.docx").exists, "can not found template.docx"
        self._device(text="template.docx").click.wait()
        time.sleep(2)
        print "Excute:Select all"
        self._locator.more_btn.click.wait()
        time.sleep(1)
        self._locator.edit_btn.click.wait()
        time.sleep(1)
        self._locator.select_all_btn.click.wait()
        time.sleep(2)
        self.get_docx_screenshot("after.png")
        time.sleep(2)
        rms = compare_pic.compare_pic(self.tmpdir + "/before.png", self.tmpdir + "/after.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms <= 5, "Add front fail,rms=%s" % rms

    def get_docx_screenshot(self, picname):
        """ check cocx which template docx by screenshot in DocsToGo.
        """
        print "catch screenshot %s" % picname
        bounds = self._device(
            resourceId="com.dataviz.docstogo:id/wtg_main_field_id").bounds
        path = get_screenshot_region(bounds)
        print path
        cmd = ('mv %s ' + self.tmpdir + '/' + picname) % path
        os.system(cmd)

    def check_ad(self):
        """ check if AD dialog popup,and close it.
        """
        _, activity = self.systemui.get_current_focus()
        time.sleep(2)
        if activity == "com.google.android.gms.ads.AdActivity" or activity == "com.amazon.device.ads.AdActivity":
            if self._device(description="nativeCloseButtonImage").exists:
                print "static ads"
                self._device(description="nativeCloseButtonImage").click.wait()
            elif self._device(description="Skip Ad ").exists:
                print "video ads"
                self._device(description="Skip Ad ").click.wait()
            else:
                print "other ads"
                self._device.press.back()
            time.sleep(3)

    def uninstall_app(self):
        """ uninstall DocsToGo
        """
        print "uninstall DocsToGo"
        cmd = 'uninstall %s' % DocsToGo.pkg_name
        g_common_obj.adb_cmd_common(cmd)
