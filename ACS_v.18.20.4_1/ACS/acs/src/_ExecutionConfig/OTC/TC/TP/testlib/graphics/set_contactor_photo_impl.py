# -*- coding: utf-8 -*-
# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
# Intel Corporation All Rights Reserved.
from pydoc import classname

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
@summary: Class for set contact photo Operation
@since: 05/11/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.common import Allow_Permission


class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=10):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    @property
    def create_contactor(self):
        return self._device(
            resourceId="com.android.contacts:id/floating_action_button")

    @property
    def input_name(self):
        return self._device(textMatches="Name|First name")

    @property
    def save_name(self):
        return self._device(resourceId="android:id/action_bar")\
            .child(className="android.widget.ImageButton")

    @property
    def delete_account(self):
        return self._device(resourceId="com.android.contacts:id/menu_delete")

    @property
    def verify_delete(self):
        return self._device(resourceId="android:id/button1")

    @property
    def first_picture(self):
        return self._device(resourceId="com.google.android.apps.plus:id/tile_row")\
            .child(className="android.view.View")

    @property
    def open_picture_menu(self):
        return self._device(resourceId="com.google.android.apps.plus:id/action_bar")\
            .child(className="android.widget.ImageView")

    @property
    def choose_setas(self):
        return self._device(className="android.widget.ListView")\
            .child(text="Set as…")

    @property
    def choose_contactphoto(self):
        return self._device(resourceId="android:id/resolver_list")\
            .child_by_text("Contact photo", className="android.widget.TextView")

    @property
    def choose_done(self):
        return self._device(resourceId="android:id/action_bar")\
            .child(className="android.widget.TextView")

    @property
    def select_contactor(self):
        return self._device(resourceId="android:id/list")\
            .child(className="android.view.View")

    @property
    def reedit_contactor(self):
        return self._device(
            resourceId="com.android.contacts:id/menu_edit")

    @property
    def change_button(self):
        return self._device(
            resourceId="com.android.contacts:id/change_button")

    @property
    def removephoto_button(self):
        return self._device(className="android.widget.ListView")\
            .child(text="Remove photo")

    @property
    def delete_image(self):
        return self._device(resourceId="com.google.android.apps.plus:id/delete")


class SetAccount:

    '''
    classdocs
    '''

    pkg_name = "com.android.contacts"
    activity_name = "com.android.contacts.activities.PeopleActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.a = Allow_Permission()

    def launch_app_am(self):
        """ Launch Contacts via adb am command
        """
        print "Launch Contacts by adb am"
        g_common_obj.launch_app_am(\
            SetAccount.pkg_name, SetAccount.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_app_am():
        """ Stop Contacts via adb am command
        """
        print "Stop Contacts by adb am"
        g_common_obj.stop_app_am(SetAccount.pkg_name)

    def create_account(self, contactor_name):
        """create test contactor
        """
        time.sleep(5)
        if not self._device(textContains="All").exists:
            self._device.press.home()
            time.sleep(2)
            self.launch_app_am()
        self._locator.create_contactor.click()
        if self._device(resourceId="com.android.packageinstaller:id/permission_allow_button").exists:
            self._device(resourceId="com.android.packageinstaller:id/permission_allow_button").click()
        time.sleep(2)
        if self._device(text="OK").exists:
            self._device(text="OK").click()
            time.sleep(2)
        if self._device(textContains="Keep local").exists:
            self._device(textContains="Keep local").click()
            time.sleep(2)
        if self._device(text="KEEP LOCAL"):
            self._device(text="KEEP LOCAL").click.wait()
        if self._device(textMatches="Cancel|CANCEL").exists:
            self._device(textMatches="Cancel|CANCEL").click()
            time.sleep(2)
        self._locator.input_name.set_text(contactor_name)
        time.sleep(2)
        if self._device(resourceId="com.android.contacts:id/menu_save").exists:
            self._device(resourceId="com.android.contacts:id/menu_save").click.wait()
        elif self._device(textMatches="SAVE|Save").wait.exists():
            self._device(textMatches="SAVE|Save").click()
        else:
            self._locator.save_name.click()
        time.sleep(2)
        self.a.permission_allower()

    def check_setphoto(self, contactor_name, qr_string):
        """check the photo whether be set
        """
        if self._device().scroll.to(text=contactor_name) == False:
            print "can not find the contacts!"
            return
        if not self._device(text=contactor_name).exists:
            self._device(scrollable=True).scroll.to(text=contactor_name)
            time.sleep(2)
        self._device(text=contactor_name).click()
        time.sleep(2)
        assert QRcode().decode_image_qrcode(g_common_obj.get_device().screenshot(".tmp.png"))[1] == qr_string, "Contactor image setting failed!"
#         for _ in range(0, 3):
#             if self._device(text="ALLOW").exists:
#                 self._device(text="ALLOW").click.wait()
#         self._locator.reedit_contactor.click()
#         time.sleep(2)
#         if self._device(text="More Fields").exists:
#             self._device(text="More Fields").click.wait()
#         self._locator.change_button.click()
#         time.sleep(2)
#         assert self._locator.removephoto_button.exists, "The photo not set"
#         time.sleep(2)

    def delete_account(self, _account=None):
        """delete test contactor
        """
        print "Launch Contacts by adb am"
        g_common_obj.launch_app_am(\
            SetAccount.pkg_name, SetAccount.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(2)
        self._device(textContains="All").click()
        time.sleep(1)
        if self._device(scrollable=True).scroll.to(text=_account) == False:
            print "can not find the contacts!"
            return
        if not self._device(text=_account).exists:
            self._device(scrollable=True).scroll.to(text="Test")
            time.sleep(2)
        self._device(text=_account).click()
        time.sleep(2)
        self._device(className="android.widget.ImageButton").click()
        self._device(textContains="Delete").click.wait()
        self._device(resourceId="android:id/button1").click()
#         time.sleep(2)
#         self._locator.reedit_contactor.click()
#         time.sleep(2)
#         try:
#             self._locator.delete_account.click()
#             time.sleep(2)
#             self._locator.verify_delete.click()
#             time.sleep(2)
#         except:
#             self._device(description="More options").click.wait()
#             self._device(text="Delete").click.wait()
#             self._device(text="OK").click.wait()
#         print "Stop Contacts by adb am"
#         g_common_obj.stop_app_am(SetAccount.pkg_name)


class SetPhoto:

    '''
    classdocs
    '''

    pkg_name = "com.google.android.apps.plus"
    activity_name = "com.google.android.apps.photos.phone.PhotosHomeActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am(\
            SetPhoto.pkg_name, SetPhoto.activity_name)
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_app_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am(SetPhoto.pkg_name)

    def set_contactorphoto(self):
        """ Open picture
        """
        self._locator.first_picture.click()
        time.sleep(2)
        self._locator.open_picture_menu.click()
        time.sleep(2)
        self._locator.choose_setas.click()
        time.sleep(2)
        if self._device(text="More").exists:
            self._device(text="More").click()
            time.sleep(2)
        self._locator.choose_contactphoto.click()
        time.sleep(2)
        if not self._device(text="Test").exists:
            self._device(scrollable=True).scroll.to(text="Test")
            time.sleep(2)
        self._device(text="Test").click()
        time.sleep(2)
        self._locator.choose_done.click()
        time.sleep(2)

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)
