# Copyright (C) 2015  Zhang,RongX Z <rongx.z.zhang@intel.com>
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
@summary: Class for ImageApp PhotoGrid
@since: 06/03/2015
@author: Zhang RongX,Z
'''
import time
from testlib.util.common import g_common_obj

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def photo_edit(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/single_text")

    @property
    def pick_photo(self):
        return self._device(
            text="Pick a photo to try")

    @property
    def cancle_pick_photo(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/whatsnew_close")

    @property
    def choose_pic(self):
        return self._device(
            resourceIdMatches="com.roidapp.photogrid:id/(image_selector_message|grid_image)")

    @property
    def recent_btn(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/newbie_single_image")

    def newbie_single_text(self):
        return self._device(
            text="Tap to open folder")

    @property
    def crop_btn(self):
        return self._device(
            text="Crop")

    def choose_crop_scale(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/roidapp_imagelib_crop_btn3to5")

    @property
    def save_btn(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/saveBtn")

    def confirm_save_btn(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/savedialog_mainsave")

    @property
    def confirm_btn(self):
        return self._device(
            resourceId="com.roidapp.photogrid:id/confirmBtn")

    @property
    def select_pic(self):
        return self._device(
            resourceIdMatches="com.android.documentsui:id/(icon_mime|date)")

    @property
    def add_image_btn(self):
        return self._device(text="Image")


class PhotoGridEdit:
    '''
    PhotoGrid
    '''

    pkg_name = "com.roidapp.photogrid"
    activity_name = "com.roidapp.photogrid.MainPage"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch PhotoGrid via adb am command
        """
        print "Launch PhotoGrid by adb am"
        g_common_obj.launch_app_am(\
            PhotoGridEdit.pkg_name, PhotoGridEdit.activity_name)
        time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop PhotoGrid via adb am command
        """
        print "Stop PhotoGrid by adb am"
        g_common_obj.stop_app_am(PhotoGridEdit.pkg_name)

    @staticmethod
    def fresh_sdcard():
        """ fresh sdcard to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def choose_pic(self):
        """ choose picture to edit
        """
        time.sleep(6)
        if self._locator.pick_photo.exists:
            self._locator.cancle_pick_photo.click.wait()
        self._device.press("recent")
        time.sleep(2)
        self._device.press("back")
        time.sleep(2)
        if not self._locator.photo_edit.exists:
            self._device(text="Grid").click.wait()
        else:
            self._locator.photo_edit.click.wait()
        time.sleep(2)
        if self._device(text="Tap to open folder").exists:
            self._locator.recent_btn.click.wait()
        time.sleep(3)
        self._device.press("recent")
        time.sleep(2)
        self._device.press("back")
        time.sleep(2)
        if not self._device(text="Cloud Album").exists:
            self._device(resourceId="com.roidapp.photogrid:id/selector_model_text").click.wait()
        self._device.press("recent")
        time.sleep(2)
        self._device.press("back")
        time.sleep(2)
        self._device(text="Pictures").click.wait()
        time.sleep(1)
        self._device.press("recent")
        time.sleep(2)
        self._device.press("back")
        time.sleep(2)
        self._locator.choose_pic.click.wait()
        time.sleep(3)
        if self._device(text='Next').exists: self._device(text='Next').click()
        time.sleep(3)

    def edit_pic(self):
        """ edit picture
        """
        self._locator.crop_btn.click.wait()
        time.sleep(4)
        self._device(resourceId="com.roidapp.photogrid:id/roidapp_imagelib_crop_btn2to3").click.wait()
        time.sleep(5)

    def save_pic(self):
        """ save edit picture
        """
        if self._locator.confirm_btn.exists:
            self._locator.confirm_btn.click.wait()
        time.sleep(12)
        self._locator.save_btn.click.wait()
        time.sleep(4)
        self._device(resourceId="com.roidapp.photogrid:id/savedialog_mainsave").click.wait()
        time.sleep(5)
        for _ in range(3):
            if self._device(textContains="Saving Complete").exists:
                break
            time.sleep(5)
        assert self._device(text="Saving Complete").exists, "pictures saved failed"

    def uninstall_app(self):
        """ uninstall PhotoGrid
        """
        print "uninstall PhotoGrid"
        cmd = 'uninstall %s' % PhotoGridEdit.pkg_name
        g_common_obj.adb_cmd_common(cmd)

    def function_add_image(self):
        """ Add one more image for mirror effect.
        """
        if not self._locator.add_image_btn.exists:
            self._device(
                resourceId="com.roidapp.photogrid:id/bottom_single_scrollview").scroll.horiz.toEnd()
        self._locator.add_image_btn.click.wait()

        try:
            if self._device(description='Show roots'):
                self._device(description='Show roots').click()
                time.sleep(2)
            if self._device(text="Photos"):
                self._device(text="Photos").click()
                time.sleep(2)
            if self._device(text="Pictures"):
                self._device(text="Pictures").click()
                time.sleep(2)
            if self._device(descriptionStartsWith="Photo "):
                self._device(descriptionStartsWith="Photo ").click()
                time.sleep(2)
            if self._device(resourceId="android:id/title", text="Images"):
                self._device(resourceId="android:id/title", text="Images").click.wait()
                time.sleep(2)
                self._locator.select_pic.click.wait()
                time.sleep(2)
                self._locator.select_pic.click.wait()
        except:
            raise Exception("Fail to add image!")
        time.sleep(2)
