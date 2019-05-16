# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for delete picture Operation
@since: 01/22/2015
@author: Yingjun Jin
'''

import os
import time
from testlib.util.common import g_common_obj

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
    def delete(self):
        return self._device(
            resourceId="com.google.android.apps.plus:id/delete")

    @property
    def delete_option(self):
        return self._device(
            resourceId="com.google.android.apps.plus:id/trash_can_button")

    @property
    def first_picture(self):
        return self._device(resourceId=\
            "com.google.android.apps.plus:id/tile_row")\
        .child(className="android.view.View")

    @property
    def first_folder(self):
        return self._device(
            resourceId="com.google.android.apps.plus:id/collection_title")

    @property
    def open_menu(self):
        return self._device(resourceId=\
            "com.google.android.apps.plus:id/action_bar")\
        .child(className="android.widget.ImageButton")

    @property
    def click_ondevice(self):
        return self._device(resourceId=\
            "com.google.android.apps.plus:id/navigation_item_list")\
        .child(className="android.widget.LinearLayout", index=2)

    @property
    def menu_key(self):
        return self._device(resourceId=\
            "com.google.android.apps.plus:id/action_bar")\
        .child(className="android.widget.ImageView")

    @property
    def select_btn(self):
        return self._device(className=\
            "android.widget.ListView")\
        .child(className="android.widget.LinearLayout", index=0)

    @property
    def delete_photos(self):
        return self._device(resourceId="com.google.android.apps.plus:id/trash_can_button")

class ImageDelete:
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
            ImageDelete.pkg_name, ImageDelete.activity_name)
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
        g_common_obj.stop_app_am(ImageDelete.pkg_name)

    def select_picture(self):
        """ Open picture
        """
        self._locator.first_picture.click()
        time.sleep(2)

    def delete_picture(self):
        """delete the selected picture
        """
        self._locator.delete.click()
        time.sleep(2)

    def check_delete_picture(self):
        """check the picture whether be delete
        """
        assert not self._device(resourceId="com.google.android.apps.photos:id/list_photo_tile_view").exists, "The picture not deleted"
        time.sleep(2)

    def check_delete_folder(self):
        """check the folder whether be delete
        """
        assert not self._locator.first_folder.exists, "The folder not deleted"
        time.sleep(2)

    def check_picture_thumbnailmode(self):
        """check the picture exitst in thumbnailmode
        """
        print "Check picture in thumbnailmode"
        assert self._device(resourceId="com.google.android.apps.photos:id/list_photo_tile_view").exists \
        or self._device(resourceId="com.google.android.apps.photos:id/collection_title")\
            .down(descriptionContains="Photo"), "The picture not exists in thumbnailmode"
        time.sleep(2)

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def long_press_picture(self):
        """ long press the picture
        """
        self._locator.first_picture.long_click()
        time.sleep(2)

    def delete_option(self):
        """ delete picture by option
        """
        self._locator.delete_option.click()
        time.sleep(2)

    def select_picture_by_option(self):
        """ select picture by press option
        """
        self._device(descriptionMatches="More options").click()
        time.sleep(2)
        self._device(resourceId="com.google.android.apps.plus:id/title").click()
        time.sleep(2)
        self._locator.first_picture.click()
        time.sleep(2)

    def choose_folder(self):
        """ choose folder ondevice
        """
        self._locator.open_menu.click()
        time.sleep(2)
        self._locator.click_ondevice.click()
        time.sleep(2)

    def menu_select_and_delete(self):
        """ select picture by menu select button and delete it
        """
        self._locator.menu_key.click()
        time.sleep(2)
        self._locator.select_btn.click()
        time.sleep(2)
        self._locator.first_picture.click()
        time.sleep(2)
        self._locator.delete_photos.click()
        time.sleep(2)
