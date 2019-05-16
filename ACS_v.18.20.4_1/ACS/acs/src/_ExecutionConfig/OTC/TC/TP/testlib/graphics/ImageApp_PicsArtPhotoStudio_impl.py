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
@summary: Class for ImageApp PicsArtPhotoStudio
@since: 05/13/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj
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
    def edit(self):
        return self._device(
            resourceId="com.picsart.studio:id/fab")

    @property
    def choose_pic(self):
        return self._device(
            resourceId="com.picsart.studio:id/recentImageId")

    @property
    def tool_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/btn_tool")

    @property
    def crop_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/tool_crop")

    @property
    def ruler_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/ruler")

    @property
    def crop1x1_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/btn_crop_1x1")

    @property
    def done_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/btn_done")

    @property
    def save_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/menu_item_save")

    @property
    def ok_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/save_export_button_ok")

    @property
    def check_file(self):
        return self._device(resourceId=\
            "com.picsart.studio:id/recent_photos_bottom_sheet")\
            .child(className="android.widget.FrameLayout", index=1)

    @property
    def check_file2(self):
        return self._device(resourceId=\
            "com.picsart.studio:id/recent_photos")\
            .child(className="android.widget.FrameLayout", index=1)

class PicsArtStudio:
    '''
    classdocs
    '''

    pkg_name = "com.picsart.studio"
    activity_name = "com.socialin.android.photo.picsinphoto.MainPagerActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.a = Allow_Permission()

    def launch_app_am(self):
        """ Launch PicsArtStudio via adb am command
        """
        print "Launch PicsArtStudio by adb am"
        g_common_obj.launch_app_am(\
            PicsArtStudio.pkg_name, PicsArtStudio.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop PicsArtStudio via adb am command
        """
        print "Stop PicsArtStudio by adb am"
        g_common_obj.stop_app_am(PicsArtStudio.pkg_name)

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
        self._locator.edit.click()
        time.sleep(2)
        self._locator.choose_pic.click()
        time.sleep(2)

    def edit_pic(self):
        """ edit picture
        """
        self._locator.tool_btn.click()
        time.sleep(2)
        self._locator.crop_btn.click()
        time.sleep(2)
        Width = self._device.info.get("displayWidth")
        Height = self._device.info.get("displayHeight")
        self._device.swipe(Width * 6 / 7, Height / 2, Width * 6 / 7, Height * 9 / 10)
        time.sleep(2)
        self._locator.crop1x1_btn.click()
        time.sleep(2)

    def save_pic(self):
        """ save edit picture
        """
        self._locator.done_btn.click()
        time.sleep(2)
        self._locator.save_btn.click()
        time.sleep(2)
        self.a.permission_allower()
        time.sleep(2)
        self._device.press("back")
        self._locator.ok_btn.click()

        time.sleep(2)

    def check_pic(self):
        """ check edit picture
        """
        self._device.press("back")
        time.sleep(4)
        if not self._locator.check_file.exists:
            self._device(text="Edit").click.wait()
            assert self._locator.check_file.exists, "The edit picture not exists"
            time.sleep(2)

    def uninstall_app(self):
        """ uninstall picsartstudio
        """
        print "uninstall picsartstudio"
        cmd = 'uninstall %s' % PicsArtStudio.pkg_name
        g_common_obj.adb_cmd_common(cmd)
