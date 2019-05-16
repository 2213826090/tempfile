# Copyright (C) 2014  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for AndroidBasicRenderScript APK UI Operation
@since: 01/14/2015
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
            return uiobj.wait("exists", timeout * 1000)
        return _wait

    @property
    def wait_gone(self):
        """ wait until the ui object gone """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("gone", timeout * 1000)
        return _wait

class BasicRenderScript:
    '''
    classdocs
    '''
    apk_name = "AndroidBasicRenderScript.apk"
    pkg_name = "com.intel.sample.androidbasicrs"
    activity_name = "com.intel.sample.androidbasicrs.MainActivity"

    def __init__(self, cfg={}):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.cfg = cfg

    def launch_app_am(self):
        """ Launch AndroidBasicRenderScript via adb am command
        """
        print "Launch AndroidBasicRenderScript by adb am"
        g_common_obj.launch_app_am(\
            BasicRenderScript.pkg_name, BasicRenderScript.activity_name)
        time.sleep(5)

    @staticmethod
    def stop_app_am():
        """ Stop BasicRenderScrpit via adb am command
        """
        print "Stop BasicRenderScrpit by adb am"
        g_common_obj.stop_app_am(BasicRenderScript.pkg_name)

    def press_render_screen_center(self):
        """ Press the center of the screen
        """
        loop = int(self.cfg.get("loop"))
        display = self._device.info
        display_width = display[u'displayWidth'] / 2
        display_height = display[u'displayHeight'] / 2
        for _ in range(0, loop):
            self._device.click(display_width, display_height)
            time.sleep(2)

    def change_render_via_take_photo(self):
        """ Take a photo as the background.
        """
        click_x = self._device.info.get("displayWidth") * 1 / 100
        click_y = self._device.info.get("displayHeight") * 99 / 100
        self._device.click(click_x, click_y)
        time.sleep(5)
        self._device(resourceId="com.android.camera2:id/shutter_button").click()
        time.sleep(3)
        self._device(resourceId="com.android.camera2:id/done_button").click()
        time.sleep(3)

