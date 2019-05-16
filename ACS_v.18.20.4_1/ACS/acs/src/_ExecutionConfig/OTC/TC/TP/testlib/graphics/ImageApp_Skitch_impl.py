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
@summary: Class for ImageApp Skitch
@since: 05/29/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.compare_pic_impl import compare_pic

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def choose_pic(self):
        return self._device(resourceId=\
            "com.evernote.skitch:id/gridView")\
            .child(className="android.widget.ImageView")

    @property
    def options(self):
        return self._device(description="More options")

    @property
    def rotate_right(self):
        return self._device(className=\
            "android.widget.ListView")\
            .child(className="android.widget.LinearLayout", index=2)

    @property
    def done(self):
        return self._device(
            resourceId="com.evernote.skitch:id/save_left")

    @property
    def save_pic(self):
        return self._device(resourceId=\
            "android:id/list")\
            .child(className="android.widget.LinearLayout", index=1)

class Skitch:
    '''
    classdocs
    '''

    pkg_name = "com.evernote.skitch"
    activity_name = "com.evernote.skitch.app.SwipeNavActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch Skitch via adb am command
        """
        print "Launch Skitch by adb am"
        g_common_obj.launch_app_am(\
            Skitch.pkg_name, Skitch.activity_name)
        time.sleep(2)
        if self._device(text="Let's get started!").exists:
            self._device(text="Let's get started!").click()
        time.sleep(2)
        if self._device(text="PHOTOS").exists:
            self._device(text="PHOTOS").click()
        time.sleep(1)

    @staticmethod
    def stop_app_am():
        """ Stop Skitch via adb am command
        """
        print "Stop Skitch by adb am"
        g_common_obj.stop_app_am(Skitch.pkg_name)

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
        #self._locator.choose_pic.click()
        self._device.click(192, 226)
        time.sleep(2)
        y = self._device.info["displayHeight"]
        x = self._device.info["displayWidth"]
        self._device.click(x / 2, y / 2)
        time.sleep(2)

    def edit_pic(self):
        """ edit picture
        """
        print "catch screenshot before_effect.png"
        self._device.screenshot("before.png")
        time.sleep(2)
        self._locator.options.click()
        time.sleep(2)
        self._locator.rotate_right.click()
        time.sleep(2)
        print "catch screenshot after_effect.png"
        self._device.screenshot("after.png")
        time.sleep(1)

    def save_pic(self):
        """ save edit picture
        """
        self._locator.done.click()
        time.sleep(2)
        self._locator.save_pic.click()
        time.sleep(2)

    def check_pic(self):
        """ check edit picture effect via compare 2 screenshot
        """
        path = os.getcwd()
        print "current path is " + path
        time.sleep(1)
        rms = compare_pic.compare_pic(path + "/before.png", path + "/after.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms >= 20, "The screenshot is same with before add effect"

    def uninstall_app(self):
        """ uninstall Skitch
        """
        print "uninstall Skitch"
        cmd = 'uninstall %s' % Skitch.pkg_name
        g_common_obj.adb_cmd_common(cmd)
