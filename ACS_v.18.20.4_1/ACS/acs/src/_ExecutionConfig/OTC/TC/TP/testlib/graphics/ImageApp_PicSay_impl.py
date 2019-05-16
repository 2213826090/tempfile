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
@since: 06/09/2015
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
    def get_pic(self):
        return self._device(resourceId="com.shinycore.picsayfree:id/choosePicture")

    @property
    def choose_pic(self):
        return self._device(resourceId=\
            "com.android.documentsui:id/grid")\
            .child(className="android.widget.FrameLayout")

    @property
    def images_btn(self):
        return self._device(text="Images")

    @property
    def choose_size(self):
        return self._device(resourceId=\
            "android:id/select_dialog_listview")\
            .child(className="android.widget.RelativeLayout", index=1)

    @property
    def menu_bar(self):
        return self._device(
            resourceId="com.shinycore.picsayfree:id/menuBar")

    @property
    def distort_btn(self):
        return self._device(
            resourceId="android:id/list")\
            .child(className="android.widget.LinearLayout", index=0)

    @property
    def save_pic_bar(self):
        return self._device(resourceId=\
            "com.shinycore.picsayfree:id/main")\
            .child(className="android.widget.LinearLayout")

class PicSay:
    '''
    classdocs
    '''

    pkg_name = "com.shinycore.picsayfree"
    activity_name = "com.shinycore.picsayfree.PicSay"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch PicSay via adb am command
        """
        print "Launch PicSay by adb am"
        g_common_obj.launch_app_am(\
            PicSay.pkg_name, PicSay.activity_name)
        time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop PicSay via adb am command
        """
        print "Stop PicSay by adb am"
        g_common_obj.stop_app_am(PicSay.pkg_name)

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
        self._locator.get_pic.click()
        time.sleep(2)
        if self._device(description="Show roots").exists:
            self._device(description="Show roots").click()
            time.sleep(2)
        self._locator.images_btn.click()
        time.sleep(2)
        self._locator.choose_pic.click()
        time.sleep(3)
        if self._locator.choose_pic.exists:
            self._locator.choose_pic.click()
            time.sleep(3)
        if self._locator.choose_size.exists:
            self._locator.choose_size.click()

    def edit_pic(self):
        """ edit picture
        """
        print "catch screenshot before_effect.png"
        self._device.screenshot("before.png")
        time.sleep(2)
        density = g_common_obj.adb_cmd_capture_msg('wm density')
        print "density is: " + density
        def density1():
            print "density1"
            self.left = 180
            self.down = 7
            self.step = 50
            print "after desity1: %s,%s,%s " % (self.left, self.down, self.step)
        def density2():
            print "density2"
            self.left = 250
            self.down = 21
            self.step = 50
            print "after desity2: %s,%s,%s " % (self.left, self.down, self.step)
        def density3():
            print "density3"
            self.left = 350
            self.down = 25
            self.step = 100
            print "after desity3: %s,%s,%s " % (self.left, self.down, self.step)
        def density4():
            print "density4"
            self.left = 280
            self.down = 37
            self.step = 100
            print "after desity4: %s,%s,%s " % (self.left, self.down, self.step)
        def density5():
            print "density5"
            self.left = 370
            self.down = 62
            self.step = 100
            print "after desity5: %s,%s,%s " % (self.left, self.down, self.step)
        def density6():
            print "density6"
            self.left = 450
            self.down = 67
            self.step = 150
            print "after desity6: %s,%s,%s " % (self.left, self.down, self.step)
        def density7():
            print "density7"
            self.left = 550
            self.down = 87
            self.step = 150
            print "after desity7: %s,%s,%s " % (self.left, self.down, self.step)
        def density8():
            print "density8"
            self.left = 650
            self.down = 105
            self.step = 200
            print "after desity8: %s,%s,%s " % (self.left, self.down, self.step)
        def density9():
            print "density9"
            self.left = 750
            self.down = 112
            self.step = 200
            print "after desity9: %s,%s,%s " % (self.left, self.down, self.step)
        def _default_density():
            print "_default_density"
            self.left = 350
            self.down = 25
            self.step = 100
        values = {
                  "Physicaldensity120": density1,
                  "Physicaldensity160": density2,
                  "Physicaldensity213": density3,
                  "Physicaldensity240": density4,
                  "Physicaldensity320": density5,
                  "Physicaldensity400": density6,
                  "Physicaldensity480": density7,
                  "Physicaldensity560": density8,
                  "Physicaldensity640": density9
                  }
        print "density:", density
        density = density.replace(' ', '')
        density = density.replace(':', '')
        density_str = str(density)
        print "values.get(%s)" % density_str
        values.get(density_str, _default_density)()
        right = self._locator.menu_bar.info.get("bounds").get("right")
        top = self._locator.menu_bar.info.get("bounds").get("top")
        print "bounds right: %s,bounds top: %s" % (right, top)
        print "right: %s, left: %s" % (right, self.left)
        x = right - self.left
        y = top + self.down
        print "click(%s,%s) " % (x, y)
        self._device.click(x, y)
        time.sleep(2)
        if not self._device(text="Add Effect").exists:
            print "Add Effect not exists"
            x = right
            while True:
                self._device.click(x, y)
                time.sleep(2)
                if not self._device(text="Add Effect").exists:
                    x = x - self.step
                    if self._device(resourceId="com.shinycore.picsayfree:id/cancel").exists:
                        self._device(resourceId="com.shinycore.picsayfree:id/cancel").click()
                    continue
                if self._device(text="Add Effect").exists:
                    break
        if self._device(text="Add Effect").exists:
            self._locator.distort_btn.click()
            time.sleep(2)
        hight = self._locator.save_pic_bar.info.get("bounds").get("top") + self.down
        self._device.click(20, hight)
        time.sleep(2)
        print "catch screenshot after_effect.png"
        self._device.screenshot("after.png")
        time.sleep(1)

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
        """ uninstall PicSay
        """
        print "uninstall PicSay"
        cmd = 'uninstall %s' % PicSay.pkg_name
        g_common_obj.adb_cmd_common(cmd)
