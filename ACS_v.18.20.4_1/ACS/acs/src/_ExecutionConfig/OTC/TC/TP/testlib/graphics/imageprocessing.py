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
@summary: Class for ImageProcessing APK UI Operation
@since: 01/15/2015
@author: Yingjun Jin
'''

import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import get_screenshot_region
from testlib.graphics.compare_pic_impl import compare_pic

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
            return uiobj.wait("exists", timeout * 300)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    @property
    def wait_gone(self):
        """ wait until the ui object gone """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("gone", timeout * 1000)
        return _wait

    @property
    def choose_mode(self):
        return self._device(resourceId="com.android.rs.image:id/filterselection")

class ImageProcessingTest:
    '''
    classdocs
    '''
    apk_name = "ImageProcessing.apk"
    pkg_name = "com.android.rs.image"
    activity_name = "com.android.rs.image.ImageProcessingActivity"

    def __init__(self, cfg={}):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.cfg = cfg

    def launch_app_am(self):
        """ Launch ImageProcessing via adb am command
        """
        print "Launch ImageProcessing by adb am"
        g_common_obj.launch_app_am(\
            ImageProcessingTest.pkg_name, ImageProcessingTest.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def uninstall_app():
        """ uninstall the ImageProcessing
        """
        print "Uninstall the ImageProcessing"
        cmd = 'uninstall %s' % ImageProcessingTest.pkg_name
        g_common_obj.adb_cmd_common(cmd)

    @staticmethod
    def stop_app_am():
        """ Stop ImageProcessing via adb am command
        """
        print "Stop ImageProcessing by adb am"
        g_common_obj.stop_app_am(ImageProcessingTest.pkg_name)

    def scroll_to_bottom(self):
        """ scroll to the bottom of the screen
        """
        self._device().scroll.to(text="Benchmark All")

    def slider_saturation(self):
        """ swipe the slider
        """
        loop = int(self.cfg.get("loop"))
        for _ in range(0, loop):
            self._device(resourceId=\
            "com.android.rs.image:id/slider1").swipe.right(steps=50)
            time.sleep(1)
            self._device(resourceId=\
            "com.android.rs.image:id/slider1").swipe.left(steps=20)
            time.sleep(2)

    def slider_inblack(self):
        """ swipe the slider
        """
        loop = int(self.cfg.get("loop"))
        for _ in range(0, loop):
            self._device(resourceId=\
            "com.android.rs.image:id/slider2").swipe.right(steps=30)
            time.sleep(1)
            self._device(resourceId=\
            "com.android.rs.image:id/slider2").swipe.left(steps=15)
            time.sleep(2)

    def slider_outblack(self):
        """ swipe the slider
        """
        loop = int(self.cfg.get("loop"))
        for _ in range(0, loop):
            self._device(resourceId=\
            "com.android.rs.image:id/slider3").swipe.right(steps=20)
            time.sleep(1)
            self._device(resourceId=\
            "com.android.rs.image:id/slider3").swipe.left(steps=25)
            time.sleep(2)

    def slider_outwhite1(self):
        """ swipe the slider
        """
        loop = int(self.cfg.get("loop"))
        for _ in range(0, loop):
            self._device(resourceId=\
            "com.android.rs.image:id/slider4").swipe.right(steps=30)
            time.sleep(1)
            self._device(resourceId=\
            "com.android.rs.image:id/slider4").swipe.left(steps=15)
            time.sleep(2)

    def slider_outwhite2(self):
        """ swipe the slider
        """
        loop = int(self.cfg.get("loop"))
        for _ in range(0, loop):
            self._device(resourceId=\
            "com.android.rs.image:id/slider5").swipe.right(steps=50)
            time.sleep(1)
            self._device(resourceId=\
            "com.android.rs.image:id/slider5").swipe.left(steps=10)
            time.sleep(2)

    def sleep_wakeup(self):
        """ sleep and resume
        """
        self._device.screen.off()
        time.sleep(2)
        self._device.screen.on()
        time.sleep(2)

    def slide_all_max(self):
        """ slide all slider to max
        """
        self._device(resourceId=\
            "com.android.rs.image:id/slider1").swipe.right(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider2").swipe.right(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider3").swipe.right(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider4").swipe.right(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider5").swipe.right(steps=1)
        time.sleep(1)

    def slide_all_min(self):
        """slide all slider to min
        """
        self._device(resourceId=\
            "com.android.rs.image:id/slider1").swipe.left(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider2").swipe.left(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider3").swipe.left(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider4").swipe.left(steps=1)
        time.sleep(1)
        self._device(resourceId=\
            "com.android.rs.image:id/slider5").swipe.left(steps=1)
        time.sleep(1)

    def launchexit_20times(self):
        """ lanch and exit app 20times
        """
        for loop in range(1, 21):
            self.launch_app_am()
            self.stop_app_am()
            print "[loop times = %s]" % loop

    def switchtophotos_switchback(self):
        """ Swith to photos then swith back
        """
        self._device.press.home()
        time.sleep(2)
        g_common_obj.launch_app_am("com.google.android.apps.photos", \
            "com.google.android.apps.photos.home.HomeActivity")
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        time.sleep(3)
        # self._device.press.recent()
        # time.sleep(2)
        # self._device(text="Image Processing").click()
        self.launch_app_am()
        time.sleep(2)

    def choose_mode(self, mode):
        """ Choose a mode in Imageprocessing apk
        """
        y = self._device.info["displayHeight"]
        x = self._device.info["displayWidth"]
        if self._locator.choose_mode.exists:
            self._locator.choose_mode.drag.to(x / 2, y / 2, steps=1)
            time.sleep(1)
            if self._locator.choose_mode.exists:
                self._locator.choose_mode.click.wait()
        else:
            self._device(scrollable=True).scroll.to(resourceId="com.android.rs.image:id/filterselection")
            self._locator.choose_mode.click.wait()
        time.sleep(2)
        if self._device(text=mode).exists:
            self._device(text=mode).click.wait()
        else:
            self._device(scrollable=True).scroll.to(text=mode)
            assert self._device(text=mode).exists, "Can not find the mode: %s" % mode
            self._device(text=mode).click.wait()
        time.sleep(2)
        if self._device(className="android.widget.ListView").exists:
            self._device.press.back()

    def check_picture(self, mode):
        """ Check picture effect
        """
        g_common_obj.assert_exp_happens()
        for _ in range(0, 3):
            if self._device(resourceId="com.android.rs.image:id/display").exists:
                break
            else:
                self.launch_app_am()
                time.sleep(3)
        print "catch before screenshot:"
        assert self._device(resourceId="com.android.rs.image:id/display").exists, "can not find the picture widget"
        bounds_template = self._device(
            resourceId="com.android.rs.image:id/display").bounds
        self.path_before = get_screenshot_region(bounds_template)
        print self.path_before
        self.choose_mode(mode)
        g_common_obj.assert_exp_happens()
        self._device(scrollable=True).scroll.vert.toEnd()
        time.sleep(2)
        self.add_effect()
        g_common_obj.assert_exp_happens()
        self._device(scrollable=True).scroll.vert.backward(steps=5)
        time.sleep(1)
        print "catch after screenshot:"
        assert self._device(resourceId="com.android.rs.image:id/display").exists, "can not find the picture widget"
        bounds_template = self._device(
            resourceId="com.android.rs.image:id/display").bounds
        self.path_after = get_screenshot_region(bounds_template)
        print self.path_after
        rms = compare_pic.compare_pic(self.path_before, self.path_after)
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        if mode == "Copy":
            assert rms == 0, "The screenshot is not same with before change in Imageprocessing"
        else:
            assert rms > 0, "The screenshot is same with before change in Imageprocessing"

    def add_effect(self):
        """ Add effect if bar exist in Imageprocessing apk
        """
        if self._device(resourceId="com.android.rs.image:id/slider1").exists:
            self._device(resourceId=\
                         "com.android.rs.image:id/slider1").swipe.right(steps=1)
        time.sleep(2)
        if self._device(resourceId="com.android.rs.image:id/slider2").exists:
            self._device(resourceId=\
                         "com.android.rs.image:id/slider2").swipe.right(steps=1)
        time.sleep(2)
        if self._device(resourceId="com.android.rs.image:id/slider3").exists:
            self._device(resourceId=\
                         "com.android.rs.image:id/slider3").swipe.right(steps=1)
        time.sleep(2)
        if self._device(resourceId="com.android.rs.image:id/slider4").exists:
            self._device(resourceId=\
                         "com.android.rs.image:id/slider4").swipe.right(steps=1)
        time.sleep(2)
        if self._device(resourceId="com.android.rs.image:id/slider5").exists:
            self._device(resourceId=\
                         "com.android.rs.image:id/slider5").swipe.right(steps=1)
        time.sleep(2)

    def change_to_vertical(self):
        """ change device to vertical
        """
        g_common_obj.set_vertical_screen()

    def remove_file(self):
        """
        remove the screenshot file
        """
        try:
            os.system("rm -rf %s" % (self.path_before))
            os.system("rm -rf %s" % (self.path_after))
        except IOError:
            print "file not exists"
        except:
            print "Can not found screenshot file"
