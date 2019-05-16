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
@summary: Class for check picture Operation
@since: 01/21/2015
@author: Yingjun Jin
'''

import os
import time
from testlib.graphics.md5_impl import make_md5
from testlib.util.common import g_common_obj
from testlib.graphics.common import get_screenshot_region
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.photos_impl import get_photo_implement
from testlib.common.base import getTmpDir


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


class ImageDetails:

    '''
    classdocs
    '''

    pkg_name = "com.google.android.apps.plus"
    activity_name = "com.google.android.apps.photos.phone.PhotosHomeActivity"

    def __init__(self):
        self.tmpdir = getTmpDir()
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.photos = get_photo_implement()

    def launch_app_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am(
            ImageDetails.pkg_name, ImageDetails.activity_name)
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
        g_common_obj.stop_app_am(ImageDetails.pkg_name)

    def open_picture_detail(self):
        """ Open pic details
        """
        self._device(resourceId="com.google.android.apps.plus:id/tile_row")\
            .child(className="android.view.View").click()
        time.sleep(2)
        self._device(descriptionMatches="More options").click()
        time.sleep(2)
        self._device(
            resourceId="com.google.android.apps.plus:id/title").click()
        time.sleep(2)

    def check_detail(self):
        """check the picture details
        """
        time.sleep(2)
        assert self._device(text="Details").exists, 'Not found the Details'

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    @staticmethod
    def delete_picture():
        """ delete the test picture
        """
        cmd = 'rm -rf /sdcard/Pictures/*'
        g_common_obj.adb_cmd(cmd)

    @staticmethod
    def clean_tmp():
        """ clean tmp folder
        """
        cmd = 'rm /tmp/rotate.png'
        os.system(cmd)
        cmd = 'rm /tmp/screenshot.png'
        os.system(cmd)
        cmd = 'rm /tmp/template.png'
        os.system(cmd)

    def open_photo_and_rotate_left_7times(self):
        """ open photo and rotate left 7times
        """
        self.photos.open_a_picture()
        time.sleep(1)
        self.photos.click_crop_tools()
        time.sleep(2)
        for _ in range(0, 7):
            self.photos.rotate_90_degrees()
            time.sleep(1)
        g_common_obj.assert_exp_happens()
        self._device.press.back()

    def open_photo_and_rotate_right_7times(self):
        """ open photo and rotate right 7times
        """
        self.photos.open_a_picture()
        time.sleep(1)
        self.photos.click_crop_tools()
        time.sleep(2)
        self.photos.rotate_right_45_degrees()
        for _ in range(0, 7):
            self.photos.rotate_90_degrees()
            time.sleep(1)
        g_common_obj.assert_exp_happens()
        self._device.press.back()

    def diff_pic_after_rotate(self):
        """ diff the pic in widget after rotate
        """
        self.photos.open_a_picture()
        self.photos.click_crop_tools()
        print "catch template.png"
        bounds_template = self._device(
            resourceId="com.google.android.apps.photos:id/cpe_crop_overlay").bounds
        path_template = get_screenshot_region(bounds_template)
        cmd = "mv %s /tmp/template.png" % path_template
        os.system(cmd)
        time.sleep(2)
        for _ in range(0, 4):
            self.photos.rotate_90_degrees()
            time.sleep(1)
        print "catch rotate.png"
        bounds = self._device(
            resourceId="com.google.android.apps.photos:id/cpe_crop_overlay").bounds
        path = get_screenshot_region(bounds)
        cmd = "mv %s /tmp/rotate.png" % path
        os.system(cmd)
        rms = compare_pic.compare_pic("/tmp/template.png", "/tmp/rotate.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms < 5, "The picture is not same with template"
        if self._device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept"):
            self._device(resourceId="com.google.android.apps.photos:id/cpe_strength_accept").click.wait()
        if self._device(resourceId="com.google.android.apps.photos:id/cpe_save_button"):
            self._device(resourceId="com.google.android.apps.photos:id/cpe_save_button").click.wait()

    def AddEffects_Screenshot_HomeScreen(self):
        """ Add Effects and take Screenshot then HomeScreen
        """
        self.photos.open_a_picture()
        time.sleep(1)
        print "catch screenshot before_effect.png"
        self._device.screenshot("before.png")
        time.sleep(1)
        cmd = 'mv before.png ' + self.tmpdir + '/before.png'
        os.system(cmd)
        self.photos.apply_filter_to_photos("filter Mars")
        time.sleep(1)
        print "catch screenshot after_effect.png"
        self._device.screenshot("after.png")
        time.sleep(1)
        cmd = 'mv after.png ' + self.tmpdir + '/after.png'
        os.system(cmd)
        time.sleep(1)
        rms = compare_pic.compare_pic(self.tmpdir + "/before.png", self.tmpdir + "/after.png")
        time.sleep(1)
        print "The Similarity value of 2 screenshot is %s" % rms
        assert rms >= 5, "The screenshot is same with before add effect"
        time.sleep(2)
        self._device.press.home()

    def AddEffects_check(self, md5_list):
        """ Add Effects and check effects
        """
        self.photos.open_a_picture()
        self.photos.apply_filter_to_photos("filter Deimos")
        time.sleep(2)
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.jpg | sed -n 2p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print "compare file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        md5 = make_md5.make_md5_jumpbyte(128, self.tmpdir + '/compare.jpg')
        assert self.Md5_list_check(md5, md5_list), "check add effect fail"

    def AddStraightenEffects_LessThan20kb_check(self, md5_list):
        """ Add Straighten Effects with picture LessThan20kb and check
        """
        self.photos.open_a_picture()
        self.photos.click_crop_tools()
        self.photos.rotate_90_degrees()
        time.sleep(2)
        self.photos.save_picture_after_rotation()
        time.sleep(2)
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.bmp | sed -n p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print compare
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        md5 = make_md5.make_md5_jumpbyte(128, self.tmpdir + '/compare.jpg')
        assert self.Md5_list_check(md5, md5_list), "check add effect fail"

    def BlueImage_AddVignetteEffect_check(self, md5_list):
        """ Add Vignette Effects to Blue image and check
        """
        self.photos.open_a_picture()
        time.sleep(3)
        self.photos.add_adjustments_to_photos("Apply vignette enhancements", 100)
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.jpg | sed -n 2p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        md5 = make_md5.make_md5_jumpbyte(176, self.tmpdir + '/compare.jpg')
        assert self.Md5_list_check(md5, md5_list), "check add effect fail"

    def BlueImage_AddVignetteEffect_check_(self, file_path):
        """ Add Vignette Effects to Blue image and check
        """
        self.photos.open_a_picture()
        time.sleep(3)
        self.photos.add_adjustments_to_photos("Apply vignette enhancements", 100)
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.jpg | sed -n 2p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        assert g_common_obj.shell_cmd("diff -q %s %s" % (file_path, self.tmpdir + '/compare.jpg')) != '0', "check add effect fail"

    def Select_Random_Mediaeffect_check(self, md5_list):
        """ Add Radom Media Effects to image and check
        """
        self.photos.open_a_picture()
        time.sleep(3)
        self.photos.add_adjustments_to_photos("Apply light enhancements", 100)
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.jpg | sed -n 2p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        md5 = make_md5.make_md5_jumpbyte(320, self.tmpdir + '/compare.jpg')
        assert self.Md5_list_check(md5, md5_list), "check add effect fail"

    def Select_Random_Mediaeffect_check_(self, file_path):
        """ Add Radom Media Effects to image and check
        """
        self.photos.open_a_picture()
        time.sleep(3)
        self.photos.add_adjustments_to_photos("Apply light enhancements", 100)
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls -l /sdcard/Pictures/*.jpg | sed -n 2p')
        compare = '/sdcard/Pictures/' + os.path.basename(comp.strip("\n").split(' ')[-1])
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        assert g_common_obj.shell_cmd("diff -q %s %s" % (file_path, self.tmpdir + '/compare.jpg')) != '0', "check add effect fail"

    def picture_fullscreen(self):
        """ open picture picture and fullscreen
        """
        self._device(resourceId="com.google.android.apps.plus:id/tile_row").child(
            className="android.view.View").click()
        time.sleep(3)
        Width = self._device.info.get("displayWidth")
        Height = self._device.info.get("displayHeight")
        self._device.click(Width / 2, Height / 2)
        time.sleep(3)
        assert not self._device(description="More options").exists, 'check full screen fail'
        g_common_obj.assert_exp_happens()

    def wbmp_fullscreen_rotate(self):
        """ open wbmp picture and fullscreen rotate
        """
        self.photos.open_a_picture()
        self.photos.view_a_picture_fullscreen()
        time.sleep(1)
        for _ in range(0, 3):
            self._device.orientation = "l"
            time.sleep(2)
            assert not self._device(description="More options").exists, 'check full screen fail'
            g_common_obj.assert_exp_happens()
            self._device.orientation = "n"
            time.sleep(2)
            assert not self._device(description="More options").exists, 'check full screen fail'
            g_common_obj.assert_exp_happens()
            self._device.orientation = "r"
            time.sleep(2)
            assert not self._device(description="More options").exists, 'check full screen fail'
            g_common_obj.assert_exp_happens()
        print "Reset orientation to vertical"
        width = self._device.info["displayWidth"]
        height = self._device.info["displayHeight"]
        orientation = self._device.info["displayRotation"]
        if width > height and orientation == 0:
            self._device.orientation = "r"
        elif width > height and orientation > 0:
            self._device.orientation = "n"
        self._device.freeze_rotation()

    def check_image_fullscreen_onebyone(self, num):
        """ open wbmp picture and fullscreen one by one
        """
        num = int(num)
        self.photos.open_a_picture()
        self.photos.view_a_picture_fullscreen()
        time.sleep(3)
        print "check image:1"
        assert not self._device(description="More options").exists, 'check full screen fail'
        g_common_obj.assert_exp_happens()
        for n in range(0, num):
            Width = self._device.info.get("displayWidth")
            Height = self._device.info.get("displayHeight")
            self._device.swipe(Width * 4 / 5, Height / 2, 0, Height / 2)
            time.sleep(2)
            print "check image:%s" % (n + 2)
            assert not self._device(description="More options").exists, 'check full screen fail'
            assert not self._device(text="Couldn't load photo").exists, "view 1k jpeg failed, and popup 'Couldn't load photo'"
            g_common_obj.assert_exp_happens()

    def imageView_sildeshow(self):
        """ open images and Sildeshow
        """
        self.photos.open_a_picture()
        self.photos.pictures_slideshow()
        time.sleep(2)
        for n in range(0, 5):
            time.sleep(4)
            print "check image:%s" % (n + 2)
            g_common_obj.assert_exp_happens()
            assert not self._device(description="More options").exists, 'check full screen fail'

    def Md5_list_check(self, md5, md5_list):
        """ Check md5 value in list
        """
        md5_list = list(md5_list)
        print md5_list
        for i in range(0, len(md5_list)):
            print md5_list[i]
            if md5_list[i] == md5:
                return True
        return False
