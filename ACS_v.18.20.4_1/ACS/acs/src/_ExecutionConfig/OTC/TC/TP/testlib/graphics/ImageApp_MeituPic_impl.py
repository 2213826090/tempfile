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
@summary: Class for ImageApp MeituPic
@since: 06/02/2015
@author: Zhang RongX,Z
'''
import time
from testlib.util.common import g_common_obj
from testlib.common.base import getTmpDir
from testlib.graphics.md5_impl import make_md5
from testlib.graphics.imagedetails_impl import ImageDetails

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def edit(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/btn_embellish")

    @property
    def choose_pic(self):
        return self._device(
            text="Pictures")

    def small_pic(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/album_thumb")

    @property
    def edit_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/btn_edit")

    @property
    def crop_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/edit_cut_finish")

    @property
    def rotate_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/rbtn_edit_rotate")

    @property
    def mirrot_horizontal_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/rotateHorizontal")

    @property
    def tip_info_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/tip_info")

    @property
    def done_btn(self):
        return self._device(
            resourceId="com.picsart.studio:id/btn_done")

    @property
    def save_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/btn_save")

    @property
    def ok_btn(self):
        return self._device(
            resourceId="com.mt.mtxx.mtxx:id/btn_ok")

class MeituPicEdit:
    '''
    MeituPicEdit
    '''

    pkg_name = "com.mt.mtxx.mtxx"
    activity_name = "com.meitu.mtxx.MainActivity"

    def __init__(self):
        self.tmpdir = getTmpDir()
        self._imageviewer = ImageDetails()
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch MeituPic via adb am command
        """
        print "Launch MeituPic by adb am"
        g_common_obj.launch_app_am(\
            MeituPicEdit.pkg_name, MeituPicEdit.activity_name)
        time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop MeituPic via adb am command
        """
        print "Stop MeituPic by adb am"
        g_common_obj.stop_app_am(MeituPicEdit.pkg_name)

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
        self._locator.edit.click.wait()
        time.sleep(2)
        self._locator.choose_pic.click.wait()
        time.sleep(2)
        self._device(resourceId="com.mt.mtxx.mtxx:id/album_thumb").click.wait()
        time.sleep(2)

    def edit_pic(self):
        """ edit picture
        """
        self._locator.edit_btn.click.wait()
        time.sleep(2)
        self._locator.crop_btn.click.wait()
        time.sleep(2)

    def mirror_effect(self):
        """ edit picture as mirro effect
        """
        self._locator.edit_btn.click.wait()
        time.sleep(2)
        self._locator.rotate_btn.click.wait()
        time.sleep(1)
        self._locator.tip_info_btn.click.wait()
        time.sleep(1)
        self._locator.mirrot_horizontal_btn.click.wait()
        time.sleep(2)

    def save_pic(self):
        """ save edit picture
        """
        self._locator.ok_btn.click.wait()
        time.sleep(1)
        self._locator.save_btn.click.wait()
        time.sleep(5)
        assert self._device(text="Saved to Album").exists, "pictures saved failed"

    def mirror_effect_check(self, md5_list):
        """ check mirror effect
        """
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls /sdcard/DCIM/Camera/*.jpg')
        compare = comp.strip("\n")
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        md5 = make_md5.make_md5_jumpbyte(176, self.tmpdir + '/compare.jpg')
        assert self._imageviewer.Md5_list_check(md5, md5_list), "check add effect fail"

    def mirror_effect_check_(self, file_path):
        """ check mirror effect
        """
        print "get template picture name"
        comp = g_common_obj.adb_cmd_capture_msg('ls /sdcard/DCIM/Camera/*.jpg')
        compare = comp.strip("\n")
        print "template file: " + compare
        g_common_obj.adb_cmd_common('pull ' + compare + ' ' + self.tmpdir + '/compare.jpg')
        assert g_common_obj.shell_cmd("diff -q %s %s" % (file_path, self.tmpdir + '/compare.jpg')) != '0', "check add effect fail"

    def uninstall_app(self):
        """ uninstall MeituPic
        """
        print "uninstall MeituPic"
        cmd = 'uninstall %s' % MeituPicEdit.pkg_name
        g_common_obj.adb_cmd_common(cmd)
