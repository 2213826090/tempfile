# -*- coding: utf-8 -*-
'''
Created on 03/10/2016
@author: Zhao, XiangyiX
'''

import re
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.extend_chrome_impl import ChromeExtendImpl
from testlib.graphics.extend_photos_impl import PhotosExtendImpl


class DPSTFunctionalityTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DPSTFunctionalityTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DPSTFunctionalityTest, self).setUp()
        self.photos = get_photo_implement()
        self.chrome = ChromeExtendImpl()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(count=1, like='dark', exts='.mp4')[0]
        self.chrome.launch()
        self.chrome.chrome_setup()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DPSTFunctionalityTest, self).tearDown()
        self.photos.rm_delete_photos()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DPSTFunctionalityTest, cls).tearDownClass()

    def test_DPST_Functionality(self):
        """
        test_DPST_Functionality

        Steps:
        1. Connect DUT to linux host, execute commands: 
        adb root
        adb shell cat /sys/kernel/debug/dri/0/i915_dpst_status
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. adb shell cat /sys/kernel/debug/dri/0/i915_dpst_status.
            return backlight adjustment value."""
        flag = "backlight adjustment:"
        cmd = 'cat /sys/kernel/debug/dri/0/i915_dpst_status'
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        info = msg.split('\n')
        for line in info:
            if not line.find(flag) == -1:
                print line
                light1 = re.sub(r'\D', "", line)

        print """[Step] 2. Launch Chrome and play the video.
            Video played successfully. No artifacts or flickers."""
        self.chrome.launch()
        time.sleep(3)
        self.chrome.open_website('file://%s' % (self.remote_path))
        self.chrome.play_video(secs=5)

        print """[Step] 3. adb shell cat /sys/kernel/debug/dri/0/i915_dpst_status.
            return backlight adjustment value and check it less."""
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        info = msg.split('\n')
        for line in info:
            if not line.find(flag) == -1:
                print line
                light2 = re.sub(r'\D', "", line)
        assert light2 < light1, \
            "[FAILURE] The backlight adjustment did not less than before after play vedio.now is from %s to %s\n" % (light1, light2)
