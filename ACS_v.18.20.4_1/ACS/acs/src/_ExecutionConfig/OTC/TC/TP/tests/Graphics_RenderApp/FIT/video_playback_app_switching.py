# -*- coding: utf-8 -*-
'''
Created on Feb 5, 2015
@author: Ding, JunnanX
'''
import time
import os
import tempfile
from testlib.util.common import g_common_obj
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.gfxbench_impl import Gfxbench
from testlib.graphics.QRcode_impl import QRcode
from testlib.util.uiatestbase import UIATestBase


class VideoPlayback(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(VideoPlayback, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(VideoPlayback, self).setUp()
        self.photos = get_photo_implement()
        self.bench = Gfxbench()
        self._qr = QRcode()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(count=1, like='test_video_qr', exts='.mp4')[0]

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self.photos.stop_photos_am()
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    def test_AppSwitching(self):
        print "[RunTest]: %s" % self.__str__()
        self.photos.play_video_command(self.remote_path, 5)
        self.bench.setup()
        self.bench.run_test("ALU 2", 50000)
        time.sleep(10)
        self.photos.play_video_command(self.remote_path, 5)
        screen_temp = tempfile.mktemp(suffix='.png', prefix='screenshot_', dir='/tmp')
        g_common_obj.take_screenshot(screen_temp)
        ret, qrcode = self._qr.decode_image_qrcode(screen_temp)
        assert ret and qrcode == "QRCODE_TEST_STRING", "Video playback failed!"

