# -*- coding: utf-8 -*-
'''
Created on 01/25/2016
@author: Zhao Xiangyi
'''

import time
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_video_impl import VideoExtendImpl
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import artifactory


class DisplayDynamicRefreshRate(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DisplayDynamicRefreshRate, cls).setUpClass()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DisplayDynamicRefreshRate, self).setUp()
        self.d = g_common_obj.get_device()
        self.videoImpl = VideoExtendImpl()
        self.photosImpl = PhotosExtendImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

        test_config = TestConfig()
        conf_file = 'tests.tablet.artifactory.conf'
        self.config = test_config.read(conf_file, "content_video_apk")

        video_path = self.config.get('video_path1')
        push_file_path = artifactory.get(video_path)
        g_common_obj.adb_cmd_common('push ' + push_file_path + ' /sdcard/Movies')
        self.push_path = '/sdcard/Movies/' + self.config.get("video_file1")

        self.videoImpl.appPrepare()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        super(DisplayDynamicRefreshRate, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DisplayDynamicRefreshRate, cls).tearDownClass()

    def test_DisplayDynamicRefreshRate(self):
        print "[RunTest]: %s" % self.__str__()
        self.videoImpl.launchVideoApp()
        time.sleep(2)
        self.videoImpl.videoPlayBack(self.push_path)
        try:
            for _ in range(10):
                ct, tt = self.videoImpl.get_play_time()
                if tt - ct >= 50:
                    self.videoImpl.checkVideoPlayBackWithComparePicture(20)
                ret = self.videoImpl.checkVideoPlayBackComplete(300)
                if ret == 1:
                    break
        except Exception as e:
            assert self.d(textContains="Completed").exists, ("Error:", e)
            print "Error:", e
        print "case " + self.__str__() + " is pass"
