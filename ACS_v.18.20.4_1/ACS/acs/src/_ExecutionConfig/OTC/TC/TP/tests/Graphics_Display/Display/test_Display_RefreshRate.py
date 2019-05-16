# -*- coding: utf-8 -*-
'''
@summary: Test display metricsReport
@since: 05/29/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.display_metrics_report_impl import DisplayMetricsReportImpl
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.photos_impl import get_photo_implement


class RefreshRate(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(RefreshRate, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(RefreshRate, self).setUp()
        self.displaymetrics = DisplayMetricsReportImpl()
        self.photosImpl = PhotosExtendImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self.remote_path = self.photosImpl.push_videos(count=1, like='H264_L3.2_HP_720p_60fps_AAC_192kb_44KHz', exts='.mp4')[0]

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        super(RefreshRate, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(RefreshRate, cls).tearDownClass()

    def test_Display_RefreshRate(self):
        print "[RunTest]: %s" % self.__str__()
        self.displaymetrics.compare_refresh_rate()
        self.photos.play_video_command(self.remote_path)
#         self.photos.launch_photos_am()
#         self.photos.play_video()
        time.sleep(10)
        self.photos.stop_photos_am()
        g_common_obj.assert_exp_happens()
