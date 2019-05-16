# -*- coding: utf-8 -*-
'''
@summary: Boot up
@since: 09/18/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.splash_impl import SplashImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import file_sys, adb32, logcat


class SplashScreen(UIATestBase):

    def setUp(self):
        super(SplashScreen, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.common = g_common_obj
        self.d = self.common.get_device()
        self.sImpl = SplashImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SplashScreen, self).tearDown()

    def test_Display_BootingUp(self):
        '''
        Need enhance;
        So far it's only support to check boot animation file existing in os,
        but not able to check animation status while booting into OS.
        '''
        print "[RunTest]: %s" % self.__str__()
        self.sImpl.check_earlyEvs_init()
        self.sImpl.check_static_splashfile()
        self.sImpl.check_animation_splashfile()


class RefreshRate(UIATestBase):

    def setUp(self):
        super(RefreshRate, self).setUp()
        self._test_name = __name__
        self.photosImpl = get_photo_implement()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RefreshRate, self).tearDown()
        self.photosImpl.rm_delete_photos()

    def test_DisplayRefreshRate_after_reboot(self):
        self.photosImpl.deploy_photo_content('Videos', 'video_007', '/sdcard/Movies')
        video_file = file_sys.get_file_list('/sdcard/Movies')[0]
        self.photosImpl.play_video_command(video_file, 10)
        g_common_obj.assert_exp_happens()
        adb32._adb_reboot()
        assert logcat.get_refresh_rate() >= 60, 'Refresh rate is less than 60.'
        self.photosImpl.play_video_command(video_file, 10)
        g_common_obj.assert_exp_happens()

    def test_RefreshRate_Display_before_and_after_playing_video(self):
        self.photosImpl.deploy_photo_content('Videos', 'video_007', '/sdcard/Movies')
        video_file = file_sys.get_file_list('/sdcard/Movies')[0]
        assert logcat.get_refresh_rate() >= 60, 'Refresh rate is less than 60.'
        self.photosImpl.play_video_command(video_file, 10)
        g_common_obj.assert_exp_happens()
        assert logcat.get_refresh_rate() >= 60, 'Refresh rate is less than 60.'
