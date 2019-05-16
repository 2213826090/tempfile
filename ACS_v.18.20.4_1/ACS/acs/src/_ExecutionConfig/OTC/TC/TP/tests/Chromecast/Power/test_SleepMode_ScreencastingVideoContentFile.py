# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/11/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastcastscreen_impl import ChromeCastImpl
from testlib.graphics.photos_impl import get_photo_implement


class ChromeCast(UIATestBase):

    def setUp(self):
        super(ChromeCast, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._chromecast = ChromeCastImpl()
        self._photos = get_photo_implement()
        self._chromecast.connect_chromecast()
        self._chromecast.init_local_video()
        self._photos.refresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.disconnect_chromecast()
        self._photos.rm_delete_photos()
        self._photos.refresh_sdcard()

    def test_sleepmode_screencastingvideocontentfile(self):
        ''' refer TC test_SleepMode_ScreencastingVideoContentFile
        '''
        self._photos.launch_photos_am()
        self._photos.play_video()
        self._chromecast.suspend_and_resume()
        self._photos.stop_photos_am()
