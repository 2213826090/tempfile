# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/05/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastcastscreen_impl import ChromeCastImpl

class ChromeCast(UIATestBase):

    def setUp(self):
        super(ChromeCast, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._chromecast = ChromeCastImpl()
        self._chromecast.connect_chromecast()
        self._chromecast.init_local_video()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.disconnect_chromecast()
        self._chromecast.delete_local_video()

    def test_screencasting_sleepdutduringvideoplayback(self):
        ''' refer TC test_ScreenCasting_SleepDUTduringVideoPlayback
        '''
        self._chromecast.launch_photos_am()
        self._chromecast.play_video_in_photos()
        self._chromecast.suspend_and_resume()
        self._chromecast.stop_photos_am()
