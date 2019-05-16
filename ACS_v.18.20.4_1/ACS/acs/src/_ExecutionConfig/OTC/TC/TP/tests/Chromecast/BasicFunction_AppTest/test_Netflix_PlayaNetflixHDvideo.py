# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 02/11/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastnetflix_impl import ChromeCastImpl

class ChromeCast(UIATestBase):

    def setUp(self):
        super(ChromeCast, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._chromecast = ChromeCastImpl()
        self._chromecast.set_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.set_default_screen()
        self._chromecast.uninstall_app()

    def test_netflix_play_anetflix_hdvideo(self):
        ''' refer TC test_Netflix_PlayaNetflixHDvideo
        '''
        self._chromecast.launch_app_am()
        self._chromecast.login_account()
        self._chromecast.play_video_via_chromecast()
        self._chromecast.check_chromecast_status()
        self._chromecast.stop_app_am()
