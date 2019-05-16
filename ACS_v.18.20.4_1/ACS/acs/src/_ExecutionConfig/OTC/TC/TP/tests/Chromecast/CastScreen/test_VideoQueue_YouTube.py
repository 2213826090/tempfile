# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/03/2015
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

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()

    def test_videoqueue_youtube(self):
        ''' refer TC test_VideoQueue_YouTube
        '''
        self._chromecast.launch_youtube_am()
        self._chromecast.play_video_in_youtube()
        self._chromecast.add_three_video_in_queue()
        self._chromecast.stop_youtube_am()
