# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/06/2015
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

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.disconnect_chromecast()
        self._chromecast.swith_camera_to_capture_mode()
        self._chromecast.delete_captured_video()

    def test_camera_sdvideorecording(self):
        ''' refer TC test_Camera_SDVideoRecording
        '''
        self._chromecast.launch_camera_am()
        self._chromecast.set_record_sd_mode()
        self._chromecast.capture_a_video()
        self._chromecast.stop_camera_am()

