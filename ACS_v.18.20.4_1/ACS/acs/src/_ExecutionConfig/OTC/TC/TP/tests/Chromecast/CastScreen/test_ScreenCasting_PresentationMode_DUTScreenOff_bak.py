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
        self._chromecast.connect_chromecast()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.disconnect_chromecast()

    def test_screencasting_presentationmode_dutscreenoff(self):
        ''' refer TC test_ScreenCasting_PresentationMode_DUTScreenOff
        '''
        self._chromecast.install_apidemos()
        self._chromecast.launch_apidemos_am()
        self._chromecast.run_medie_router()
        self._chromecast.suspend_and_resume()
        self._chromecast.stop_apidemos_am()
