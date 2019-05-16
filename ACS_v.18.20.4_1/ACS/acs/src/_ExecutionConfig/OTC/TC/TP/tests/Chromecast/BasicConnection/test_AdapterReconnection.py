# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 02/26/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastconnection_impl import ChromeCastImpl

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
        self._chromecast.disconnect_cast_if_failed()

    def test_adapter_reconnection(self):
        ''' refer TC test_AdapterReconnection&test_AdapterConnection
        '''
        self._chromecast.launch_app_am()
        self._chromecast.goto_castscreen()
        self._chromecast.reconnect_chromecastx10()
        self._chromecast.stop_app_am()
