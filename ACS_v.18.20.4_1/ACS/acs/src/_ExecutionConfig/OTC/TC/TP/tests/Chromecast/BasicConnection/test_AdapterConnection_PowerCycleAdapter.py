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

    def test_adapter_connection_reboot_adapter(self):
        ''' refer TC test_AdapterConnection_PowerCycleAdapter
        '''
        self._chromecast.launch_app_am()
        self._chromecast.goto_castscreen()
        self._chromecast.scan_adapter()
        self._chromecast.connect_chromecast()
        self._chromecast.reboot_chromecast_adapter()
        self._chromecast.scan_adapter()
        self._chromecast.connect_chromecast()
        self._chromecast.disconnect_chromecast()
        self._chromecast.stop_app_am()
