"""
@summary: Test wifi connect 
@since: 10/1/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.wifi.wifisetting_impl import WifiSettingImpl

class WiFiConnect(UIATestBase):
    """
    @summary: used to test test wifi connect function
    """

    def setUp(self):
        super(WiFiConnect, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.google_fast.conf'
        self.wifi = WifiSettingImpl(\
            self.config.read(cfg_file, 'wificonnect'))
        self.wifi.set_orientation_n()

    def tearDown(self):
        super(WiFiConnect, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testWiFiConnect(self):
        """
        This test used to test create a note function.
        The test case spec is following:
        1. Launch the "setting" and connect a wifi.
        2. verify that connect a ap success.
        """

        print "[RunTest]: %s" % self.__str__()
        self.wifi.wifi_connect()