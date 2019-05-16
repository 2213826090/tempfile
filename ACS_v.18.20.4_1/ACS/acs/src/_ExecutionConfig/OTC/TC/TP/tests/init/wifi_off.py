import os
from testlib.util.uiatestbase import UIATestBase
from testlib.wifi.wifi_impl import WifiSettingImpl

class ConnectWifiOff(UIATestBase):

    def setUp(self):
        super(ConnectWifiOff, self).setUp()
        cfg_file = 'tests.tablet.wifi.conf'
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.wifi = WifiSettingImpl({})

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ConnectWifiOff, self).tearDown()
        self.wifi = None

    def testWifiOff(self):
        """
        wifi off
        """
        print "[RunTest]: %s" % self.__str__()
        self.wifi.launch_from_settings()
        self.wifi.off()

