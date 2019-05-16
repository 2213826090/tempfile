# -*- coding:utf-8 -*-

'''
@summary: GPS positioning test, 1: with WiFi connection; 2: w/o WiFi connection.
@since: 06/20/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.gps.common import GPS_Common
from testlib.util.common import g_common_obj


class GPSPositioning(UIATestBase):

    def setUp(self):
        super(GPSPositioning, self).setUp()
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.gc = GPS_Common()
        self.gc.check_if_wifi_connected()
        self.gc.init_apk()
        self.gc.init_location()

    def test_GPSPositioningWhenWifiIsConnectedAndStayIndoors(self):
        self.gc.check_if_wifi_connected()
        self.gc.kill_uiautomator()
        test_cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.WithInternet#testGpsPositioningWifiConnectedIndoor \
        -w com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner"
        test_result = g_common_obj.adb_cmd_common(test_cmd, 600)
        assert "OK" in test_result, "%s failed" % self.__str__()

    def test_GPSPositioningWhenWifiIsDisconnectedAndStayIndoors(self):
        self.gc.kill_uiautomator()
        test_cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.NoInternet#testNetworksPositioningWifiDisconnectedIndoor \
        -w com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner"
        test_result = g_common_obj.adb_cmd_common(test_cmd, 600)
        assert "OK" in test_result, "%s failed" % self.__str__()
