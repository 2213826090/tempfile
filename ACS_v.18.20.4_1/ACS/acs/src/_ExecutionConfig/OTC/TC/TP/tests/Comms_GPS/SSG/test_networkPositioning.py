# -*- coding:utf-8 -*-

'''
@summary: Network positioning test, 1: with WiFi connection; 2: w/o WiFi connection.
@since: 06/21/2016
@author: Lijin Xiong
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gps.common import GPS_Common
from testlib.util.common import g_common_obj
from testlib.dut_init.dut_init_impl import Function


class NetworkPositioning(UIATestBase):

    def setUp(self):
        super(NetworkPositioning, self).setUp()
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.func=Function()
        self.gc = GPS_Common()
        self.gc.check_if_wifi_connected()
        self.gc.init_apk()
        self.gc.init_location()
        (self._lattd, self._longtd) = self.gc.fetch_latitude_longitude()

    def test_networkPositioningWhenWifiIsConnectedAndStayIndoors(self):
        self.gc.kill_uiautomator()
        _cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.Coordinates#testNetworkingPositioningWifiConnectedIndoor \
        -e latitude %s -e longitude %s \
        -w  com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner " % (self._lattd, self._longtd)
        _test_result = g_common_obj.adb_cmd_common(_cmd, 600)
        assert "OK" in _test_result, "%s failed" % self.__str__()

    def test_networkPositioningWhenWifiIsDisconnectedAndStayIndoors(self):
        self.gc.kill_uiautomator()
        _cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.NoInternet#testNetworksPositioningWifiDisconnectedIndoor \
        -w  com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner "
        _test_result = g_common_obj.adb_cmd_common(_cmd, 600)
        assert "OK" in _test_result, "%s failed" % self.__str__()
