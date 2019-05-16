# -*- coding:utf-8 -*-

'''
@summary: Verifiy that no satellites are detected while indoor.
@since: 06/21/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.gps.common import GPS_Common
from testlib.util.common import g_common_obj


class Detect_Satellites(UIATestBase):

    def setUp(self):
        super(Detect_Satellites, self).setUp()
        self._test_name = __name__
        self.device = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.gc = GPS_Common()
        self.gc.check_if_wifi_connected()
        self.gc.init_apk()
        self.gc.init_location()

    def test_receiverSensitivityAndNumberOfVisibleSatellitesIndoor(self):
        self.gc.check_if_wifi_connected()
        self.gc.kill_uiautomator()
        _cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.WithInternet#testNumberOfSatellitesIndoor \
        -w  com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner "
        _test_result = g_common_obj.adb_cmd_common(_cmd, 600)
        assert "OK" in _test_result, "%s failed" % self.__str__()
