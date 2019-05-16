# -*- coding:utf-8 -*-

'''
@summary: Verify that location On/Off takes immediate effect.
@since: 06/21/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.gps.common import GPS_Common
from testlib.util.common import g_common_obj

class Location_OnOff_Effect(UIATestBase):

    def setUp(self):
        super(Location_OnOff_Effect, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gc = GPS_Common()
        self.gc.check_if_wifi_connected()
        self.gc.init_apk()
        self.gc.init_location()
        (self._lattd, self._longtd) = self.gc.fetch_latitude_longitude()

    def test_turnOnOffLocationServiceAndCheckIfSettingIsObeyed(self):
        self.gc.kill_uiautomator()
        _cmd = "shell am instrument \
        -e class com.intel.uitests.tests.comms.location.Coordinates#testTurnOnOffLocationSettingIsObeyed \
        -e latitude %s -e longitude %s \
        -w  com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner " % (self._lattd, self._longtd)
        _test_result = g_common_obj.adb_cmd_common(_cmd, 600)
        assert "OK" in _test_result, "%s failed" % self.__str__()