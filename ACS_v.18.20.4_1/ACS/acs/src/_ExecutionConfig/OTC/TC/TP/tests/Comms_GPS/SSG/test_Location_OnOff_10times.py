# -*- coding:utf-8 -*-

'''
@summary: Test location on/off for 10 times. 
@since: 06/21/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.gps.common import GPS_Common
from testlib.util.common import g_common_obj
from testlib.util.log import Logger


class Location_OnOff(UIATestBase):

    def setUp(self):
        super(Location_OnOff, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.d = g_common_obj.get_device()
        self.l = Logger.getlogger(__name__)
        self.gc = GPS_Common()
        self.gc.check_if_wifi_connected()
        self.gc.init_apk()
        self.gc.Turn_On_Location()
        self.gc.init_location()
        (self._lattd, self._longtd) = self.gc.fetch_latitude_longitude()

    def test_turnOnOffLocation10Times(self):
#         self.gc.kill_uiautomator()
#         _cmd = "shell am instrument \
#         -e class com.intel.uitests.tests.comms.location.CoordinatesIterative#testTurnOnOffLocationIterative \
#         -e latitude %s -e longitude %s -e iterations 10 \
#         -w  com.intel.uitests.test/com.intel.uitests.runner.UiTestRunner " % (self._lattd, self._longtd)
#         _test_result = g_common_obj.adb_cmd_common(_cmd, 600)
#         assert "OK" in _test_result, "%s failed" % self.__str__()
        for i in range(10):
            self.gc.Turn_Off_Location()
            self.gc.location_not_get()
            self.gc.Turn_On_Location()
            self.gc.get_location_and_check_accuracy(self._lattd, self._longtd)
            self.l.info("GPS On/Off cycle %s Pass." % (i+1))