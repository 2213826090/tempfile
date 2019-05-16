#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class RealBatteryAllowCharging(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_real_battery_allow_charging(self):
        """
        test real battery allow charging
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = "dumpsys battery | grep -E '(AC|USB) powered: true'"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        assert msg != ""

