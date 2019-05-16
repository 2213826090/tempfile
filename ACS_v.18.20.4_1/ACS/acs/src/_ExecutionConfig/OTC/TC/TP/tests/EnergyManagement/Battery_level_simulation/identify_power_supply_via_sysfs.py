#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class IdentifyPowerSupply(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_identify_power_supply_via_sysfs(self):
        """
        test identify power supply via sysfs
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = "cat /sys/class/power_supply/battery/uevent | grep POWER_SUPPLY_TECHNOLOGY=Unknown"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        assert msg != ""


