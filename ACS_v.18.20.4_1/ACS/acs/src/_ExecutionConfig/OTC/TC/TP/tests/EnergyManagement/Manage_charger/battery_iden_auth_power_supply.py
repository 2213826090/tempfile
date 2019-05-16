#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class IdenAuthPowerSupply(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_iden_auth_power_supply(self):
        """
        test battery identification and authentication SDQ interface power supply
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = "'ls /sys/firmware/acpi/tables/OEM0 2>/dev/null'"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        assert msg == ""

