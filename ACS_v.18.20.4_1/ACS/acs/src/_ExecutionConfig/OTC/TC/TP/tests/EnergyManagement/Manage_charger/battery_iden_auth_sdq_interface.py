#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class BatteryIdenAuth(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_battery_iden_auth(self):
        """
        test battery identification and authentication SDQ interface
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = "od -x /sys/firmware/acpi/tables/OEM0 | head -n1"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        msg_arr = msg.split()
        assert len(msg_arr) == 9
        # varify there are only hexadecimal numbers
        for item in msg_arr:
            int(item, 16)


