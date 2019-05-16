#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.power import get_power_obj

class FG_graceful_shutdown_aplogs(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.power = get_power_obj()
        self.power.adb_root()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_FG_graceful_shutdown_aplogs(self):

        print "[RunTest]: %s" % self.__str__()
        cmd0 = "rm -rf /data/logs/aplog*"
        self.power.testDevice.adb_cmd(cmd0)

        self.power.reboot()
        self.power.adb_root()

        cmd1 = "grep 'MCFG' -ri /data/logs/aplog"
        cmd2 = "grep 'BCFG' -ri /data/logs/aplog"
        result1 = self.power.testDevice.adb_cmd_capture_msg(cmd1)
        result2 = self.power.testDevice.adb_cmd_capture_msg(cmd2)
        print result1
        print result2
        if not result1 and not result2:
            assert False, "Fuel Gauger Graceful Shutdown Aplogs Not Exist."

