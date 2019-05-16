#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy

class check_FG_driver_registration(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_FG_driver_registration(self):

        print "[RunTest]: %s" % self.__str__()
        self.energy.adb_root()
        cmd = "ls /sys/bus/i2c/drivers/ | grep max"
        result = self.energy.testDevice.adb_cmd_capture_msg(cmd)
        if not result:
            assert False, "Fuel Gauger Driver Registration Not Exist."

