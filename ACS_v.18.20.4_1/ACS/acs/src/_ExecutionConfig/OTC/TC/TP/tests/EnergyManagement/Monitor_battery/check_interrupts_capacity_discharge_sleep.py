#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.power import get_power_obj
from testlib.em.usb_cut import USBCut

class check_interrupts_capacity_discharge_sleep(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.power = get_power_obj()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_interrupts_capacity_discharge_sleep(self):

        print "[RunTest]: %s" % self.__str__()

        self.power.adb_root()
        cmd0 = "rm -rf /data/logs/aplog*"
        self.power.testDevice.adb_cmd(cmd0)

        self.power.adb_reboot()
        USBCut().cut(1200)
        self.power.adb_root()
        cmd = "grep 'max17042-INTR: Status-val:400' -ri /data/logs/aplog*"
        result = self.power.testDevice.adb_cmd_capture_msg(cmd)
        if result:
            assert False, "Aplogs should not contain 'max17042-INTR: Status-val:400'."

