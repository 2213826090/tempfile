#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class PowerShowBatteryFull(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.set_screen_status("off")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_power_show_battery_full(self):
        """
        Test power show battery full
        """
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.charge_to_percentage(100)
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.check_charging_full()

