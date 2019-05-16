#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class PowerRefreshBatteryStatus(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        super(PowerRefreshBatteryStatus, self).setUp()
        self.emImpl = EMImpl()
        self.emImpl.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PowerRefreshBatteryStatus, self).tearDown()

    def test_power_refresh_battery_status(self):
        """
        Test power refresh battery status
        """
        print "[RunTest]: %s" % self.__str__()

        self.emImpl.verify_charging_from_settings()

