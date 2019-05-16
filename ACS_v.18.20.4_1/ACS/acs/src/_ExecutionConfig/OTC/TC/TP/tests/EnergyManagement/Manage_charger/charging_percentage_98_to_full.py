#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class ChargingPercentage98ToFull(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.set_screen_status("off")
        super(ChargingPercentage98ToFull, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_charging_percentage_98_to_full(self):
        """
        Test charging percentage 98 to full
        """
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.charge_to_percentage(98)
        self.emImpl.charge_to_percentage(100)

