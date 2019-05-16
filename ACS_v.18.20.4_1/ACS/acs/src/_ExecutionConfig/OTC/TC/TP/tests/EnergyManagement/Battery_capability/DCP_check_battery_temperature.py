#! /usr/bin/env python
# coding:utf-8

import os
import time

from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class DCP_check_battery_temperature(UIATestBase):
    """
    Connecting DCP charging 30 minitus, check Battery temperature is not too higt
    """
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_DCP_check_battery_temperature(self):

        print "[RunTest]: %s" % self.__str__()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("15 seconds")

        self.emImpl.enable_dcp_charging()
        time.sleep(1800)
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)

        temperature = self.emImpl.get_power_temperature()
        if temperature < 0:
            assert False, "Check temperature < 0 on DCP charging."
        if temperature > 450:
            assert False, "temperature is too high on DCP charging."

