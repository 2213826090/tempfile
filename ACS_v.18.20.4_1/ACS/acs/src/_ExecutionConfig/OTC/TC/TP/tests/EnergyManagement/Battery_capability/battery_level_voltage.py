#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.level_voltage import verify_level_voltage_match
from testlib.em.energy import Energy

class BatteryLevelVoltage(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.adb_root()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_battery_level_at_voltage_3_6(self):
        """
        Test battery level at voltage 3.6V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(3, 3600)

    def test_battery_level_at_voltage_3_7(self):
        """
        Test battery level at voltage 3.7V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(15, 3700)

    def test_battery_level_at_voltage_3_8(self):
        """
        Test battery level at voltage 3.8V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(38, 3800)

    def test_battery_level_at_voltage_3_9(self):
        """
        Test battery level at voltage 3.9V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(57, 3900)

    def test_battery_level_at_voltage_4_0(self):
        """
        Test battery level at voltage 4.0V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(68, 4000)

    def test_battery_level_at_voltage_4_1(self):
        """
        Test battery level at voltage 4.1V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(80, 4100)

    def test_battery_level_at_voltage_4_2(self):
        """
        Test battery level at voltage 4.2V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(90, 4200)

    def test_battery_level_at_voltage_4_3(self):
        """
        Test battery level at voltage 4.3V
        """
        print "[RunTest]: %s" % self.__str__()
        verify_level_voltage_match(98, 4300)

