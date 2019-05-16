#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal import ThermalNormal

class ThermalZone(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = ThermalNormal()
        self.thermal.adb_root()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_thermal_zone0(self):
        self.thermal.check_thermal_zone(0)

    def test_check_thermal_zone1(self):
        self.thermal.check_thermal_zone(1)

    def test_check_thermal_zone2(self):
        self.thermal.check_thermal_zone(2)

    def test_check_thermal_zone3(self):
        self.thermal.check_thermal_zone(3)

    def test_check_thermal_zone4(self):
        self.thermal.check_thermal_zone(4)

    def test_check_thermal_zone5(self):
        self.thermal.check_thermal_zone(5)

    def test_check_thermal_zone6(self):
        self.thermal.check_thermal_zone(6)

    def test_check_thermal_zone7(self):
        self.thermal.check_thermal_zone(7)

    def test_check_thermal_zone8(self):
        self.thermal.check_thermal_zone(8)

