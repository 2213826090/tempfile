#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class TemperatureDataPowerOnOff(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(TemperatureDataPowerOnOff, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TemperatureDataPowerOnOff, self).tearDown()

    def test_temperature_data_after_power_on_off(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.check_temperature_sensor_data()
        self.sensorImpl.power_off_on_from_os_by_ignition()
        self.sensorImpl.check_temperature_sensor_data()
