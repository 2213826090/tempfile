#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl


class TemperatureDataReboot(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(TemperatureDataReboot, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TemperatureDataReboot, self).tearDown()

    def test_temperature_data_after_reboot(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.check_temperature_sensor_data()
        self.sensorImpl.reboot_cycle()
        self.sensorImpl.check_temperature_sensor_data()
