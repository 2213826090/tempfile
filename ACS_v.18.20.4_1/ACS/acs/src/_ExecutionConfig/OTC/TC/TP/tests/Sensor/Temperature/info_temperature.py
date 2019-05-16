#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class InfoTemperature(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(InfoTemperature, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InfoTemperature, self).tearDown()

    def test_info_temperature(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.launch_My_Sensor_App()
        self.sensorImpl.check_temperature_sensor_info("Name")
        self.sensorImpl.check_temperature_sensor_info("Type")
        self.sensorImpl.check_temperature_sensor_info("Version")
        self.sensorImpl.check_temperature_sensor_info("Delay")
