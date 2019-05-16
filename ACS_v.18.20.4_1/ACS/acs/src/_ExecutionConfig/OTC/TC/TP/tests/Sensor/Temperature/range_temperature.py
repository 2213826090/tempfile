#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class RangeTemperature(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(RangeTemperature, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RangeTemperature, self).tearDown()

    def test_range_temperature(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.launch_My_Sensor_App()
        self.sensorImpl.check_temperature_sensor_info("Range")

