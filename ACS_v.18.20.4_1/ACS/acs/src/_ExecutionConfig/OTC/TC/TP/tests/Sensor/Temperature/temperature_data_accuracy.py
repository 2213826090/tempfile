#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class TemperatureDataAccuracy(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(TemperatureDataAccuracy, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TemperatureDataAccuracy, self).tearDown()

    def test_temperature_data_accuracy(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.check_temperature_sensor_accuracy()

