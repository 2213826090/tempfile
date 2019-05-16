#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl


class TemperatureDataResponding(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(TemperatureDataResponding, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TemperatureDataResponding, self).tearDown()

    def test_temperature_data_responding(self):
        print "[RunTest]: %s" % self.__str__()
        self.sensorImpl.check_temperature_sensor_data()
        self.sensorImpl.check_screen_response_comm()
        self.sensorImpl.check_temperature_sensor_data()
