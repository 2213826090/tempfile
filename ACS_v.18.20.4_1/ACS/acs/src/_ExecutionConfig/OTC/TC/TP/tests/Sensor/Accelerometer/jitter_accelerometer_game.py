#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class jitter_accelerometer_game(UIATestBase):
    '''
    calculate the jitter of accelerometer sensor game delay time
    '''

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg_file = "tests.tablet.sensor.conf"
        self.sensorImpl = SensorImpl()
        super(jitter_accelerometer_game, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(jitter_accelerometer_game, self).tearDown()

    def test_jitter_accelerometer_game(self):

        fileName = os.path.split(os.path.realpath(__file__))[1]
        sensor_type = fileName.split('_')[1].strip()

        self.sensorImpl.sensor_log(self.config.read(self.cfg_file, "artifactory"), "Game", sensor_type)
        self.sensorImpl.check_log()

