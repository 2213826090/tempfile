#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class jitter_compass_ui(UIATestBase):
    '''
    calculate the jitter of compass sensor ui delay time
    '''

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        configuration = "tests.tablet.sensor.conf"
        self.cfg_file = configuration
        self.sensorImpl = SensorImpl()
        super(jitter_compass_ui, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(jitter_compass_ui, self).tearDown()

    def test_jitter_compass_ui(self):

        fileName = os.path.split(os.path.realpath(__file__))[1]
        
        self.sensorImpl.sensor_log(self.config.read(self.cfg_file, "artifactory"), "UI", "magnetic field", 198)
        self.sensorImpl.check_log(40)

