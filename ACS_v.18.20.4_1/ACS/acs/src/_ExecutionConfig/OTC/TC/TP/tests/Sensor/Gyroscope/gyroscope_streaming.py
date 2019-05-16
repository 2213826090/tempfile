#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class gyroscope_streaming(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.sensortest_package = "android.intel.umg.sensor.test"
        self.sensor_package = "android.intel.umg.sensor"
        self.sensorImpl = SensorImpl()
        self.cfg_file = "tests.tablet.sensor.conf"
        self.sensorImpl.install_app(self.config.read(self.cfg_file, "artifactory"), "location_apks", "testprjsenortest", self.sensortest_package)
        self.sensorImpl.install_app(self.config.read(self.cfg_file, "artifactory"), "location_apks", "sensortest", self.sensor_package)
        super(gyroscope_streaming, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(gyroscope_streaming, self).tearDown()

    def test_gyroscope_streaming(self):
        fileName = os.path.split(os.path.realpath(__file__))[1]
        sensor_type = fileName.split('_')[0].strip()

        num = 0
        for loop in range(10):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/10 test gyroscope delay" %(loop+1))
            if self.sensorImpl.excute(fileName, "delay", sensor_type, str(10), "fastest", str(0), "fastest") is False:
                num += 1
        if num > 5:
            assert False

        self.sensorImpl.logger.info("test gyroscope jitter--------^_^--------")
        self.sensorImpl.sensor_log(self.config.read(self.cfg_file, "artifactory"), "Fastest", sensor_type)
        self.sensorImpl.check_log()

