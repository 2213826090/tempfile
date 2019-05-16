#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl
from testlib.sensor.sensor_common import SensorCommon

class light_get_number(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()

        self.sensorCommon = SensorCommon()
        #self.cfg_file = "tests.tablet.sensor.conf"
        testprjsenortest_package = "android.intel.umg.sensor.test"
        sensortest_package = "android.intel.umg.sensor"
        self.sensorImpl.install_artifactory_app("testprjsenortest", testprjsenortest_package)
        self.sensorImpl.install_artifactory_app("sensortest", sensortest_package)
        super(light_get_number, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(light_get_number, self).tearDown()

    def test_light_get_number(self):
        sensor_name = "Light"
        self.sensorImpl.basicTest_SensorTest_App(sensor_name)
        self.sensorImpl.light_check_reuslt(sensor_name)
