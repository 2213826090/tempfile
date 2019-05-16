#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class delay_accelerometer_all_type_10_minutes(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        # self.cfg_file = self.sensorImpl.match_configuration()
        #self.cfg_file = "tests.tablet.sensor.conf"
        testprjsenortest_package = "android.intel.umg.sensor.test"
        sensortest_package = "android.intel.umg.sensor"
        self.sensorImpl.install_artifactory_app("testprjsenortest", testprjsenortest_package)
        self.sensorImpl.install_artifactory_app("sensortest", sensortest_package)
        super(delay_accelerometer_all_type_10_minutes, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(delay_accelerometer_all_type_10_minutes, self).tearDown()

    def test_delay_accelerometer_all_type_10_minutes(self):
        fileName_ = "delay_accelerometer_"
        type_fastest = "fastest"
        type_game = "game"
        type_normal = "normal"
        type_ui = "ui"
        num = 0
        for loop in range(15):
            self.sensorImpl.logger.info("Loop %d/15 delay_accelerometer_all_type_10_minutes" % (loop + 1))
            self.sensorImpl.logger.info("File Name: " + fileName_+type_fastest)
            if self.sensorImpl.sensor_execute_get_para_common(fileName_+type_fastest, type_fastest) is False:
                num += 1
            time.sleep(3)
            self.sensorImpl.logger.info("File Name: " + fileName_ + type_game)
            if self.sensorImpl.sensor_execute_get_para_common(fileName_ + type_game, type_game) is False:
                num += 1
            time.sleep(3)
            self.sensorImpl.logger.info("File Name: " + fileName_ + type_normal)
            if self.sensorImpl.sensor_execute_get_para_common(fileName_ + type_normal, type_normal) is False:
                num += 1
            time.sleep(3)
            self.sensorImpl.logger.info("File Name: " + fileName_ + type_ui)
            if self.sensorImpl.sensor_execute_get_para_common(fileName_ + type_ui, type_ui) is False:
                num += 1
        print num
        if num > 19:
            assert False

