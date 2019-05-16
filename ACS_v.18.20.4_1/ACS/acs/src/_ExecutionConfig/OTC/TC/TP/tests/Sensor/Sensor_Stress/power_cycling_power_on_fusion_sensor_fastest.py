#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class power_cycling_power_on_fusion_sensor_fastest(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        #self.cfg_file = "tests.tablet.sensor.conf"
        testprjsenortest_package = "android.intel.umg.sensor.test"
        sensortest_package = "android.intel.umg.sensor"
        self.sensorImpl.install_artifactory_app("testprjsenortest", testprjsenortest_package)
        self.sensorImpl.install_artifactory_app("sensortest", sensortest_package)
        super(power_cycling_power_on_fusion_sensor_fastest, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(power_cycling_power_on_fusion_sensor_fastest, self).tearDown()


    def test_power_cycling_power_on_gravity_fastest(self):
        fileName = "frequency_gravity_fastest"
        freq_type = fileName.split('_')[2].strip()
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.set_screen_status("on")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)

            self.sensorImpl.power_off()
            self.sensorImpl.check_cos_mode()
            self.sensorImpl.power_on()
            time.sleep(5)
            if self.sensorImpl.sensor_execute_get_para_common(fileName, freq_type) is False:
                num += 1
        print num
        if num > 90:
            assert False


    def test_power_cycling_power_on_orientation_fastest(self):
        fileName = "frequency_orientation_fastest"
        freq_type = fileName.split('_')[2].strip()
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.set_screen_status("on")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)

            self.sensorImpl.power_off()
            self.sensorImpl.check_cos_mode()
            self.sensorImpl.power_on()
            time.sleep(5)
            if self.sensorImpl.sensor_execute_get_para_common(fileName, freq_type) is False:
                num += 1
        print num
        if num > 90:
            assert False

    def test_power_cycling_power_on_rotation_vector_fastest(self):
        fileName = "frequency_rotation_vector_fastest"
        freq_type = fileName.split('_')[3].strip()
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.set_screen_status("on")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)

            self.sensorImpl.power_off()
            self.sensorImpl.check_cos_mode()
            self.sensorImpl.power_on()
            time.sleep(5)
            if self.sensorImpl.sensor_execute_get_para_common(fileName, freq_type) is False:
                num += 1
        print num
        if num > 90:
            assert False

    def test_power_cycling_power_on_linear_accelerometer_fastest(self):
        fileName = "frequency_linear_accelerometer_fastest"
        freq_type = fileName.split('_')[3].strip()
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.set_screen_status("on")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)

            self.sensorImpl.power_off()
            self.sensorImpl.check_cos_mode()
            self.sensorImpl.power_on()
            time.sleep(5)
            if self.sensorImpl.sensor_execute_get_para_common(fileName, freq_type) is False:
                num += 1
        print num
        if num > 90:
            assert False



