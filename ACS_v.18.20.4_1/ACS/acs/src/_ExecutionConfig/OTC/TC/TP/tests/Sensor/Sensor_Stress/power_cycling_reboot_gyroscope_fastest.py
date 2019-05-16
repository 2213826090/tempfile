#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class power_cycling_reboot_gyroscope_fastest(UIATestBase):

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
        super(power_cycling_reboot_gyroscope_fastest, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(power_cycling_reboot_gyroscope_fastest, self).tearDown()


    def test_power_cycling_reboot_gyroscope_fastest(self):
        file_Name = os.path.split(os.path.realpath(__file__))[1].split('.')[0].split('_', 3)[3]
        str_fre = 'frequency_'
        fileName = str_fre + file_Name
        freq_type = fileName.split('_')[2].strip()
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            self.sensorImpl.reboot_cycle()
            time.sleep(5)
            if self.sensorImpl.sensor_execute_get_para_common(fileName, freq_type) is False:
                num += 1
        print num
        if num > 90:
            assert False
