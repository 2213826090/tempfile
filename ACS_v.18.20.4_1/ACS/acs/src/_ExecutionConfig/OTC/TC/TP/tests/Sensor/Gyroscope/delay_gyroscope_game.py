#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl

class delay_gyroscope_game(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        self.apk1_package = "android.intel.umg.sensor.test"
        self.apk2_package = "android.intel.umg.sensor"
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        # self.cfg_file = self.sensorImpl.match_configuration()
        self.cfg_file = "tests.tablet.sensor.conf"
        self.sensorImpl.install_app(self.config.read(self.cfg_file, "artifactory"), "location_apks", "testprjsenortest", self.apk1_package)
        self.sensorImpl.install_app(self.config.read(self.cfg_file, "artifactory"), "location_apks", "sensortest", self.apk2_package)
        super(delay_gyroscope_game, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(delay_gyroscope_game, self).tearDown()

    def test_delay_gyroscope_game(self):
        fileName = os.path.split(os.path.realpath(__file__))[1]
        freq_type = fileName.split('_')[2].split('.')[0].strip()

        cfg = self.config.read(self.cfg_file,'delay_gyroscope_game')
        catagory = cfg.get("catagory")
        sensorName = cfg.get("sensorname")
        compare_result = cfg.get("compare_result")
        para1 = cfg.get("para1")
        para2 = cfg.get("para2")
        num = 0

        for loop in range(10):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/10 delay_gyroscope_game" %(loop+1))
            if self.sensorImpl.excute(fileName, catagory, sensorName, compare_result, para1, para2, freq_type) is False:
                num += 1
        if num > 5:
            assert False

