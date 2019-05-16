#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl
from testlib.sensor.sensor_common import SensorCommon

class gyroscope_longlasting_streaming(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        self.sensorCommon = SensorCommon()
        self.sensorCommon.keep_awake()
        super(gyroscope_longlasting_streaming, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(gyroscope_longlasting_streaming, self).tearDown()


    def test_gyroscope_longlasting_streaming(self):
        fileName = os.path.split(os.path.realpath(__file__))[1]
        sensor_type = fileName.split('_')[0].strip()
        sensor_longlasting_mark = "longlasting"

        self.sensorImpl.logger.info("-------test %s jitter----------------" % sensor_type)
        self.sensorImpl.sensor_log(sensor_longlasting_mark, "Fastest", sensor_type)
        num = 0
        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            #self.sensorImpl.check_log()

            self.sensorImpl.longlasting_20min_get()
            if self.sensorImpl.long_lasting_check_log() is False:
                num += 1
        print num
        if num > 90:
            self.sensorImpl.long_lasting_check_log()
            assert self.sensorImpl.long_lasting_check_log()
