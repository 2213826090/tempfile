#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl
from testlib.sensor.sensor_common import SensorCommon

class fusion_sensor_longlasting_streaming(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        self.sensorCommon = SensorCommon()
        self.sensorCommon.keep_awake()
        super(fusion_sensor_longlasting_streaming, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(fusion_sensor_longlasting_streaming, self).tearDown()


    def test_gravity_longlasting_streaming(self):
        fileName = "gravity_longlasting_streaming"
        sensor_type = fileName.split('_')[0].strip()
        sensor_longlasting_mark = "longlasting"
        num = 0
        self.sensorImpl.logger.info("-------test %s jitter----------------" % sensor_type)
        self.sensorImpl.sensor_log(sensor_longlasting_mark, "Fastest", sensor_type)

        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            self.sensorImpl.longlasting_20min_get()
            if self.sensorImpl.long_lasting_check_log() is False:
                num += 1
        print num
        if num > 90:
            self.sensorImpl.long_lasting_check_log()
            assert self.sensorImpl.long_lasting_check_log()

    def test_orientation_longlasting_streaming(self):
        fileName = "orientation_longlasting_streaming"
        sensor_type = fileName.split('_')[0].strip()
        sensor_longlasting_mark = "longlasting"
        num = 0
        self.sensorImpl.logger.info("-------test %s jitter----------------" % sensor_type)
        self.sensorImpl.sensor_log(sensor_longlasting_mark, "Fastest", sensor_type)

        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            self.sensorImpl.longlasting_20min_get()
            if self.sensorImpl.long_lasting_check_log() is False:
                num += 1
        print num
        if num > 90:
            self.sensorImpl.long_lasting_check_log()
            assert self.sensorImpl.long_lasting_check_log()

    def test_rotation_vector_longlasting_streaming(self):
        fileName = "rotation_vector_longlasting_streaming"
        sensor_type = fileName.split('_')[0].strip() + " " + fileName.split('_')[1].strip()
        print sensor_type
        sensor_longlasting_mark = "longlasting"
        num = 0
        self.sensorImpl.logger.info("-------test %s jitter----------------" % sensor_type)
        self.sensorImpl.sensor_log(sensor_longlasting_mark, "Fastest", sensor_type)

        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            self.sensorImpl.longlasting_20min_get()
            if self.sensorImpl.long_lasting_check_log() is False:
                num += 1
        print num
        if num > 90:
            self.sensorImpl.long_lasting_check_log()
            assert self.sensorImpl.long_lasting_check_log()

    def test_linear_accelerometer_longlasting_streaming(self):
        fileName = "linear_acceleration_longlasting_streaming"
        sensor_type = fileName.split('_')[0].strip() + " " + fileName.split('_')[1].strip()
        sensor_longlasting_mark = "longlasting"
        num = 0
        self.sensorImpl.logger.info("-------test %s jitter----------------" % sensor_type)
        self.sensorImpl.sensor_log(sensor_longlasting_mark, "Fastest", sensor_type)

        for loop in range(100):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/100 " % (loop + 1) + fileName)
            self.sensorImpl.longlasting_20min_get()
            if self.sensorImpl.long_lasting_check_log() is False:
                num += 1
        print num
        if num > 90:
            self.sensorImpl.long_lasting_check_log()
            assert self.sensorImpl.long_lasting_check_log()



