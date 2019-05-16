#! /usr/bin/env python
# coding:utf-8

import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.sensor.sensor_impl import SensorImpl


class TemperatureDataSuspendAndResume10Times(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.sensorImpl = SensorImpl()
        super(TemperatureDataSuspendAndResume10Times, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.sensorImpl.boot_up_completed_skip_boot_ui()
        super(TemperatureDataSuspendAndResume10Times, self).tearDown()

    def test_temperature_data_after_turn_screen_on_off_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        #get method self name
        fileName = sys._getframe().f_code.co_name
        for loop in range(10):
            self.sensorImpl.logger.info("")
            self.sensorImpl.logger.info("Loop %d/10 " % (loop + 1) + fileName)
            self.sensorImpl.check_enter_s0i3_state_for_ivi()
            self.sensorImpl.check_temperature_sensor_data()
