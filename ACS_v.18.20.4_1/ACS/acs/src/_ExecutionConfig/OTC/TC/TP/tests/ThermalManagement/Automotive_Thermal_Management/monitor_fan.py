#! /usr/bin/env python
# coding:utf-8

import time
import unittest
from testlib.em.thermal import Thermal

class MonitorFan(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = Thermal()
        self.thermal.adb_root()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_monitor_fan_status_ioc_uart_log(self):
        print "[RunTest]: %s" % self.__str__()
        board_temp, ambient_temp, fan_percent = self.thermal.get_fan_status()
        print "board_temp:", board_temp
        print "ambient_temp:", ambient_temp
        print "fan_percent:", fan_percent

    def check_fan_status(self, retry = 10):
        for _ in range(retry):
            cpu_temp = self.thermal.ivi_get_cpu_temp() / 1000
            board_temp, ambient_temp, fan_real = self.thermal.get_fan_status()
            print "cpu_temp:", cpu_temp
            print "board_temp:", board_temp
            print "ambient_temp:", ambient_temp
            print "fan_real:", fan_real
            status = self.thermal.map_fan_percent(cpu_temp, board_temp, ambient_temp, fan_real)
            if status:
                return True
            time.sleep(3)
        else:
            return False

    def test_fan_max_temp_of_two_base_board_thermistors_less_than_30(self):
        print "[RunTest]: %s" % self.__str__()
        board_temp, ambient_temp, _ = self.thermal.get_fan_status()
        temp = max(board_temp, ambient_temp)
        assert temp < 30
        assert self.check_fan_status(2)

