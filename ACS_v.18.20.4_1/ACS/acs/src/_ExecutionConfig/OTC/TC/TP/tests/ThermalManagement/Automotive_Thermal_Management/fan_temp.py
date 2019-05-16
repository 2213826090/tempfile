#! /usr/bin/env python
# coding:utf-8

import unittest
from testlib.em.thermal import Thermal
from testlib.util.process import shell_command
from testlib.em.tools import read_file_by_line

class Fan(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.data_files = shell_command("ls Thermal_data_*.csv")[1]
        self.thermal = Thermal()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def check_fan_status(self, temp1, temp2):
        for data_file in self.data_files:
            data_file = data_file.strip()
            lines = read_file_by_line(data_file)
            lines.next()
            for line in lines:
                line_arr = line.split(",")
                tcpu, tskn, tamb, fan = tuple([int(item) for item in line_arr[0: 4]])
                if temp1 <= tamb <= temp2:
                    status = self.thermal.map_fan_percent(tcpu, tskn, tamb, fan)
                    if status:
                        print "tcpu:", tcpu
                        print "tskn:", tskn
                        print "tamb:", tamb
                        print "fan:", fan
                        return True
        else:
            return False

    def test_fan_max_temp_of_two_base_board_thermistors_between_30_to_35_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_fan_status(30, 35)

    def test_fan_max_temp_of_two_base_board_thermistors_between_35_to_40_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_fan_status(35, 40)

    def test_fan_max_temp_of_two_base_board_thermistors_between_40_to_45_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_fan_status(40, 45)

    def test_fan_max_temp_of_two_base_board_thermistors_between_45_to_50_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_fan_status(45, 50)

    def test_fan_max_temp_of_two_base_board_thermistors_more_than_50_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_fan_status(50, 60)
        print "--------------"
        assert self.check_fan_status(65, 70)
