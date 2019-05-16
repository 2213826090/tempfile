#! /usr/bin/env python
# coding:utf-8

import unittest
from testlib.em.thermal import Thermal
from testlib.util.process import shell_command
from testlib.em.tools import read_file_by_line

class BootUp(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.data_files = shell_command("ls Thermal_data_*.csv")[1]

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def check_boot_up_status(self, temp1, temp2):
        for data_file in self.data_files:
            data_file = data_file.strip()
            lines = read_file_by_line(data_file)
            lines.next()
            for line in lines:
                line_arr = line.split(",")
                tcpu, tskn, tamb, fan, tp, tboot = tuple([(item) for item in line_arr[0: 6]])
                if temp1 <= int(tamb) <= temp2:
                    if tboot == "boot_up_completed":
                        print "tcpu:", tcpu
                        print "tskn:", tskn
                        print "tamb:", tamb
                        print "fan:", fan
                        print "boot status:", tboot
                        return True
        else:
            return False

    def test_system_boot_up_aos_with_0_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_boot_up_status(-2, 2)

    def test_system_boot_up_aos_with_20_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_boot_up_status(18, 22)

    def test_system_boot_up_aos_with_40_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_boot_up_status(38, 42)

    def test_system_boot_up_aos_with_55_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_boot_up_status(53, 57)

    def test_system_boot_up_aos_with_75_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.check_boot_up_status(73, 77)

