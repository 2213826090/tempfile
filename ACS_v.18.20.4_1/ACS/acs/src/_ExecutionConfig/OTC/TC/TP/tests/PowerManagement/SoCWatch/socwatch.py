#! /usr/bin/env python
# coding:utf-8

import unittest
import os
from testlib.em.tools import ADBTools
from testlib.em.socwatch import SoCWatch

class SoC(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.adb = ADBTools()
        self.adb.adb_root()
        self.soc = SoCWatch()
        self.soc.socwatch_setup()
        self.soc.clean_result()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.soc.clean_result()

    def test_capture_cpu_c_state_and_p_state(self):
        print "[RunTest]: %s" % self.__str__()
        self.soc.socwatch_cmd("-f cpu -f gfx -t 60 -o results/test")

    def test_capture_gpu_c_state_and_p_state(self):
        print "[RunTest]: %s" % self.__str__()
        self.soc.socwatch_cmd("-f cpu -f gfx -t 60 -o results/test")

    def test_capture_thermal_temperature(self):
        print "[RunTest]: %s" % self.__str__()
        self.soc.socwatch_cmd("-f sys -t 30 -o console")

