#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal import Thermal
#from testlib.util.common import g_common_obj
#from testlib.em.constants_def import *

class Monitor_Board_Temp(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = Thermal()
        self.thermal.adb_root()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_monitor_board_temp_sysfs(self):
        print "[RunTest]: %s" % self.__str__()
        temp = self.thermal.ivi_get_board_temp()
        print "[info]--- Current CPU temp:", temp

