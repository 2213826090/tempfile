#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.apps import Antutu4
#from testlib.em.constants_def import *

class Monitor_CPU_Temp_Freq_Antutu(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.antutu = Antutu4()
        self.antutu.adb_root()
        self.antutu.set_screen_status("on")
        self.antutu.unlock_screen()
        self.antutu.testDevice.close_background_apps()
        super(Monitor_CPU_Temp_Freq_Antutu, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.antutu.stop()
        self.antutu.testDevice.close_background_apps()
        super(Monitor_CPU_Temp_Freq_Antutu, self).tearDown()

    def test_monitor_cpu_temp_freq_antutu(self):
        print "[RunTest]: %s" % self.__str__()
        self.antutu.install()
        self.antutu.reject_network_access()
        temp = self.antutu.ivi_get_cpu_temp()
        freq_max = self.antutu.ivi_get_cpu_freq(freq_name = "cpuinfo_max_freq")
        freq_target = freq_max * 0.9
        temp_changed = False
        freq_reach_max = False
        self.antutu.run_test()
        for i in range(10):
            time.sleep(10)
            if not temp_changed:
                temp1 = self.antutu.ivi_get_cpu_temp()
                if temp != temp1:
                    temp_changed = True
            if not freq_reach_max:
                freq = self.antutu.ivi_get_cpu_freq()
                if freq > freq_target:
                    freq_reach_max = True
            if temp_changed and freq_reach_max:
                break

        assert temp_changed
        assert freq_reach_max

