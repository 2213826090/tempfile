#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.tools import UIBase
from testlib.em.constants_def import *

class ActiveIdle(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.uibase = UIBase()
        self.uibase.adb_root()
        self.uibase.unlock_screen()
        self.uibase.set_screen_orientation('n')
        self.uibase.testDevice.close_background_apps()
        super(ActiveIdle, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.uibase.testDevice.close_background_apps()
        super(ActiveIdle, self).tearDown()

    def test_active_idle_running_antutu(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Antutu4 as Antutu
        antutu = Antutu()
        product_name = self.uibase.get_product()
        if "androidia" in product_name or "celadon" in product_name:
            max_freq = antutu.ivi_get_cpu_freq(0, "scaling_max_freq")
        else:
            max_freq = antutu.ivi_get_cpu_freq(0, "cpuinfo_max_freq")
        print max_freq
        target_freq = max_freq * 0.9
        antutu.install()
        antutu.reject_network_access()
        antutu.run_test()
        for i in range(1, 100):
            print "[Info]--- Cycle", i
            time.sleep(3)
            if "androidia" in product_name or "celadon" in product_name:
                cur_freq = antutu.ivi_get_cpu_freq(0, "scaling_cur_freq")
            else:
                cur_freq = antutu.ivi_get_cpu_freq(0, "cpuinfo_cur_freq")
            print cur_freq
            if cur_freq >= target_freq:
                antutu.stop()
                break
        else:
            assert False, "CPU freq not reach Max"

    def test_active_idle_running_glbenchmark(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import GLBenchmark
        glb = GLBenchmark()
        overclocked_freq_Mhz = glb.get_gpu_freq("Max overclocked frequency")
        print "[Info] Max overclocked frequency: {}".format(overclocked_freq_Mhz)
        glb.install()
        glb.launch()
        glb.select_all_tests()
        glb.run_tests()
        for i in range(1, 100):
            print "[Info]--- Cycle", i
            time.sleep(3)
            current_freq_Mhz = glb.get_gpu_freq("Current freq")
            print "[Info] Current freq: {}".format(current_freq_Mhz)
            if overclocked_freq_Mhz == current_freq_Mhz:
                break
        else:
            assert False, "GPU freq not reach Max"

    def test_active_idle_monitor_current_freq_come_back(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import GLBenchmark
        glb = GLBenchmark()
        overclocked_freq_Mhz = glb.get_gpu_freq("Max overclocked frequency")
        glb.install()
        glb.launch()
        glb.select_all_tests()
        glb.run_tests()
        for i in range(1, 100):
            print "[Info]--- Cycle", i
            time.sleep(3)
            current_freq_Mhz = glb.get_gpu_freq("Current freq")
            if overclocked_freq_Mhz == current_freq_Mhz:
                break
        else:
            assert False, "GPU freq not reach Max"

        glb.stop()
        for i in range(1, 20):
            print "[Info]--- Cycle", i
            time.sleep(3)
            current_freq_Mhz = glb.get_gpu_freq("Current freq")
            if overclocked_freq_Mhz > current_freq_Mhz:
                break
        else:
            assert False, "GPU freq not drop down"

    def active_idle_monitor_sleep(self, sleep_time):
        from testlib.em.s0i3 import S0i3
        max_freq = self.uibase.ivi_get_cpu_freq(0, "cpuinfo_max_freq")
        S0i3().suspend_resume(sleep_time = sleep_time, retry = 1)
        cpus = self.uibase.get_cpus()
        check_num = 10
        freq_sum = 0
        for i in range(check_num):
            freqs = self.uibase.ivi_get_cpus_freq(cpus)
            print freqs
            freq_sum += sum(freqs)
            time.sleep(3)
        freq_rate = float(freq_sum) / (max_freq * len(cpus) * check_num)
        print "CPU usage:", freq_rate
        if len(cpus) < 4:
            high_rate = 0.9
        else:
            high_rate = 0.7
        assert freq_rate <= high_rate, "CPU freq too high when idle"

    def active_idle_monitor_sleep_for_AIA(self, sleep_time):
        from testlib.em.settings import DisplaySetting
        max_freq = self.uibase.ivi_get_cpu_freq(0, "scaling_max_freq")
        DisplaySetting().set_sleep_mode(sleep_time)
        if sleep_time == "1 minute":
            time.sleep(60)
        elif sleep_time == "5 minutes":
            time.sleep(300)
        elif sleep_time == "10 minutes":
            time.sleep(600)
        elif sleep_time == "30 minutes":
            time.sleep(1800)
        else:
            time.sleep(60)
        cpus = self.uibase.get_cpus()
        check_num = 10
        freq_sum = 0
        for i in range(check_num):
            freqs = self.uibase.ivi_get_cpus_freq(cpus, "scaling_cur_freq")
            print freqs
            freq_sum += sum(freqs)
            time.sleep(3)
        freq_rate = float(freq_sum) / (max_freq * len(cpus) * check_num)
        print "CPU usage:", freq_rate
        if len(cpus) < 4:
            high_rate = 0.9
        else:
            high_rate = 0.7
        assert freq_rate <= high_rate, "CPU freq too high when idle"

    def test_active_idle_monitor_sleep_5min_idle(self):
        print "[RunTest]: %s" % self.__str__()
        product_name = self.uibase.get_product()
        if "androidia" in product_name or "celadon" in product_name:
            self.active_idle_monitor_sleep_for_AIA("5 minutes")
        else:
            self.active_idle_monitor_sleep(300)

    def test_active_idle_monitor_sleep_10min_idle(self):
        print "[RunTest]: %s" % self.__str__()
        product_name = self.uibase.get_product()
        if "androidia" in product_name or "celadon" in product_name:
            self.active_idle_monitor_sleep_for_AIA("10 minutes")
        else:
            self.active_idle_monitor_sleep(600)

