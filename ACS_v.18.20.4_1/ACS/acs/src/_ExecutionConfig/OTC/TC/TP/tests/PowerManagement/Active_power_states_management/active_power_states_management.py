#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.tools import UIBase
from testlib.em.constants_def import *

class ActivePowerStatesManagement(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        super(ActivePowerStatesManagement, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.app.stop()
        super(ActivePowerStatesManagement, self).tearDown()

    def test_active_power_states_management_cpu_stats(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import CPU_Stats
        self.app = CPU_Stats()
        self.app.install()
        self.app.reject_network_access()
        self.app.launch()
        freq = self.app.get_cpu_freq()
        for i in range(1, 20):
            print "[Info]--- Cycle", i
            time.sleep(3)
            freq2 = self.app.get_cpu_freq()
            if freq != freq2:
                break
        else:
            assert False, "CPU freq not change"

    def test_active_power_states_management_cpu_monitor(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import CPU_Monitor
        self.app = CPU_Monitor()
        self.app.install()
        self.app.reject_network_access()
        self.app.launch()
        percent = self.app.get_cpu_freq_percent()
        for i in range(1, 20):
            print "[Info]--- Cycle", i
            time.sleep(3)
            percent2 = self.app.get_cpu_freq_percent()
            if percent != percent2:
                break
        else:
            assert False

    def test_active_power_states_management_androtics(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Androtics
        self.app = Androtics()
        self.app.install()
        self.app.reject_network_access()
        self.app.clear_data()
        self.app.launch(filed = "CPU")
        freq_list1 = self.app.get_cpu_freq()
        for i in range(1, 20):
            print "[Info]--- Cycle", i
            time.sleep(3)
            freq_list2 = self.app.get_cpu_freq()
            if freq_list1 != freq_list2:
                break
        else:
            assert False, "CPU freq not change"

