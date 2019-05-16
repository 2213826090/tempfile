#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.tools import UIBase
from testlib.em.constants_def import *

class MonitorCPU(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.uibase = UIBase()
        self.uibase.adb_root()
        self.uibase.set_screen_orientation('n')
        self.uibase.testDevice.close_background_apps()
        super(MonitorCPU, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MonitorCPU, self).tearDown()

    def test_cpu_freq_idle(self):
        product_name = self.uibase.get_product()
        if "androidia" in product_name or "celadon" in product_name:
            max_freq = self.uibase.ivi_get_cpu_freq(0, "scaling_max_freq")
        else:
            max_freq = self.uibase.ivi_get_cpu_freq(0, "cpuinfo_max_freq")
        high_feq = max_freq * 0.7
        for i in range(10):
            if "androidia" in product_name or "celadon" in product_name:
                cur_freq = self.uibase.ivi_get_cpu_freq(0, "scaling_cur_freq")
            else:
                cur_freq = self.uibase.ivi_get_cpu_freq(0, "cpuinfo_cur_freq")
            if cur_freq < high_feq:
                break
            time.sleep(3)
        else:
            assert False, "CPU freq too high"

