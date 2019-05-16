#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.apps import GLBenchmark

class Monitor_GPU_Temp_Freq_GLBenchmark(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.glb = GLBenchmark()
        self.glb.adb_root()
        self.glb.set_screen_status("on")
        self.glb.unlock_screen()
        self.glb.testDevice.close_background_apps()
        super(Monitor_GPU_Temp_Freq_GLBenchmark, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.glb.testDevice.close_background_apps()
        super(Monitor_GPU_Temp_Freq_GLBenchmark, self).tearDown()

    def test_monitor_gpu_temp_freq_glbenchmark(self):
        print "[RunTest]: %s" % self.__str__()
        self.glb.install()
        freq = self.glb.get_gpu_freq()
        print "[info]--- freq:", freq
        self.glb.clear_data()
        self.glb.launch()
        self.glb.select_offscreen_etc1_test()
        self.glb.run_tests()
        for i in range(10):
            time.sleep(10)
            freq1 = self.glb.get_gpu_freq()
            print "[info]--- freq:", freq1
            if freq != freq1:
                break
        else:
            assert False

