#! /usr/bin/env python
# coding:utf-8

import time
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.em.apps import GLBenchmark
from testlib.em.nohup_process import NohupProcess
from testlib.em.tools import get_tmp_dir, read_file_by_line

class Balancer(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.glb = GLBenchmark()
        self.glb.adb_root()
        self.glb.testDevice.close_background_apps()
        self.tmp_dir = get_tmp_dir()
        self.device_script = "cat_cpu_gpu.sh"
        self.device_exec_dir = "/data/local/tmp/"
        self.csv_result = "out_" + time.strftime("%y%m%d_%H%M%S") + ".csv"
        #self.csv_result = "out_171113_165905.csv"
        self.device_result_path = os.path.join(self.device_exec_dir, self.csv_result)
        self.local_result_path = os.path.join(self.tmp_dir, self.csv_result)
        self.np = NohupProcess(self.glb.testDevice, self.device_script, self.device_exec_dir, self.csv_result)
        #super(Balancer, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.np.stop()
        self.glb.stop()
        #super(Balancer, self).tearDown()

    def get_csv_result(self):
        self.np.start()
        time.sleep(60)
        self.np.stop()
        self.glb.testDevice.pull_file(self.local_result_path, self.device_result_path)

    def test_cpu_gpu_power_budget_balancer_idle(self):
        self.get_csv_result()
        lines = read_file_by_line(self.local_result_path)
        col_num = len(lines.next().split(","))
        line_total = 0
        line_issue = 0
        for line in lines:
            line_list = line.split(",")
            if len(line_list) == col_num:
                line_total += 1
                if "100" != line_list[-1]:
                    line_issue += 1
        print line_issue, line_total
        assert line_issue < line_total * 0.1

    def test_cpu_gpu_power_budget_balancer_glbenchmark(self):
        self.glb.install()
        self.glb.launch()
        self.glb.select_all_tests()
        #self.glb.select_offscreen_etc1_test()
        self.glb.run_tests()
        self.get_csv_result()
        lines = read_file_by_line(self.local_result_path)
        col_num = len(lines.next().split(","))
        line_total = 0
        line_issue = 0
        for line in lines:
            line_list = line.split(",")
            if len(line_list) == col_num:
                line_total += 1
                gpu_act= int(line_list[-2])
                gpu_req= int(line_list[-1])
                if abs(gpu_act - gpu_req) >= 100:
                    line_issue += 1
        print line_issue, line_total
        assert line_issue < line_total * 0.1

