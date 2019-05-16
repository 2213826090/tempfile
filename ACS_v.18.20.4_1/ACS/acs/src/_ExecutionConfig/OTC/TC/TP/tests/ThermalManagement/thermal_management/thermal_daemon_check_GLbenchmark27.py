#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal_impl import ThermalImpl

class ThermalDaemonCheckGLbenchmark27(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.apk_package = "com.glbenchmark.glbenchmark27"
        self.thermal = ThermalImpl()
        self.thermal.install_artifactory_app("glbenchmark", self.apk_package)
        self.thermal.set_screen_status("on")
        self.thermal.unlock_screen()
        self.thermal.setSleepMode("30 minutes")
        super(ThermalDaemonCheckGLbenchmark27, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.thermal.stop_app(self.apk_package)
        super(ThermalDaemonCheckGLbenchmark27, self).tearDown()

    def run_glbenchmark_retry(self, retry):
        succeed = False
        for i in range(retry):
            self.thermal.clear_app_data(self.apk_package)
            self.thermal.launch_glbenchmark()
            self.thermal.select_glbenchmark_all_tests()
            if self.thermal.run_glbenchmark_tests():
                succeed = True
                break
        assert succeed, "Start GLbenchmark failed"

    def test_thermal_daemon_check_glbenchmark27(self):
        """
        Thermal daemon check GLbenchmark2.7
        """
        print "[RunTest]: %s" % self.__str__()

        logcat_begin = self.thermal.get_logcat_format_time()
        self.run_glbenchmark_retry(3)
        for i in range(50):
            print "Cycle number: %d" % (i + 1)
            time.sleep(60)
            if self.thermal.check_thermal_zone_in_logcat(logcat_begin):
                return
            if not self.thermal.check_glbenchmark_running():
                self.run_glbenchmark_retry(3)
        assert False, "Check ThermalZone in logcat failed"

