#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal_impl import ThermalImpl

class ThermalDaemonCheckRobocop(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.apk_package = "com.glu.robocop"
        self.thermal = ThermalImpl()
        self.thermal.adb_root()
        self.thermal.set_screen_status("on")
        self.thermal.unlock_screen()
        self.thermal.setSleepMode("30 minutes")
        super(ThermalDaemonCheckRobocop, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        time.sleep(2)
        self.thermal.stop_app(self.apk_package)
        self.thermal.switch_airplane("OFF")
        super(ThermalDaemonCheckRobocop, self).tearDown()

    def test_thermal_daemon_check_robocop(self):
        """
        Thermal daemon check Robocop
        """
        print "[RunTest]: %s" % self.__str__()
        self.thermal.install_artifactory_app("robocop", self.apk_package)
        self.thermal.push_artifactory_resource("robocop_cache", "/mnt/sdcard/Android/obb/com.glu.robocop")
        self.thermal.clear_app_data(self.apk_package)
        self.thermal.switch_airplane("ON")

        logcat_begin = self.thermal.get_logcat_format_time()
        displayWidth, displayHeight = self.thermal.get_display_info()
        self.thermal.launch_robocop()
        time.sleep(60)
        self.thermal.start_play_robocop(displayWidth, displayHeight)
        for i in range(50):
            print "Cycle number: %d" % (i + 1)
            if self.thermal.check_thermal_zone_in_logcat(logcat_begin):
                return
            time.sleep(60)
        assert False, "Check THERMALD in logcat failed"

