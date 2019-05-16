#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal_impl import ThermalImpl

class ThermalDaemonCheckPlayHDVideo(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = ThermalImpl()
        self.thermal.set_screen_status("on")
        self.thermal.unlock_screen()
        self.thermal.setSleepMode("30 minutes")
        super(ThermalDaemonCheckPlayHDVideo, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.thermal.stop_app(self.thermal.photos_package)
        super(ThermalDaemonCheckPlayHDVideo, self).tearDown()

    def test_thermal_daemon_check_play_HD_video(self):
        """
        Thermal daemon check play HD video
        """
        print "[RunTest]: %s" % self.__str__()

        video_file = self.thermal.push_artifactory_resource("hd_video", "/mnt/sdcard/Movies")
        self.thermal.play_media_file("video/*", "file://" + video_file)
        logcat_begin = self.thermal.get_logcat_format_time()
        for i in range(50):
            print "Cycle number: %d" % (i + 1)
            if self.thermal.check_thermal_zone_in_logcat(logcat_begin):
                return
            time.sleep(60)
        assert False, "Check THERMALD in logcat failed"

