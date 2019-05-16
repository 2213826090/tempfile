#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal import ThermalNormal
from testlib.em.settings import DisplaySetting

class ThermalCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.table_score = []
        self.app = None
        self.thermal = ThermalNormal()
        self.thermal.adb_root()
        self.thermal.set_screen_status("on")
        self.thermal.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(ThermalCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        #self.thermal.check_adb_or_reconnect()
        if self.app:
            self.app.stop()
        self.thermal.end_thermal_capture_on_device()
        super(ThermalCheck, self).tearDown()

    def check_thermal(self):
        count = self.thermal.get_zones_count()
        temps = self.thermal.get_zones_temps(range(count))
        print temps

    def test_thermal_check_30m_antutu(self):
        """
        Thermal check Antutu
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Antutu4
        self.app = Antutu4()
        self.app.install()
        timeout = 100
        stop_time = time.time() + timeout
        cycle = 1
        while time.time() < stop_time:
            if self.app.get_score() or not self.app.check_running():
                print "Cycle:", cycle
                self.app.run_test()
                cycle += 1
            time.sleep(20)
        self.check_thermal()

    def test_thermal_check_30m_glbenchmark(self):
        """
        Thermal check GLbenchmark
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import GLBenchmark
        self.app = GLBenchmark()
        self.app.install()
        timeout = 100
        stop_time = time.time() + timeout
        cycle = 1
        while time.time() < stop_time:
            if self.app.get_score() or not self.app.check_running():
                print "Cycle:", cycle
                self.app.select_all_tests()
                self.app.run_test()
                cycle += 1
            time.sleep(20)
        self.check_thermal()

    def test_thermal_30min_3Dgame(self):
        """
        Thermal check 3D game 30 minutes
        """
        print "[RunTest]: %s" % self.__str__()
        self.app_package = "com.imangi.templerun2"
        self.thermal.install_artifactory_app("templerun", self.app_package)
        self.thermal.launch_templerun()
        time.sleep(60)
        self.thermal.play_templerun()
        self.check_thermal(name = None, timeout = 1800)

    def test_thermal_30min_idle(self):
        """
        Thermal check Idle 30 minutes
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.settings import WifiSetting
        WifiSetting().switch_wifi("ON")
        time.sleep(1800)
        self.check_thermal()

    def test_thermal_30min_play_HD_video(self):
        """
        Thermal check play HD video for 30 minutes
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import VideoPlayer
        video_file = self.thermal.push_artifactory_resource("hd_video", "/mnt/sdcard/Movies")
        self.app = VideoPlayer()
        self.app.install()
        timeout = 1800
        n = 3
        for i in range(n):
        self.app.play_local_file(video_file)
            time.sleep(timeout / n)
        self.check_thermal()

    def test_thermal_1h_record_video(self):
        """
        Thermal check record video for an hour
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Camera
        self.app = Camera()
        self.app.grant_permissions()
        self.app.launch("Video")
        self.app.click_shutter_button("Video")
        time.sleep(3600)

    def test_thermal_30min_download_LTE(self):
        """
        Thermal check lte network downloading for 30 minutes
        """
        url = self.thermal.get_config_value("thermal", "download_url")
        self.thermal.install_em_tools()
        self.thermal.download_file_by_tool(url)
        self.check_thermal(name = None, timeout = 1800)

    def test_thermal_30min_DCP_charge(self):
        """
        Thermal check DCP charge for 30 minutes
        """
        print "[RunTest]: %s" % self.__str__()
        self.check_thermal(name = "DCP", timeout = 1800)

    def keep_running_web_sunspider(self, timeout, url):
        stop_time = time.time() + timeout
        cycle = 1
        while time.time() < stop_time:
            print "Cycle:", cycle
            self.thermal.close_chrome_tabs()
            self.thermal.chrome_open_url(url)
            self.thermal.disconnect_adb()
            time.sleep(45)
            self.thermal.connect_adb()
            cycle += 1

    def test_thermal_30min_web_sunspider(self):
        """
        Thermal check browsing web sunspider for 30 minutes
        """
        print "[RunTest]: %s" % self.__str__()
        url = self.thermal.get_config_value("thermal", "web_sunspider")
        self.check_thermal(name = "web_sunspider", timeout = 180, data = url)

