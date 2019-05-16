#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.constants_def import *
from testlib.em.s0i3 import S0i3

class IVI_S0i3(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        super(IVI_S0i3, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(IVI_S0i3, self).tearDown()

    def test_adb_over_usb_work_after_resume(self):
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3

    def test_s0i3_by_ignition_off(self):
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3

    def test_s0i3_with_usb_cable_connected(self):
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3

    def test_system_resume_from_s0i3_by_ignition(self):
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3

    def test_boot_up_from_s5_by_ignition(self):
        from testlib.em.power import get_power_obj
        get_power_obj().reboot()
        self.s0i3.adb_root()
        s3 = self.s0i3.get_s0i3_suspend_stat()
        assert s3 == 0

    def test_check_cpu_freq_well_after_resume(self):
        cpus = self.s0i3.get_cpus()
        freqs1 = self.s0i3.ivi_get_cpus_freq(cpus)
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3
        for _ in range(10):
            time.sleep(2)
            freqs2 = self.s0i3.ivi_get_cpus_freq(cpus)
            if freqs1 != freqs2:
                return
        assert False

    def test_check_gpu_freq_well_after_resume(self):
        freq1 = self.s0i3.get_gpu_freq()
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3
        from testlib.em.apps import GLBenchmark
        glb = GLBenchmark()
        glb.install()
        glb.clear_data()
        glb.launch()
        glb.select_all_tests()
        glb.run_tests()
        for _ in range(10):
            time.sleep(2)
            freq2 = self.s0i3.get_gpu_freq()
            if freq1 != freq2:
                return
        assert False

    def test_s0i3_read_sdcard_after_resume(self):
        path = self.s0i3.get_external_disk_mount_path("SD")
        assert path != ""
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3
        path = self.s0i3.get_external_disk_mount_path("SD")
        assert path != ""

    def test_get_location_after_resume(self):
        from testlib.em.settings import WifiSetting
        from testlib.em.apps import GeoLocation
        from testlib.em.settings import LocationSetting
        from testlib.em.tools import get_geo_location
        ls = LocationSetting()
        ls.switch_GPS("ON")
        ls.set_mode("High accuracy")
        actual_location = get_geo_location()
        geo = GeoLocation()
        geo.install()
        geo.grant_permissions()
        wifi = WifiSetting()
        wifi.connect_wifi_by_conf("wifisetting")
        assert wifi.check_wifi_connect()
        enter_s3 = self.s0i3.suspend_resume(retry = 2)
        assert enter_s3
        location = geo.get_latitude_longitude()
        if location:
            diff_lati = abs(actual_location[0] - location[0])
            diff_long = abs(actual_location[1] - location[1])
            assert diff_lati < 0.1
            assert diff_long < 0.1
        else:
            assert False, "Can not get location info"

