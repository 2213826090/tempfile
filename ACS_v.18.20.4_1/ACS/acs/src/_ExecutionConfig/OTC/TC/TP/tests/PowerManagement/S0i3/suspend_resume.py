#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0i3 import S0i3
from testlib.em.constants_def import *
from testlib.em.settings import BTSetting, WifiSetting
from testlib.em.apps import Camera, CleanMusic

class SuspendResume(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        self.s0i3.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.bt.host_teardown()
        self.bt.switch_bt("OFF")
        self.music.stop()
        self.camera.stop()
        super(SuspendResume, self).tearDown()

    def test_suspend_resume_50_times(self):
        cycles = 50
        self.bt = BTSetting()
        self.wifi = WifiSetting()
        self.camera = Camera()
        self.music = CleanMusic()
        self.camera.grant_permissions()
        self.music.install()
        self.music.grant_permissions()
        music_file = self.music.push_artifactory_resource("long_music", "/mnt/sdcard/Music")

        alias = self.bt.get_host_alias()
        self.bt.host_setup()
        self.bt.switch_bt("ON")
        self.bt.bt_search(alias)
        self.bt.bt_pair(alias)
        self.wifi.connect_wifi_by_conf("wifisetting")
        assert self.wifi.check_wifi_connect()
        for i in range(1, 1 + cycles):
            print "Cycle: %s/%s" % (i, cycles)
            assert self.s0i3.suspend_resume(sleep_time = 120, retry = 1), "Not enter S0i3"
            time.sleep(100)
            assert self.bt.get_bt_status(), "BT is OFF"
            assert self.bt.get_paired_status(alias), "BT is not paired"
            assert self.wifi.check_wifi_connect(), "WiFi is not connected"
            if self.s0i3.get_product() == BXT_M:
                self.camera.launch()
                self.camera.click_shutter_button()
            self.music.play_audio("file://" + music_file)
            time.sleep(200)

