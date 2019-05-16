#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class DCP_low_Battery(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.adb_root()
        assert self.emImpl.get_battery_level() <= 15
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        super(DCP_low_Battery, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.stop_focused_activity()
        super(DCP_low_Battery, self).tearDown()

    def test_DCP_playing_music_low_battery(self):
        self.emImpl.grant_music_app_permissions()
        audio_file = self.emImpl.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.play_media_file("audio/*", "file://" + audio_file)
        time.sleep(5)
        self.emImpl.check_cdp_dcp_charger("USB_DCP")

    def test_DCP_playing_video_low_battery(self):
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.play_media_file("video/*", "file://" + video_file)
        time.sleep(5)
        self.emImpl.check_cdp_dcp_charger("USB_DCP")

    def test_DCP_record_camera_low_battery(self):
        self.emImpl.grant_permissions_for_camera_app()
        self.emImpl.launch_camera("Video")
        self.emImpl.click_shutter_button("Video")
        cycles = 25
        for i in range(1, 1 + cycles):
            print "[info]--- Cycle: %s/%s" % (i, cycles)
            self.emImpl.check_cdp_dcp_charger("USB_DCP")

    def test_DCP_surfing_internet_low_battery(self):
        ssid, passwd = self.emImpl.read_wifi_conf()
        self.emImpl.connect_wifi(ssid, passwd)
        self.webpage = "http://" + self.emImpl.get_server_ip() + self.emImpl.get_config_value("webpage", "simple_page")
        self.emImpl.close_chrome_tabs()
        self.emImpl.chrome_open_url(self.webpage)
        time.sleep(5)
        self.emImpl.check_cdp_dcp_charger("USB_DCP")

    def test_DCP_taking_image_low_battery(self):
        self.emImpl.grant_permissions_for_camera_app()
        self.emImpl.launch_camera("Camera")
        self.emImpl.click_shutter_button("Camera")
        self.emImpl.check_cdp_dcp_charger("USB_DCP")

