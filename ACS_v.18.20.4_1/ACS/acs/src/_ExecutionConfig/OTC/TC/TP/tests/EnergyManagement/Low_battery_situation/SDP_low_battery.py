#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class SDP_Low_Battery(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        assert self.emImpl.get_battery_level() <= 15
        self.emImpl.adb_root()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        super(SDP_Low_Battery, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.stop_focused_activity()

    def test_SDP_playing_music_low_battery(self):
        self.emImpl.grant_music_app_permissions()
        audio_file = self.emImpl.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.emImpl.play_media_file("audio/*", "file://" + audio_file)
        time.sleep(5)
        assert self.emImpl.get_sdp_charge_status()

    def test_SDP_playing_video_low_battery(self):
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.emImpl.play_media_file("video/*", "file://" + video_file)
        time.sleep(5)
        assert self.emImpl.get_sdp_charge_status()

    def test_SDP_plug_unplug_low_battery(self):
        cycles = 10
        for i in range(1, 1 + cycles):
            print "[info]--- Cycles: %s/%s" % (i, cycles)
            self.emImpl.set_three_way_cutter_usb(0)
            time.sleep(2)
            self.emImpl.set_three_way_cutter_usb(1)
            time.sleep(5)
            assert self.emImpl.get_sdp_charge_status()

    def test_SDP_surfing_internet_low_battery(self):
        ssid, passwd = self.emImpl.read_wifi_conf()
        self.emImpl.connect_wifi(ssid, passwd)
        self.webpage = "http://" + self.emImpl.get_server_ip() + self.emImpl.get_config_value("webpage", "simple_page")
        self.emImpl.close_chrome_tabs()
        self.emImpl.chrome_open_url(self.webpage)
        time.sleep(5)
        assert self.emImpl.get_sdp_charge_status()

    def test_SDP_taking_image_low_battery(self):
        self.emImpl.grant_permissions_for_camera_app()
        self.emImpl.launch_camera()
        self.emImpl.click_shutter_button()
        time.sleep(5)
        assert self.emImpl.get_sdp_charge_status()

    def test_SDP_taking_videorecord_low_battery(self):
        self.emImpl.grant_permissions_for_camera_app()
        self.emImpl.launch_camera("Video")
        self.emImpl.click_shutter_button("Video")
        time.sleep(5)
        assert self.emImpl.get_sdp_charge_status()

