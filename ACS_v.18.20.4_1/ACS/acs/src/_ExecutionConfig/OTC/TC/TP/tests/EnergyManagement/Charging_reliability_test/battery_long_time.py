#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.em_impl import EMImpl

timeout = 12 * 3600

class BatteryLongTime(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.tmp_dir = self.emImpl.get_tmp_dir()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        self.emImpl.set_brightness_level(255)
        g_common_obj.close_background_apps()
        self.emImpl.install_artifactory_app("battery_log", "kr.hwangti.batterylog")
        self.emImpl.reset_battery_log()
        super(BatteryLongTime, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def consume_power(self, target_level, check_interval, mode = None):
        for i in range(timeout / check_interval):
            if self.emImpl.get_battery_level() <= target_level:
                break
            if mode == "audio":
                self.emImpl.keep_playing_audio()
            self.emImpl.set_three_way_cutter_usb(0)
            time.sleep(check_interval)
            self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)

    def check_battery_drop_smoothly(self):
        csv_path = self.emImpl.save_battery_log_csv(self.tmp_dir)
        self.emImpl.analyze_battery_log_csv(csv_path)

    def test_battery_long_time_play_3D_game_percentage(self):
        print "[RunTest]: %s" % self.__str__()
        temple_run_package = "com.imangi.templerun2"
        self.emImpl.install_artifactory_app("templerun", temple_run_package)
        # play 3D game
        self.emImpl.start_battery_log_service()
        time.sleep(5)
        self.emImpl.launch_templerun()
        time.sleep(60)
        self.emImpl.play_templerun()
        # check battery level no jump
        self.consume_power(20, 120)
        self.check_battery_drop_smoothly()

    def test_battery_long_time_play_music_percentage(self):
        self.emImpl.grant_music_app_permissions()
        music_file = self.emImpl.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.emImpl.start_battery_log_service()
        time.sleep(5)
        self.emImpl.play_media_file("audio/*", "file://" + music_file)
        self.consume_power(15, 300, mode = "audio")
        self.check_battery_drop_smoothly()

    def test_battery_long_time_play_video_percentage(self):
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.emImpl.start_battery_log_service()
        time.sleep(5)
        self.emImpl.play_media_file("video/*", "file://" + video_file)
        self.consume_power(20, 120, mode = "video")
        self.check_battery_drop_smoothly()

    def test_battery_idle_overnight_with_all_radio_connected(self):
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.switch_wifi("ON")
        self.emImpl.switch_bt("ON")
        self.emImpl.switch_GPS("ON")
        self.emImpl.start_battery_log_service()
        time.sleep(5)
        self.consume_power(15, 600)
        self.check_battery_drop_smoothly()

