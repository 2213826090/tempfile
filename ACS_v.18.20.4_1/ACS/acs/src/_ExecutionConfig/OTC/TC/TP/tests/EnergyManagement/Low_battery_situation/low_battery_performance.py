#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class LowBatteryPerformance(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.adb_root()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.set_sleep_mode("30 minutes")
        self.enable_wifi_adb()
        super(LowBatteryPerformance, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        super(LowBatteryPerformance, self).tearDown()

    def enable_wifi_adb(self):
        ssid, passwd = self.emImpl.read_wifi_conf()
        self.emImpl.connect_wifi(ssid, passwd)
        serial_wifi = self.emImpl.enable_wifi_adb()
        self.emImpl_wifi = EMImpl(serial_wifi)

    def prepare_battery_level(self, target_level):
        actual_level = self.emImpl.get_battery_level()
        if actual_level < target_level - 1:
            self.emImpl.set_screen_status("off")
            self.emImpl.enable_dcp_charging(2, 2)
            cycles = 20
            for i in range(1, 1 + cycles):
                wait_time = 60 * (target_level - actual_level)
                print "Cycle: %d/%d, wait %ds" % (i, cycles, wait_time)
                time.sleep(wait_time)
                actual_level = self.emImpl_wifi.get_battery_level()
                if actual_level >= target_level:
                    break
            else:
                assert False
            self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
            self.emImpl.set_screen_status("on")
            self.emImpl.unlock_screen()

        elif actual_level > target_level + 1:
            video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
            self.emImpl.play_media_file("video/*", "file://" + video_file)
            time.sleep(5)
            self.emImpl.set_three_way_cutter_usb(0)
            cycles = 20
            for i in range(1, 1 + cycles):
                wait_time = 60 * (actual_level - target_level)
                print "Cycle: %d/%d, wait %ds" % (i, cycles, wait_time)
                time.sleep(wait_time)
                actual_level = self.emImpl_wifi.get_battery_level()
                if actual_level <= target_level:
                    break
            else:
                assert False
            self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
            self.emImpl.stop_focused_activity()

    def check_low_battery_warning(self, low_level):
        cycles = 20
        for i in range(1, 1 + cycles):
            print "Check low battery warning: %d/%d" % (i, cycles)
            time.sleep(60)
            if self.emImpl_wifi.get_battery_level() <= low_level:
                break
        else:
            assert False, "battery level higher than 15%"
        assert self.emImpl_wifi.check_lowbattery_notification()

    def test_low_battery_warning_15_play_video(self):
        """
        Test low battery warning 15% play video
        """
        print "[RunTest]: %s" % self.__str__()
        self.prepare_battery_level(17)
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.emImpl.play_media_file("video/*", "file://" + video_file)
        time.sleep(5)
        self.emImpl.set_three_way_cutter_usb(0)
        self.check_low_battery_warning(15)
        self.emImpl.stop_focused_activity()

    def test_low_battery_warning_15(self):
        """
        Test low battery warning 15
        """
        print "[RunTest]: %s" % self.__str__()
        self.prepare_battery_level(17)
        self.emImpl.set_three_way_cutter_usb(0)
        self.check_low_battery_warning(15)

    def test_low_battery_warning_5(self):
        """
        Test low battery warning 5
        """
        print "[RunTest]: %s" % self.__str__()
        self.prepare_battery_level(7)
        self.emImpl.set_three_way_cutter_usb(0)
        time.sleep(5)
        self.emImpl_wifi.clean_notification()
        self.check_low_battery_warning(5)

    def test_battery_20_discharge_util_shutdown(self):
        print "[RunTest]: %s" % self.__str__()
        self.prepare_battery_level(20)
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.emImpl.play_media_file("video/*", "file://" + video_file)
        time.sleep(5)
        self.emImpl.set_three_way_cutter_usb(0)
        for _ in range(20):
            if not self.emImpl_wifi.get_state():
                break
            try:
                level = self.emImpl_wifi.get_battery_level()
            except:
                break
            wait_time = 60 * level + 5
            print "Wait %ds" % wait_time
            time.sleep(wait_time)
        else:
            assert False
        self.emImpl.enable_dcp_charging(2, 2)
        time.sleep(600)
        self.emImpl.enable_sdp_charging(2, 2)
        time.sleep(60)
        self.emImpl.boot_up_device()
        self.emImpl.adb_root()

