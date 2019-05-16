#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.power import get_power_obj
from testlib.em.settings import DisplaySetting

class PowerOnOff(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.power = get_power_obj()
        self.power.adb_root()
        self.display = DisplaySetting()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.display.set_sleep_mode("30 minutes")
        self.power.testDevice.close_background_apps()
        super(PowerOnOff, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.power.stop_focused_activity()
        super(PowerOnOff, self).tearDown()

    def common_reboot(self):
        self.power.reboot()
        time.sleep(10)
        self.power.adb_root()
        self.power.set_screen_status("on")
        self.power.unlock_screen()

    def set_mixedmode(self, status):
        from testlib.em.settings import AirplaneModeSetting, WifiSetting, \
            BTSetting, LocationSetting
        AirplaneModeSetting().switch_airplane(status)
        WifiSetting().switch_wifi(status)
        BTSetting().switch_bt(status)
        LocationSetting().switch_GPS(status)

    def test_power_option_dialog_shows(self):
        self.power.trigger_power_off_dialog()

    def test_power_on(self):
        self.common_reboot()

    def test_power_on_with_sdcard_plugged(self):
        dirname = self.power.get_external_sdcard_dirname()
        assert dirname != ""
        self.common_reboot()

    def test_power_off(self):
        self.common_reboot()

    def test_power_off_music_play(self):
        from testlib.em.apps import CleanMusic
        self.app = CleanMusic()
        self.app.install()
        self.app.grant_permissions()
        music_file = self.app.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.app.play_audio("file://" + music_file)
        time.sleep(3)
        self.common_reboot()

    def test_power_off_camera_capture_screen(self):
        from testlib.em.apps import Camera
        self.app = Camera()
        self.app.grant_permissions()
        self.app.launch()
        time.sleep(3)
        self.common_reboot()

    def test_power_off_receive_email(self):
        from testlib.em.apps import Email
        self.app = Email()
        self.app.open_email()
        self.common_reboot()

    def test_power_off_video_record(self):
        from testlib.em.apps import Camera
        self.app = Camera()
        self.app.grant_permissions()
        self.app.launch("Video")
        self.app.click_shutter_button("Video")
        self.common_reboot()

    def test_power_off_mixedmode_on(self):
        self.set_mixedmode("ON")
        self.common_reboot()
        self.set_mixedmode("OFF")

    def test_power_off_on_with_headset(self):
        self.common_reboot()

    def test_power_off_with_SDP_charger(self):
        #assert self.power.get_sdp_charge_status()
        self.common_reboot()

    def test_power_off_with_wifi_BT_GPS_NFC_off(self):
        self.set_mixedmode("OFF")
        self.common_reboot()

    def test_power_on_off_with_USB_pluged(self):
        self.common_reboot()

    def test_press_power_every_2_seconds(self):
        self.power.power_off_wait()
        time.sleep(60)
        self.power.power_on()
        for i in range(100):
            if self.power.check_boot_completed():
                self.power.adb_root()
                self.power.set_screen_status("on")
                self.power.unlock_screen()
                return
            self.power.press_power_key(2)
            time.sleep(2)
        assert False, "Boot up failed"

    def test_programs_background_longpress_reboot(self):
        from testlib.em.apps import Camera, Email, KittenCat
        camera = Camera()
        camera.grant_permissions()
        camera.launch()
        email = Email()
        email.grant_permissions()
        email.launch()
        cat = KittenCat()
        cat.grant_permissions()
        cat.install()
        cat.launch()
        time.sleep(10)

        self.power.press_power_key(15)
        time.sleep(20)
        self.power.boot_up()
        self.power.adb_root()

    def test_reboot_with_airplane_on(self):
        from testlib.em.settings import AirplaneModeSetting
        air = AirplaneModeSetting()
        air.switch_airplane("ON")
        self.common_reboot()
        air.check_airplane_status("ON")
        air.switch_airplane("OFF")

    def test_restart_device_CPU_load(self):
        from testlib.em.apps import CPUPrimeBenchmark
        app = CPUPrimeBenchmark()
        app.install()
        app.cpu_load()
        self.common_reboot()
        #self.power.reboot()

    def test_power_off_downloadingfrombrowser(self):
        from testlib.em.apps import EMToolsDownload
        from testlib.em.tools import get_server_ip, get_config_value
        from testlib.em.settings import WifiSetting
        url = "http://" + get_server_ip() + get_config_value("webpage", "big_file")
        # connect wifi
        wifi = WifiSetting()
        wifi.connect_wifi_by_conf("wifi_adb")
        em = EMToolsDownload()
        em.install()
        em.grant_permissions()
        em.download_file(url)
        time.sleep(5)
        assert em.get_download_file_status()
        self.common_reboot()

    def test_coldreboot_many_times(self):
        times = 50
        for i in range(1, times + 1):
            print "[info]---Cycle: %d/%d" % (i, times)
            self.common_reboot()

    def test_power_off_with_hub(self):
        self.common_reboot()

    def test_power_off_with_CDP_charger(self):
        self.power.power_off_timer(10)
        self.power.enable_cdp_charging(1, 0)
        time.sleep(30)
        self.power.enable_sdp_charging(2, 5)
        time.sleep(30)
        self.power.boot_up_device()
        self.power.adb_root()

    def test_power_off_with_DCP_charger(self):
        self.power.power_off_timer(10)
        self.power.enable_dcp_charging(1, 0)
        time.sleep(30)
        self.power.enable_sdp_charging(2, 5)
        time.sleep(30)
        self.power.boot_up_device()
        self.power.adb_root()

