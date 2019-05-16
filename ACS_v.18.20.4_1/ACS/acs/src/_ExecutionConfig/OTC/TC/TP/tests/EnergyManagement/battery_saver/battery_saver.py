#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.em_impl import EMImpl

class BatterySaver(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.adb_root()
        self.result_dir = self.emImpl.get_tmp_dir()
        self.wait_time = int(self.emImpl.get_config_value("battery_saver", "wait_time"))
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        self.emImpl.set_brightness_level(255)
        g_common_obj.close_background_apps()
        super(BatterySaver, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.check_adb_or_reconnect()
        self.emImpl.remove_tmp_dir(self.result_dir)
        super(BatterySaver, self).tearDown()

    def check_battery_history(self, name = None, data = None):
        # connect WIFI_ADB
        ssid, passwd = self.emImpl.read_wifi_conf()
        self.emImpl.connect_wifi(ssid, passwd)
        wifi_serial = self.emImpl.enable_wifi_adb()
        self.emImpl_wifi = EMImpl(wifi_serial)

        self.emImpl.enable_full_wake_history()
        self.emImpl.reset_batterystats()
        self.emImpl.launch_settings("Battery saver")
        self.emImpl.disconnect_adb()
        self.emImpl_wifi.open_battery_saver()

        if name == "record_video":
            self.emImpl_wifi.grant_permissions_for_camera_app()
            self.emImpl_wifi.launch_camera("Video")
            self.emImpl_wifi.click_shutter_button()
        elif name == "chrome":
            self.emImpl_wifi.chrome_open_url(data)
            if data.endswith("mp3"):
                self.emImpl_wifi.play_audio_chrome()
            elif data.endswith("mp4"):
                self.emImpl_wifi.play_video_chrome()
        elif name == "audio":
            self.emImpl_wifi.play_media_file("audio/*", data)
        elif name == "video":
            self.emImpl_wifi.play_media_file("video/*", data)
        elif name == "templerun":
            self.emImpl_wifi.launch_templerun()
            time.sleep(60)
            self.emImpl_wifi.play_templerun()
        elif name == "glbenchmark":
            self.emImpl_wifi.run_glbenchmark27()

        time.sleep(self.wait_time)
        self.emImpl.connect_adb()

        bugreport_file = self.emImpl.save_bugreport(self.result_dir)
        html_file = self.emImpl.generate_battery_history_html(bugreport_file)
        self.emImpl.parse_battery_history_html(html_file)

    def test_battery_life_compared_record_video(self):
        """
        Test battery life compared record 1080P video max backlight
        """
        print "[RunTest]: %s" % self.__str__()
        self.check_battery_history(name = "record_video")

    def test_battery_life_compared_chrome_web_browser(self):
        """
        Test battery life compared chrome web browser
        """
        print "[RunTest]: %s" % self.__str__()
        url = "http://" + self.emImpl.get_server_ip() + self.emImpl.get_config_value("webpage", "webgl")
        self.check_battery_history(name = "chrome", data = url)

    def test_battery_life_compared_chrome_audio_play(self):
        """
        Test battery life compared chrome audio play
        """
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.close_chrome_tabs()
        url = "http://" + self.emImpl.get_server_ip() + self.emImpl.get_config_value("webpage", "audio_play")
        self.check_battery_history(name = "chrome", data = url)

    def test_battery_life_compared_chrome_video_play(self):
        """
        Test battery life compared chrome video play
        """
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.close_chrome_tabs()
        url = "http://" + self.emImpl.get_server_ip() + self.emImpl.get_config_value("webpage", "video_play")
        self.check_battery_history(name = "chrome", data = url)

    def test_battery_life_compared_audio_play(self):
        """
        Test battery life compared audio play
        """
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.grant_music_app_permissions()
        music_file = self.emImpl.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.check_battery_history(name = "audio", data = "file://" + music_file)

    def test_battery_life_compared_video_play(self):
        """
        Test battery life compared video play
        """
        print "[RunTest]: %s" % self.__str__()
        video_file = self.emImpl.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.check_battery_history(name = "video", data = "file://" + video_file)

    def test_battery_life_compared_play_3D_game(self):
        """
        Test battery life compared play 3D game
        """
        print "[RunTest]: %s" % self.__str__()
        temple_run_package = "com.imangi.templerun2"
        self.emImpl.install_artifactory_app("templerun", temple_run_package)
        self.check_battery_history(name = "templerun")

    def test_battery_historian_tool(self):
        """
        Test battery historian tool
        """
        print "[RunTest]: %s" % self.__str__()
        self.check_battery_history()

    def test_battery_life_compared_glbenchmark27(self):
        """
        Test battery life compared glbenchmark27
        """
        print "[RunTest]: %s" % self.__str__()
        app_package = 'com.glbenchmark.glbenchmark27'
        self.emImpl.install_artifactory_app("glbenchmark", app_package)
        self.check_battery_history(name = "glbenchmark")

