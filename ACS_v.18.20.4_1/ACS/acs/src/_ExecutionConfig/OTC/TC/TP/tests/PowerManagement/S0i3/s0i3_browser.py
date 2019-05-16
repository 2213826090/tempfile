#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.apps import Firefox
from testlib.em.settings import WifiSetting
from testlib.em.s0i3 import S0i3
from testlib.em.tools import get_config_value, get_server_ip
#from testlib.em.constants_def import *

class S0i3Browser(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.browser = Firefox()
        #self.browser.adb_root()
        self.browser.install()
        self.browser.clear_data()
        self.browser.set_screen_status("on")
        self.browser.unlock_screen()
        #g_common_obj.close_background_apps()
        #super(S0i3Browser, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.browser.stop()
        super(S0i3Browser, self).tearDown()

    def check_s0i3_browser(self, url, data_type = None):
        #WifiSetting().connect_wifi_by_conf("wifi_adb")
        WifiSetting().switch_wifi("off")
        self.browser.open_url(url)
        #if data_type == "audio":
        #    self.browser.play_audio_browser()
        #elif data_type == "video":
        #    self.browser.play_video_browser()
        enter_s3 = S0i3().check_enter_s0i3_state()
        assert enter_s3, "Not enter S0i3 state"

    def test_audio_streaming_enter_s0i3(self):
        url = "http://" + get_server_ip() + get_config_value("webpage", "audio_play")
        self.check_s0i3_browser(url, "audio")

    def test_video_streaming_enter_s0i3(self):
        url = "http://" + get_server_ip() + get_config_value("webpage", "video_play")
        self.check_s0i3_browser(url, "video")

    def test_html5_enter_s0i3(self):
        url = "http://" + get_server_ip() + get_config_value("webpage", "html5demos")
        self.check_s0i3_browser(url)

    def test_webgl_enter_s0i3(self):
        url = "http://" + get_server_ip() + get_config_value("webpage", "webgl")
        self.check_s0i3_browser(url)

    def test_webpage_enter_s0i3(self):
        url = "http://" + get_server_ip() + get_config_value("webpage", "simple_page")
        self.check_s0i3_browser(url)

