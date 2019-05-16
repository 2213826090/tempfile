#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.constants_def import BXT_M, BXT_O
from testlib.em.tools import get_server_ip, get_config_value
from testlib.em.apps import EMToolsDownload
from testlib.em.settings import WifiSetting, DisplaySetting
from testlib.em.s0ix import get_s0ix_obj

class S0i3httpDownload(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.url = "http://" + get_server_ip() + get_config_value("webpage", "big_file")
        self.emtools = EMToolsDownload()
        self.emtools.install()
        self.emtools.grant_permissions()
        self.emtools.clean_download_folder()
        self.display = DisplaySetting()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.display.set_sleep_mode("30 minutes")
        self.connect_wifi()
        super(S0i3httpDownload, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emtools.stop()
        self.display.set_sleep_mode("30 minutes")
        super(S0i3httpDownload, self).tearDown()

    def connect_wifi(self):
        wifi = WifiSetting()
        wifi.connect_wifi_by_conf("wifi_adb")
        assert wifi.check_wifi_connect()

    def test_s0i3_http_download(self):
        self.emtools.download_file(self.url)
        time.sleep(10)
        stat_inc = self.s0ix.suspend_resume()
        if self.s0ix.get_product() in [BXT_M, BXT_O]:
            assert stat_inc > 0, "Not enter S3"
        else:
            assert stat_inc == 0, "Enter S3"

    def test_not_enter_s0i3_15s_idle_http_download(self):
        self.display.set_sleep_mode("15 seconds")
        self.emtools.download_file(self.url)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = 200, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        assert stat_inc == 0, "Enter S3"

    def test_screen_off_wifi_download_background(self):
        self.emtools.download_file(self.url)
        time.sleep(10)
        self.emtools.set_screen_status("off")
        size1 = self.emtools.get_download_file_size()
        time.sleep(30)
        size2 = self.emtools.get_download_file_size()
        assert size1 < size2, "Download file stopped"

