#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class S0i3ThirdApps(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.s0ix.set_screen_status("on")
        self.s0ix.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        self.app = None
        #super(S0i3ThirdApps, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.app.stop()
        #super(S0i3ThirdApps, self).tearDown()

    def check_s0i3_third_app(self):
        self.app.install()
        self.app.clear_data()
        self.app.grant_permissions()
        self.app.launch()
        time.sleep(10)
        assert self.app.check_app_running()
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Enter S3"
        assert self.app.check_app_running()

    def test_enter_s0i3_kitten_cat(self):
        from testlib.em.apps import KittenCat
        self.app = KittenCat()
        self.check_s0i3_third_app()

    def test_enter_s0i3_riptidegp(self):
        from testlib.em.apps import Riptidegp
        self.app = Riptidegp()
        self.check_s0i3_third_app()

    def test_enter_s0i3_clean_music(self):
        from testlib.em.apps import CleanMusic
        self.app = CleanMusic()
        self.check_s0i3_third_app()

    def test_enter_s0i3_gallery(self):
        from testlib.em.apps import SimpleGallery
        self.app = SimpleGallery()
        self.check_s0i3_third_app()

    def test_enter_s0i3_messenger(self):
        from testlib.em.apps import Messenger
        self.app = Messenger()
        self.check_s0i3_third_app()

    def test_enter_s0i3_mytuner(self):
        from testlib.em.apps import MyTuner
        self.app = MyTuner()
        self.check_s0i3_third_app()

    def test_enter_s0i3_vlc(self):
        from testlib.em.apps import VLC
        self.app = VLC()
        self.check_s0i3_third_app()

    def test_enter_s0i3_telegram(self):
        from testlib.em.apps import Telegram
        self.app = Telegram()
        self.check_s0i3_third_app()

    def test_enter_s0i3_music_player(self):
        from testlib.em.apps import MusicPlayer
        self.app = MusicPlayer()
        self.check_s0i3_third_app()

    def test_enter_s0i3_geo_location(self):
        from testlib.em.apps import GeoLocation
        self.app = GeoLocation()
        self.check_s0i3_third_app()

    def test_enter_s0i3_radio_fm(self):
        from testlib.em.apps import RadioFM
        self.app = RadioFM()
        self.check_s0i3_third_app()

