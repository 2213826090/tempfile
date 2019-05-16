#! /usr/bin/env python
# coding:utf-8

#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.apps import CleanMusic
from testlib.em.settings import DisplaySetting

class S0i3Audio(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.music = CleanMusic()
        self.music.adb_root()
        self.music.install()
        self.music_file = self.music.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.music.set_screen_status("on")
        self.music.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(S0i3Audio, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.music.stop()
        super(S0i3Audio, self).tearDown()

    def test_s0i3_audio_play(self):
        from testlib.em.constants_def import BXT_M, BXT_O
        self.music.play_audio("file://" + self.music_file)
        time.sleep(5)
        stat_inc = get_s0ix_obj().suspend_resume()
        if self.music.get_product() in [BXT_M, BXT_O]:
            assert stat_inc > 0, "Not enter S3"
        else:
            assert stat_inc == 0, "Enter S3"

    def test_s0i3_resume_audio_play(self):
        self.music.clear_data()
        self.music.get_play_button_position()
        self.music.play_audio("file://" + self.music_file)
        time.sleep(5)

        self.music.press_play_pause_button()
        p1 = self.music.get_play_position()
        self.music.press_play_pause_button()

        stat_inc = get_s0ix_obj().suspend_resume()
        assert stat_inc > 0, "Not enter S3"

        time.sleep(20)
        self.music.press_play_pause_button()
        p2 = self.music.get_play_position()
        assert p2 >= 25

