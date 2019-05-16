#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class S0i3_15s_Idle(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.sleep_time = self.s0ix.get_sleep_time_from_conf() + 15
        self.display = DisplaySetting()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.display.set_sleep_mode("15 seconds")
        self.s0ix.testDevice.close_background_apps()
        super(S0i3_15s_Idle, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.s0ix.stop_focused_activity()
        self.s0ix.set_screen_status("on")
        self.s0ix.unlock_screen()
        super(S0i3_15s_Idle, self).tearDown()

    def test_not_enter_s0i3_15s_idle_audio_play(self):
        from testlib.em.apps import CleanMusic
        self.app = CleanMusic()
        self.app.install()
        self.app.grant_permissions()
        music_file = self.app.push_artifactory_resource("long_music", "/mnt/sdcard/Music")
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.app.play_audio("file://" + music_file)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = self.sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        self.s0ix.stop_focused_activity()
        assert stat_inc == 0, "Enter S3"

    def test_not_enter_s0i3_15s_idle_video_play(self):
        from testlib.em.apps import VideoPlayer
        self.app = VideoPlayer()
        self.app.install()
        self.app.grant_permissions()
        video_file = self.app.push_artifactory_resource("video", "/mnt/sdcard/Movies")
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.app.play_local_video(video_file)
        time.sleep(5)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = self.sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        self.s0ix.stop_focused_activity()
        assert stat_inc == 0, "Enter S3"

    def test_not_enter_s0i3_15s_idle_kitten_cat(self):
        from testlib.em.apps import KittenCat
        self.app = KittenCat()
        self.app.install()
        self.app.grant_permissions()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.app.launch()
        time.sleep(10)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = self.sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        self.s0ix.stop_focused_activity()
        assert stat_inc == 0, "Enter S3"

    def test_enter_s0i3_15s_idle_camera(self):
        from testlib.em.apps import Camera
        self.app = Camera()
        self.app.grant_permissions()
        self.app.launch()
        self.sleep_time = max(self.sleep_time, 300)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = self.sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        self.s0ix.stop_focused_activity()
        assert stat_inc > 0, "Not enter S3"

    def test_enter_s0i3_15s_idle_auto_rotation(self):
        self.display.set_auto_rotate(True)
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = self.sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        assert stat_inc > 0, "Not enter S3"
        self.s0ix.rotate("l")
        self.s0ix.rotate("n")

