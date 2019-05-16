#! /usr/bin/env python
# coding:utf-8

#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.apps import VideoPlayer
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class S0i3Video(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.player = VideoPlayer()
        self.player.adb_root()
        self.player.install()
        #self.player.grant_permissions()
        self.player.set_screen_status("on")
        self.player.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        #super(S0i3Video, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.player.stop()
        #super(S0i3Video, self).tearDown()

    def test_enter_s0i3_video_play(self):
        video_file = self.player.push_artifactory_resource("video_racing", "/mnt/sdcard/Movies")
        self.player.play_local_video(video_file)
        time.sleep(10)
        assert get_s0ix_obj().suspend_resume(), "Not enter S3"

    def test_post_s0i3_video_play(self):
        video_file = self.player.push_artifactory_resource("video_racing", "/mnt/sdcard/Movies")
        self.player.play_local_video(video_file)
        time.sleep(10)
        assert get_s0ix_obj().suspend_resume(), "Not enter S3"

        #self.player.play_local_video(video_file)
        time.sleep(20)
        position = self.player.get_play_position()
        assert position > 10

