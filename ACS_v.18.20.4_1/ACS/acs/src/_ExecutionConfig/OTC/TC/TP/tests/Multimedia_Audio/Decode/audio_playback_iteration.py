# coding: utf-8

from testlib.audio import resource
from testlib.util.common import g_common_obj
from testlib.audio.audio_test_base import AudioStubTestBase
import time


class AudioPlayBackIteration(AudioStubTestBase):
    """
    @summary: AudioPlayback used to test audio playback function
    """
    CONFIG_FILE = 'tests.tablet.mum_auto_audio.conf'

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(AudioPlayBackIteration, self).setUp()
        self._test_name = __name__
        self.logger.debug( "[Setup]: %s" % self._test_name)
        self.test_device.stop_app_am("com.google.android.music")

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(AudioPlayBackIteration, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        time.sleep(3)
        self.test_device.gmusic.cleanUpData()
        time.sleep(3)
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        time.sleep(10)
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))

    def appPrepare(self, case_name):
        self.audio.cfg = self.config.read(self.CONFIG_FILE, case_name)
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        self.file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        self.push_path = self.audio.cfg.get("push_audio").split("\" \"")[1].replace("\"","")
        ret_file = resource.get_media_content(self.file_name)
        g_common_obj.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + self.push_path + "\"")
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))
        self.audio.set_orientation_n()
        self.audio.cleanUpData()

    def audioPlaybackIteration(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        for it in range(int(self.video.cfg.get("iteration_times"))):
            self.audio.enterPlayPageFromHome()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.audio.cfg.get("audio_name"))
            self.audio.checkMusicPlayBack(stopTime=self.audio.cfg.get("stop_time"))
            self.setting.recent_app()
            self.setting.remove_recent_app("Play music")
        self.logger.debug("case " + str(case_name) + " is pass")


    def test_audio_playback_iteration_001(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 for 3GP continuouly play 20 times')
        """
        self.audioPlaybackIteration("test_audio_playback_iteration_001")
