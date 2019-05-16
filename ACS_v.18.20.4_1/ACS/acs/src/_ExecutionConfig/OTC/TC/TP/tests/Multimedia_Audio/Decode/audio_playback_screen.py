# coding: utf-8

import time

from testlib.audio import resource
from testlib.util.common import g_common_obj
from testlib.audio.audio_test_base import AudioStubTestBase

class AudioPlayback(AudioStubTestBase):
    """
    @summary: AudioPlayback used to test audio playback function
    """
    CONFIG_FILE = "tests.tablet.audio.stress.conf"

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(AudioPlayback, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)
        self.audio.cleanUpData()
        self.enable_screen_lock()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(AudioPlayback, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        self.systemui.unlock_screen()
        g_common_obj.stop_app_am("com.google.android.music")
        time.sleep(3)
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        time.sleep(10)
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))

    def appPrepare(self, case_name):
        self.audio.cfg = self.config.read(self.CONFIG_FILE, case_name)
        self.downloadAudioandPush(case_name)
        self.audio.set_orientation_n()

    def downloadAudioandPush(self, config):
        file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        push_path = self.audio.cfg.get("push_audio").split("\" \"")[1].replace("\"","")
        ret_file = resource.get_media_content(file_name)
        g_common_obj.adb_cmd_common(
            "push \"" + ret_file + "\" " + "\"" + push_path + "\"", 600)
        if "extra_list" in self.audio.cfg:
            for each in self.audio.cfg.get("extra_list").split(","):
                each = each.strip()
                ret_file = resource.get_media_content(each)
                assert ret_file, "download file failed: " + each
                g_common_obj.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + push_path + "\"")

        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))

    def audioPlaybackLonglasting(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusicLongLasting(self.audio.cfg.get("audio_name"), \
                                        self.audio.cfg.get("duration_time"), \
                                        self.audio.cfg.get("click_repeat"))

    def screenLockUnlock(self, repeat):
        """
        Lock and Unlock screen loops
        """
        self.logger.debug("[INFO] Lock Unlock screen for %d times" % int(repeat))
        for _ in range(0, int(repeat)):
            self.systemui.lock_screen()
            time.sleep(3)
            self.logger.debug("[INFO]: lock the device successfully")
            self.systemui.wakeup_screen()
            time.sleep(3)
            self.systemui.unlock_screen()
            self.logger.debug("[INFO]: unlock the device successfully")

    def changePlayState(self, repeat):
        """
        @summary: change play state loops
        """
        self.logger.debug("[INFO] Change play state for %d times" % int(repeat))
        for _ in range(0, int(repeat)):
            self.audio.changePlayStateWhileScreenLock()

    def playNextLoop(self, repeat):
        """
        @summary:play next audio loops
        """
        self.logger.debug("[INFO] Change play state for %d times" % int(repeat))
        for _ in range(0, int(repeat)):
            self.audio.playNextWhileScreenLock()

    def testAudioPlayback_Iteration_LockScreenPausePlay_200cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Wake up
        5. Pause/Play the audio
        """
        self.audioPlaybackLonglasting('test_audio_lock_pause_play_loop')
        repeat = self.audio.cfg.get("pause_repeat")
        self.systemui.lock_screen()
        self.systemui.wakeup_screen()
        self.changePlayState(int(repeat))
        self.systemui.wakeup_screen()
        self.audio.checkPlayWhileScreenLock()
        self.systemui.unlock_screen()

    def testAudioLockPlayNextLoop(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Wake up
        5. Play next
        """
        self.downloadAudioandPush('test_audio_lock_play_next_loop_multi')
        self.audioPlaybackLonglasting('test_audio_lock_play_next_loop')
        repeat = self.audio.cfg.get("next_repeat")
        self.systemui.lock_screen()
        self.systemui.wakeup_screen()
        self.playNextLoop(int(repeat))
        self.systemui.wakeup_screen()
        self.audio.checkPlayWhileScreenLock()
        self.systemui.unlock_screen()

    def testAudioPlayback_Iteration_LockUnlockScreen_200cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Unlock the screen
        """
        self.audioPlaybackLonglasting('test_audio_screen_lock_unlock')
        repeat = self.audio.cfg.get("lock_repeat")
        self.systemui.lock_screen()
        self.systemui.wakeup_screen()
        self.screenLockUnlock(int(repeat))
        self.systemui.wakeup_screen()
        self.systemui.unlock_screen()
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))

    def testAudioPlayback_Iteration_LockPausePlayMultiSongs_200cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Pause/Play on Lock screen 200 cycles
        4. Unlock the screen
        """
        #self.downloadAudioandPush('test_audio_lock_play_next_loop_multi')
        self.appPrepare("test_audio_lock_play_next_loop_multi")
        self.audioPlaybackLonglasting('test_audio_screen_lock_unlock_multi')
        repeat = self.audio.cfg.get("lock_repeat")
        self.systemui.lock_screen()
        for _ in range(0, int(repeat)):
            self.systemui.wakeup_screen()
            time.sleep(1)
            # pause
            self.audio.pauseMusicWhileScreenLock()
            time.sleep(1)
            self.assertFalse(self.audio.isMusicPlaying())
            # resume playing
            self.audio.playMusicWhileScreenLock()
            time.sleep(1)
            self.assertTrue(self.audio.isMusicPlaying())
            # next songs
            self.audio.playNextWhileScreenLock()
        self.systemui.unlock_screen()
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))

    def testAudioPlayback_Iteration_PausePlay100cycles_LockScreen20mins(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Wake up
        5. Pause/Play the audio
        """
        self.audioPlaybackLonglasting('test_audio_pause_play_loop_lock')
        repeat = self.audio.cfg.get("pause_repeat")
        sleep_time = self.audio.cfg.get("sleep_time")
        self.audio.audioPlaybackMuchTimes(int(repeat))
        self.systemui.lock_screen()
        time.sleep(int(sleep_time))
        self.systemui.wakeup_screen()
        self.systemui.unlock_screen()
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))

    def testAudioPlayback_Iteration_PauseLockScreen5mins_200cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Pause
        4. Lock the screen
        5. Wake up, unlock screen
        """
        self.audioPlaybackLonglasting('test_audio_pause_lock_loop')
        repeat = self.audio.cfg.get("lock_repeat")
        sleep_time = self.audio.cfg.get("sleep_time")
        for _ in range (0, int(repeat)):
            self.audio.audioPlaybackMuchTimes(1)
            self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))
            self.systemui.lock_screen()
            time.sleep(int(sleep_time))
            self.systemui.wakeup_screen()
            self.systemui.unlock_screen()
            time.sleep(2)

    def testAudioPlayback_Iteration_LockScreen5mins_100cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Back home
        4. Lock the screen
        5. Wake up
        6. Play the audio
        """
        self.audioPlaybackLonglasting('test_audio_play_back_lock_loop')
        repeat = self.audio.cfg.get("lock_repeat")
        sleep_time = self.audio.cfg.get("sleep_time")
        for _ in range (0, int(repeat)):
            self.d.press.home()
            time.sleep(1)
            self.test_device.lock_screen()
            time.sleep(int(sleep_time))
            self.test_device.unlock_screen()
            self.audio.launch_music_from_home()
            self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))
            self.audio.audioPlaybackMuchTimes(1)

    def testAudioPlayback_Iteration_100SongsLoopLockUnlock_100cycles(self):
        """
        This test used to test Audio playback while screen is locked
        The test case spec is following:
        1. Launch music
        2. Play music
        3. Lock the screen
        4. Unlock the screen
        5. Play music sequence
        """
        #self.downloadAudioandPush('test_audio_lock_play_next_loop_multi')
        self.appPrepare("test_audio_lock_play_next_loop_multi")
        self.audioPlaybackLonglasting('test_audio_lock_unlock_next_multi')
        repeat = self.audio.cfg.get("play_repeat")
        for _ in range (0, int(repeat)):
            self.systemui.lock_screen()
            time.sleep(3)
            self.systemui.wakeup_screen()
            self.systemui.unlock_screen()
            time.sleep(3)
            self.audio.playNextSong()
            time.sleep(3)
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))
