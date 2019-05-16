import time
from testlib.audio import resource
from testlib.audio.audio_test_base import AudioStubTestBase
try:
    from testlib.audio.bt_audio import BTAudioAdapter
except:
    print "Warn: testaid library not found"
from testlib.audio.helper import dtmf_random_str


class InterruptionTest(AudioStubTestBase):
    def testLocalDecode_InstallSW(self):
        self.logger.debug("get apk file")
        apk_path = resource.get_app("GLBenchmark.apk")
        self.rpc.playLong()  # start play music
        # install SW
        self.adb.adb_cmd_common("install -r %s" % apk_path)
        try:
            self.assertTrue(self.rpc.isPlaying())
        finally:
            # clean up
            self.adb.adb_cmd("pm uninstall com.glbenchmark.glbenchmark25")

    def testAudioDecode_JumpNextPrevious_Continuously(self):
        self.deploy_play_music_content()
        try:
            self.launch_play_music_n_play("mp3_sample2")
            playtime = 5
            last_track = self.audio.getTrackName()
            for _ in xrange(3):
                self.audio.clickPrevious()  # first prev seek to 0
                self.audio.clickPrevious()  # jump to previous
                time.sleep(2)
                cur_track = self.audio.getTrackName()
                assert last_track != cur_track, "Can't jump to previous track"
                last_track = cur_track
                curtime = self.audio.getCurrentTimestamp()
                assert curtime < 5, \
                    "Jump to previou music failed, timestamp:%ss" % curtime
                time.sleep(playtime)
                self.audio.clickNext()
                time.sleep(2)
                cur_track = self.audio.getTrackName()
                assert last_track != cur_track, "Can't jump to next track"
                last_track = cur_track
                curtime = self.audio.getCurrentTimestamp()
                assert curtime < 5, \
                    "Jump to next music failed, timestamp:%s" % curtime
                time.sleep(playtime)
        except:
            self.screenshot()
            raise
        finally:
            self.adb.stop_app_am(self.audio.PACKAGE_NAME_PLAY_MUSIC)
            self.clean_play_music_content()

    def testAudioDecode_MovePreviousNextQuickly_TakePicture_RecordVideoHalfHour(self):  # NOQA
        from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper  # NOQA
        camera_helper = MultiMediaCameraHelper()
        self.deploy_play_music_content()
        try:
            self.launch_play_music_n_play("mp3_sample2")
            for _ in xrange(3):
                time.sleep(1)
                self.audio.clickPrevious()  # first prev seek to 0
                self.audio.clickPrevious()  # jump to previous
                time.sleep(1)
                self.audio.clickNext()
            self.adb.stop_app_am(self.audio.PACKAGE_NAME_PLAY_MUSIC)

            # record video half hour
            # clean data first
            camera_helper.camera.cleanMediaFiles()
            camera_helper.CameraRecord(60*30, "Video", "back")
            self.d.press.back()
        finally:
            self.clean_play_music_content()
            # clean video
            camera_helper.camera.cleanMediaFiles()

    def testBTA2DPDecode_DialPanelTouchTone(self):
        '''
        Verify that music play smoothly on BT A2DP during play Dial panel touch tone
        '''
        DIALERPAD = 'com.android.dialer'
        bt = None
        self.deploy_play_music_content()
        try:
            bt = BTAudioAdapter()
            bt.setup()
            bt.connect(True)
            self.launch_play_music_n_play("mp3_sample2")
            assert self.bt.a2dp_sink.is_active(), "No sound in BT a2dp"
            self.test_device.adb_cmd("am start -W %s"%DIALERPAD)
            time.sleep(2)
            syms = dtmf_random_str(5)
            for sym in syms:
                self.d(descriptionContains="%s" % sym).click.wait()
                time.sleep(.5)
            wav = bt.a2dp_sink.get_last_wav()
            self.assert_dtmf_exists(wav, syms, similar=0.6)
        finally:
            if bt:
                bt.teardown()

    def testBTEnableDisableIteratively_PlaybackBTA2DP(self):
        '''
        Verify that music can decode in BT A2DP after enable disable iteratively
        '''
        bt = None
        self.deploy_play_music_content()
        try:
            bt = BTAudioAdapter()
            bt.setup()
            for i in range(10):
                bt.connect(False)
                time.sleep(1)
                bt.connect(True)
                time.sleep(1)
            self.launch_play_music_n_play("mp3_sample2")
            assert bt.a2dp_sink.is_active(), "No sound in BT A2DP"
        finally:
            if bt:
                bt.teardown()