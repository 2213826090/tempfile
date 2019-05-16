import time
import datetime
from testlib.audio.bt_audio import BTAudioAdapter
from testlib.audio.audio_test_base import AudioStubTestBase
from testlib.audio.resource import get_media_content


class BTAudioTest(AudioStubTestBase):
    def setUp(self):
        super(BTAudioTest, self).setUp()
        self.bt = BTAudioAdapter()
        self.bt.setup()

    def tearDown(self):
        self.bt.teardown()
        super(BTAudioTest, self).tearDown()

    def check_wav(self, wav):
        pass

    def wait_a2dp_sink_pending(self, timeout=20):
        self.logger.debug("BTAudioTest: Wait for a2dp sink pending %ds"
                          % timeout)
        start = time.time()
        while self.bt.a2dp_sink.is_active():
            if time.time() - start > timeout:
                return False
            time.sleep(.5)
        return True

    def set_alarm(self, minutes):
        """
        Set an alarm after minutes
        @param int minute: the delay time of minute
        """
        time_delay = datetime.timedelta(0, minutes*60, 0)
        cmd = "date +%Y:%m:%d:%H:%M:%S"
        self.adb.root_on_device()  # set alarm Intent require root permission
        time.sleep(3)
        dut_time_str = self.adb.adb_cmd_capture_msg(cmd).strip().split(":")
        assert len(dut_time_str) == 6, \
            "Get DUT time failed. Actually get %s" % " ".join(dut_time_str)
        dut_time = datetime.datetime(*map(int, dut_time_str))
        alarm_time = dut_time + time_delay
        self.logger.debug("DUT time: %s, alarm time: %s"
                          % (str(dut_time), str(alarm_time)))
        set_alarm_cmd = ["am start -a android.intent.action.SET_ALARM",
                 "--ei android.intent.extra.alarm.HOUR %s" % alarm_time.hour,
                 "--ei android.intent.extra.alarm.MINUTES %s" % alarm_time.minute,
                 "--ez android.intent.extra.alarm.SKIP_UI true"]
        self.adb.adb_cmd(" ".join(set_alarm_cmd))
        time.sleep(2)

    def wait_alarm(self, timeout=180):
        """
        Wait for alarm comes.
        @param timeout: int, timeout second
        """
        self.logger.debug("BTAudioTest: Waiting alarm for %ds"%timeout)
        start = time.time()
        time_wait = 5
        while time.time() - start < timeout:
            if self.rpc.isStreamActive("alarm", 0):
                return True
            time.sleep(time_wait)
        return False

    def dismiss_alarm(self):
        """
        @summary: Dismiss alarm when alarm comes
        """
        assert self.rpc.isStreamActive("alarm", 0), "Alarm not comes"
        #self.launch_alarm()
        self.d(resourceId="com.android.deskclock:id/onoff", checked=True).click()
        time.sleep(.5)

    def launch_alarm(self):
        """
        @summary: Launch Alarm from Deskclock
        """
        alarm_pkg = "com.google.android.deskclock"
        alarm_activity = "com.android.deskclock.DeskClock"
        self.logger.debug("BTAudioTest: Launch alarm app")
        cmd = "am start -S %s/%s"%(alarm_pkg, alarm_activity)
        self.adb.adb_cmd(cmd)
        time.sleep(2)
        assert self.d.info.get("currentPackageName") == alarm_pkg, \
                "Launch DeskClock app failed."
        self.d(description="Alarm", index=0).click()
        time.sleep(.5)

    def delete_alarms(self):
        """
        @summary: Delete all enabled alarms
        """
        pkgname = 'com.android.deskclock'
        self.logger.debug("BTAudioTest: Delete all enabled alarms")
        switch_on = self.d(resourceId=pkgname+":id/onoff", checked=True)
        while switch_on.exists:
            expand = switch_on.down(resourceId=pkgname+":id/collapse_expand")
            if not expand:
                expand = switch_on.down(description="Expand alarm")
            expand.click.wait()
            time.sleep(1)
            self.d(resourceId=pkgname+":id/delete").click()
            time.sleep(1)

    def testAudioDecode_BTSoundbox(self):
        ''' Verify that device can play music with BT soundbox '''
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        self.assertTrue(self.rpc.isPlaying(), "Music is not playing")
        self.assertTrue(self.bt.a2dp_sink.is_active(),
                        "A2DP Sink is not active")
        self.rpc.resetPlayer()  # this will stop the music
        time.sleep(5)  # need some delay for file saved
        self.assertFalse(self.bt.a2dp_sink.is_active(),
                         "A2DP Sink is still active after music stop")
        wav = self.bt.a2dp_sink.get_last_wav()
        self.check_wav(wav)

        # actually, no needs to disconnect,
        # since when we remove profile in tearDown(),
        # BT will disconnect then.
        self.bt.connect(False)

    def testPlayback_VolumeControl_BTHeadsetButtons(self):
        '''Verify that BT headset volume button could control the volume.'''
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        # press button in BT side
        control = self.bt.get_media_control()
        control.volume_up()
        time.sleep(2)
        control.volume_down()
        time.sleep(2)
        # actually, no needs to disconnect,
        # since when we remove profile in tearDown(),
        # BT will disconnect then.
        self.bt.connect(False)

    def testBTHeadsetVolumeKept_AfterAdjustSpeakerVolume(self):
        ''' Verify BT headset volume is kept after adjust the speaker volume

        1.Connect BT headset
        2.Play music and Adjust the volume on BT and Remember the volume level
        3.Change route to IHF and Adjust the volume on IHF
          and Remember the volume level
        4.Change route to BT and check the BT's volume level
        '''
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(2)
        # adjust the volume
        vol = self.rpc.getStreamVolume("music")
        time.sleep(1)
        self.rpc.setStreamVolume("music", int(vol/2))
        time.sleep(1)
        bt_vol = self.rpc.getStreamVolume("music")
        self.assertNotEqual(vol, bt_vol, "Can't change BT A2DP volume")

        # disconect BT, adjust volume on Speaker
        self.bt.connect(False)
        vol = self.rpc.getStreamVolume("music")
        self.rpc.setStreamVolume("music", int(vol*3.0/2))
        time.sleep(1)
        spk_vol = self.rpc.getStreamVolume("music")
        self.assertNotEqual(vol, spk_vol, "Can't change speaker volume")

        # now connect BT again, and check the volume
        self.bt.connect(True)
        time.sleep(1)
        vol = self.rpc.getStreamVolume("music")
        self.assertEqual(vol, bt_vol,
                         "Speaker volume changes affect BT volume")

        # done, disconnect BT
        self.bt.connect(False)

    def testAudioEncode_BTHSP_Mono(self):
        ''' Verify that can capture mono voice memo over BT HSP '''
        self.bt.connect(True)
        self.rpc.enableSco(True)  # enable BTHSP SCO
        time.sleep(1)
        record_path = "/sdcard/sco_record.amrwb"
        self.adb.adb_cmd("rm " + record_path)  # remove file if exists
        self.rpc.startRecording(record_path)
        # play sco.wav through BTHSP mic
        sco_wav = get_media_content("sco.wav")
        self.bt.sco.send_wav_file(sco_wav)
        time.sleep(30)

        self.rpc.stopRecording()
        self.rpc.enableSco(False)

        # check if we can play record file
        self.rpc.playFile(record_path)
        time.sleep(2)
        self.assertTrue(self.rpc.isPlaying(), "Can't play record file")
        self.adb.adb_cmd("rm " + record_path)
        self.rpc.enableSco(False)

    def testPlayback_BT_PressPowerButton_LockSound(self):
        """
        @summary:Verify that screen locksound can be heard on BT while playback
        1. Connect to BT device.
        2. Start audio playback
        3. Enable screen lock sound
        3. Check screen lock sound can be heard on BT
        """
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        self.assertTrue(self.rpc.isPlaying(), "Music is not playing")
        self.assertTrue(self.bt.a2dp_sink.is_active(),
                        "A2DP Sink is not active")
        self.enable_screen_lock()#Enable screen lock sound.
        wav = self.bt.a2dp_sink.get_last_wav()
        self.check_wav(wav)#Check a2dp sound.
        #Disable screen lock sound.
        self.bt.connect(False)#BT Disconnect

    def testScreenLockSound_BTA2DP(self):
        """
        @summary:Verify that screen locksound can be heard on BT while playback
        1. Connect to BT device.
        2. Enable screen lock sound
        3. Check screen lock sound can be heard on BT
        """
        self.bt.connect(True)
        time.sleep(5)
        self.enable_screen_lock()#Enable screen lock sound.
        time.sleep(.5)
        #waiting for a2dp sink pending.
        if not self.wait_a2dp_sink_pending():
            self.failureException("Wait for a2dp sink pending timeout")
        wav_orig = self.bt.a2dp_sink.get_last_wav()
        self.logger.debug("original wav: %s"%wav_orig)
        self.test_device.lock_screen()
        time.sleep(.5)
        self.test_device.unlock_screen()
        if not self.wait_a2dp_sink_pending():
            self.failureException("Wait for a2dp sink pending timeout")
        wav_new = self.bt.a2dp_sink.get_last_wav()
        self.logger.debug("new wav: %s"%wav_new)
        assert wav_orig != wav_new, "Check screen lock sound failed, No sound come out."

    def testAudioDecodeRouteChange_IHF_BTA2DP_EnableDisableEffect(self):
        """
        @summary: Can change accessory between IHF and A2DP during playback
         and enable/disable effect
        1. Audio playback
        2. Connect to A2DP and check sound in A2DP
        3. Enable/disable effect.
        """
        self.rpc.playLong()  # play music
        time.sleep(5)
        self.assertTrue(self.rpc.isPlaying(), "Music is not playing")
        self.bt.connect(True)#Connect BT
        time.sleep(5)
        self.assertTrue(self.bt.a2dp_sink.is_active(),
                    "A2DP Sink is not active while change accessory to A2DP")
        self.rpc.enableEffect()
        self.assertTrue(self.bt.a2dp_sink.is_active(),
                    "A2DP Sink is not active while enable effects")
        #self.rpc.disableEffect()
        #self.assertTrue(self.bt.a2dp_sink.is_active(),
        #            "A2DP Sink is not active while enable effects")
        self.bt.connect(False)#BT Disconnect

    def testKeypressTone_BTA2DP(self):
        """
        @summary: Check key press tone can be heard on A2DP
        1. Enable key press sound in settings
        2. Connect to A2DP
        3. Press back/home/key
        """
        self.bt.connect(True)
        time.sleep(5)
        self.launch_sound_settings()
        self.scroll_n_click("Other sounds")
        self.set_option("Touch sounds", enable = True)
        switch_wgt = self.d(className="android.widget.TextView",
            text="Touch sounds").right(className="android.widget.Switch")
        if not switch_wgt or switch_wgt.enabled == False:
            raise self.failureException("Enable touch sound failed.")
        if not self.wait_a2dp_sink_pending():
            self.failureException("Wait for a2dp sink pending timeout")
        wav_orig = self.bt.a2dp_sink.get_last_wav()
        self.logger.debug("BTAudioTest: Original wav: %s"%wav_orig)
        self.d(text = "Other sounds").left().click()
        # sometimes, there is a delay route to A2DP
        # if not delay here, wait_a2dp_sink_pending will return immediately
        # then, we still get the old wav
        time.sleep(4)
        if not self.wait_a2dp_sink_pending():
            self.failureException("Wait for a2dp sink pending timeout")
        wav_new = self.bt.a2dp_sink.get_last_wav()
        self.logger.debug("BTAudioTest: New wav: %s"%wav_new)
        assert wav_orig != wav_new, "Check keypress sounc failed. No sound come out."
        self.bt.connect(False)

    def testPlayback_BTA2DP_VolumeControl_InSettings(self):
        """
        @summary: Verify music/alarm/notification vol can be changed in \
            settings during music playback on A2DP
        1. Connect ot A2DP and start Music playback
        2. Change Music/alarm/notification vol in settings.
        """
        def seekbar_un_down(seekbar_bounds):
            y = (seekbar_bounds["top"] + seekbar_bounds["bottom"])/2
            x_start = seekbar_bounds["left"] + 10
            x_end = seekbar_bounds["right"] - 10
            middle = ((x_end + x_start)/2, y)
            down = ((middle[0] + x_start)/2, y)
            up = ((middle[0] + x_end)/2, y)
            self.d.click(*down)
            time.sleep(1)
            self.d.click(*up)
            time.sleep(1)
            self.d.click(*middle)
            time.sleep(1)

        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        assert self.rpc.isPlaying(), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        self.launch_sound_settings()
        music_seekbar = self.d(text="Media volume")\
            .down(resourceId="android:id/seekbar").info["bounds"]
        alarm_seekbar = self.d(text="Alarm volume")\
            .down(resourceId="android:id/seekbar").info["bounds"]
        for t in ['Notification volume', 'Ring volume']:  # Tabet/Phone
            if self.d.exists(text=t):
                text = t
                break
        notf_seekbar = self.d(text=text)\
            .down(resourceId="android:id/seekbar").info["bounds"]
        self.logger.debug("BTAudioTest: Change music/alarm/notification vol")
        seekbar_un_down(music_seekbar)
        seekbar_un_down(alarm_seekbar)
        seekbar_un_down(notf_seekbar)
        assert self.bt.a2dp_sink.is_active(), "A2DP audio playback dropped!"
        self.bt.connect(False)

    def testStreaming_Disconnect_BT_A2DP(self):
        """
        @summary: Verify disconnect BT A2DP during audio streaming playback
        1. Connect BT A2DP and start audio streaming playback
        2. Disconnect A2DP, Check playback.
        """
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        assert self.rpc.isPlaying(), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        self.bt.connect(False)
        time.sleep(1)
        assert self.rpc.isPlaying(), "Music is not playing after disconnect BT"
        self.bt.connect(False)

    def testPlayback_BTA2DP_VolumeControl_HWVolumeKeys(self):
        """
        @summary: Verify that volume can be changed by hw volume keys during playback
        1. Connect to A2DP and start playback
        2. Change volume on A2DP
        """
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        assert self.rpc.isPlaying(), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        current_volume = self.rpc.getStreamVolume("music")
        self.logger.debug("BTAudioTest: Current volume: %s"%current_volume)
        self.d.press.volume_up()
        time.sleep(2)
        up_volume = self.rpc.getStreamVolume("music")
        self.logger.debug("BTAudioTest: Up volume: %s"%up_volume)
        assert up_volume > current_volume, \
                "Volume up failed: before %s, after %s"%(current_volume, up_volume)
        self.d.press.volume_down()
        time.sleep(2)
        down_volume = self.rpc.getStreamVolume("music")
        self.logger.debug("BTAudioTest: down volume: %s"%down_volume)
        assert up_volume > down_volume, \
                "Volume down failed: before %s, after %s"%(up_volume, down_volume)
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        self.bt.connect(False)

    def testBTA2DPDecode_Take20Picture(self):
        """
        @summary: Verify music play smoothly on BT during music playback + Camera(20 pictures)
        1. Connect to A2DP and Start audio playback
        2. Launch camera and take 20 photos.
        """
        LOOP = 20
        PHOTO_CAPTURE_TIMEOUT = 2

        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        assert self.rpc.isPlaying(), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper
        from testlib.camera.CameraCommon import CameraCommon
        camera_helper = MultiMediaCameraHelper()
        self.camera = camera_helper.camera
        self.camera.cleanMediaFiles()
        self.logger.debug("BTAudioTest: Launch camera")
        self.camera.startCameraApp()
        self.camera.selectMode()
        self.camera.switchRearOrFront()
        try:
            for i in range(LOOP):
                self.logger.debug("BTAudioTest: Take photos loop %d"%(i+1))
                CameraCommon().checkCameraCrash()
                self.camera.capturePhoto()
                time.sleep(PHOTO_CAPTURE_TIMEOUT)
                assert self.bt.a2dp_sink.is_active(), \
                    "A2DP Sink is not active after take %d photos."%(i+1)
        finally:
            self.camera.stopCameraApp()
            self.camera.cleanMediaFiles()

    def testAudioDecode_BTA2DP_TurnonAirPlaneMode(self):
        """
        @summary: Verify that music stopped during music playing through BT A2DP
        headset and Switch to air-plane mode
        1. Connect to BT A2DP and start audio playback
        2. Turn on airplan mode and check
        3. Trun off airplan mode and check
        """
        self.bt.connect(True)
        self.rpc.playLong()  # play music
        time.sleep(5)
        assert self.rpc.isPlaying(), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        self.adb.adb_cmd("am start -W com.android.settings")
        self.scroll_n_click("More")
        time.sleep(1)
        self.logger.debug("BTAudioTest: Enable airplane mode")
        try:
            self.set_option("Airplane mode",enable = True)
            assert self.rpc.isPlaying(), \
                    "Music is playing while airplane mode on"
            if not self.wait_a2dp_sink_pending(timeout = 30):
                self.failureException("Still have sound come out on BT A2DP")
        finally:
            self.adb.adb_cmd("am start -W com.android.settings")
            self.scroll_n_click("More")
            time.sleep(1)
            self.set_option("Airplane mode",enable = False)

    def testBTA2DPDecode_5Alarm(self):
        """
        @summary: Verify that music play normally and alarm sound route correct on
                BT A2DP when music playback +Alarm(set 5 alalarms)
        1. Connect BT A2DP and start audio playback
        2. Set an alarms
        3. Wait for alarm comes and check wav on A2DP
        4. Snooze it and check auido playback resumed on A2DP
        5. Repeat 2~4 5 loops
        """
        LOOP = 5
        self.bt.connect(True)
        self.deploy_play_music_content()
        self.launch_play_music_n_play(self.media_files[0].split(".")[0])
        time.sleep(5)
        assert self.rpc.isStreamActive("music", 0), "Music is not playing"
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        try:
            for i in range(LOOP):
                self.set_alarm(1)
                self.wait_alarm(65)
                time.sleep(1)
                assert self.audio.isInPlayPage(), "Not in music play page"
                assert not self.audio.isMusicPlaying(), \
                        "Music playback is still active when alarm comes"
                assert not self.rpc.isStreamActive("music", 0), \
                        "Music is still playing when alarm comes"
                assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
                self.launch_alarm()
                #self.dismiss_alarm()
                self.delete_alarms()
                self.d.press.back()#back to song play page
                time.sleep(.5)
                assert self.audio.isMusicPlaying(), \
                        "Music still paused after dismiss alarm"
                assert self.rpc.isStreamActive("music", 0), \
                        "Music resume failed after dismiss alarm"
                assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
                time.sleep(1)
        finally:
            self.launch_alarm()
            self.delete_alarms()
            self.clean_play_music_content()

    def do_effect_test(self, effect):
        self.bt.connect(True)
        self.rpc.playLong()  # start playing music
        time.sleep(2)
        # enable effect
        self.rpc.enableEffect(effect)
        time.sleep(5)
        assert self.rpc.isPlaying(), \
            "music not playing after enable effect: " + effect
        assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        self.rpc.disableEffect()

    def testAudioEffect_Equalizer_BTA2DP(self):
        self.do_effect_test("Equalizer")

    def testAudioEffect_BassBoost_BTA2DP(self):
        self.do_effect_test("BassBoost")

    def testAudioEffect_EnvironmentalReverb_BTA2DP(self):
        self.do_effect_test("EnvironmentalReverb")

    def testAudioEffect_PresetReverb_BTA2DP(self):
        self.do_effect_test("PresetReverb")

    def testAudioEffect_Virtualizer_BTA2DP(self):
        self.do_effect_test("Virtualizer")

    def testAllAudioEffects_BTA2DP(self):
        self.bt.connect(True)
        self.deploy_play_music_content()
        try:
            self.launch_play_music_n_play(self.media_files[0].split(".")[0],
                                          effect=True)
            time.sleep(5)
            assert self.bt.a2dp_sink.is_active(), "A2DP Sink is not active"
        finally:
            self.clean_play_music_content()
            self.adb.adb_cmd("am force-stop "
                             + self.audio.PACKAGE_NAME_PLAY_MUSIC)
            self.audio.enterPlayPageFromHome()
            self.audio.turnOffEffect()
