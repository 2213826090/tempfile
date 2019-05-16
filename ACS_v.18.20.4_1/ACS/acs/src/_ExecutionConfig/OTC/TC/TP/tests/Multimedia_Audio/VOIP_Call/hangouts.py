'''
Created on Dec 7, 2015

@author: bob
'''
from testlib.audio.audio_test_base import AudioHangoutsTestBase
try:
    from testlib.audio.bt_audio import BTAudioAdapter
    from testaid.headset import Headset
except:
    print "Warn: testaid library not found"
import time
import os
import tempfile

class HangoutsTest(AudioHangoutsTestBase):
    '''
    Audio hangouts test cases
    '''
    CONFIG_FILE = "audio.hangouts.conf"
    DUT_PATH = "/sdcard/otctest/"

    def setUp(self):
        super(HangoutsTest, self).setUp()

    def tearDown(self):
        super(HangoutsTest, self).tearDown()

    def testVOIPCall_While_WLANStreaming_MT(self):
        """
        Verify that streaming is stopped after receiving VOIP call
        1. Play a WLAN audio streaming
        2. Receive a voice hangouts call
        3. Check online streaming stopped while in VOIP call
        """
        STREAM_URL = self.config.getConfValue(file_src=self.CONFIG_FILE, section="urls", key="url_1")
        CHROME = "com.android.chrome"#,"org.chromium.chrome.browser.ChromeTabbedActivity"
        from testlib.browser.browser_impl import BrowserImpl
        chrome = BrowserImpl()
        chrome.clear_data()
        uia_d = self.d.get_device()
        try:
            chrome.launchChrome()
            time.sleep(1)
            for i in range(5):
                if uia_d(text = "Accept & continue").exists:
                    uia_d(text = "Accept & continue").click()
                    time.sleep(.5)
                if uia_d(text = "Done", className = "android.widget.Button").exists:
                    uia_d(text = "Done").click()
                    time.sleep(.5)
                if uia_d(text="No thanks").exists:
                    uia_d(text="No thanks").click.wait()
                time.sleep(1)
            assert uia_d.info.get("currentPackageName") == CHROME, "Launch Chrome failed"
            self.logger.debug("Launch Chrome succ!!")
            chrome.open_website(STREAM_URL)
            retry = 60
            playing = False
            for i in range(retry):
                if uia_d(description = "play").exists:
                    uia_d(description = "play").click()
                time.sleep(1)
                playing = self.d.rpc.isStreamActive("music", 1)
                if playing:
                    self.logger.debug("Online streaming started!")
                    break
            assert playing, "Online streaming failed in %ds"%retry
            self.logger.debug("Play online audio streaming succ!")
            self.mt_call("voice")
            time.sleep(1)
            assert self.d.hangouts._is_in_call(), "Failed to accept incoming VOIP call"
            assert not self.d.rpc.isStreamActive("music", 0), "Streaming still ongoing while in VOIP call"
        finally:
            #self.d.stop_app_am(CHROME)
            chrome.stop_from_am()

    def testVOIPCall_While_PlayingClockAlert_MT(self):
        """
        Verify that clock alarm still playing when the VOIP coming and
        alarm sound changed after VOIP call received.
        1. Add an alarm and wait for alert
        2. Check alarm muted while VOIP call comes.
        3. Check alarm resumed after end VOIP call
        """
        try:
            self.r_d.hangouts.launch_by_am()
            self.d.set_alarm(1)
            assert self.d.wait_alarm(60), "Wait for alarm alert failed. timeout 60s"
            self.d.uia_device.open.notification()
            time.sleep(1)
            self.d.uia_device(text = "Alarm").click()
            time.sleep(1)
            # Firstly check if hint ui is in foreground.
            self.d.skip_use_hints()
            self.r_d.hangouts.search_contact(self.d.hangouts.get_account()[0])
            time.sleep(1)
            self.r_d.hangouts.make_voice_call()
            self.d.hangouts.waitfor_call()
            assert self.d.rpc.isStreamActive("alarm", 0), "Alarm sound muted when hangouts call comes"
            self.logger.debug("Alarm vol: %s"%self.d.rpc.getStreamVolume("alarm"))
            self.d.hangouts.accept_call()
            self.logger.debug("Alarm vol: %s"%self.d.rpc.getStreamVolume("alarm"))
            assert self.d.rpc.isStreamActive("alarm", 0), "Alarm sound muted while in call"
            self.d.hangouts.end_call()
            assert self.d.rpc.isStreamActive("alarm", 0), "Alarm sound muted after call"
        finally:
            self.d.delete_alarms()

    def testVOIPCall_VolumeControl_wsHS_HWVolumeKey(self):
        """
        Verify that volume could be adjust using hardware key during voip call on HS
        1. Connect wsHS
        2. Make a VOIP call
        3. Adjust volume via HS vol key
        """
        try:
            devnode = self.getHsNode()
            self.logger.debug("init HS with %s"%devnode)
            self.hs = Headset(devnode, self.logger)
            self.hs.reset()
            self.hs.plug_in()
            #self.hs.enable_mono_mode()
            self.mo_call("voice")
            self.hs.volume_down()
            self.hs.volume_down()
            time.sleep(1)
            vol_start = self.d.rpc.getStreamVolume("voice_call")
            self.hs.volume_up()
            self.hs.volume_up()
            time.sleep(1)
            vol_end = self.d.rpc.getStreamVolume("voice_call")
            assert vol_end > vol_start,\
                    "Adjust vol by hs vol+ key failed. Before: %d, after: %d"%(vol_start, vol_end)
        finally:
            self.hs.reset()

    def testVOIPCall_VolumeControl_HWVolumeKey(self):
        """
        Verify that user could heard call volume changed by press volume hardware key during VOIP call on IHF
        1. Make a voice call
        2. Change volume by hw key
        """
        VOL_UP = "input keyevent KEYCODE_VOLUME_UP"
        VOL_DOWN = "input keyevent KEYCODE_VOLUME_DOWN"
        self.mo_call("voice")
        start_vol = self.d.rpc.getStreamVolume("voice_call")
        self.d.adb_cmd(VOL_UP)
        self.d.adb_cmd(VOL_UP)
        time.sleep(1)
        #assert self.d.rpc.isStreamActive("system", 3000), "Can't hear vol change"
        current = self.d.rpc.getStreamVolume("voice_call")
        assert current > start_vol,\
                "Vol+ failed. vol before: %d, vol after: %d"%(start_vol, current)
        start_vol = current
        self.d.adb_cmd(VOL_DOWN)
        self.d.adb_cmd(VOL_DOWN)
        time.sleep(1)
        #assert self.d.rpc.isStreamActive("system", 3000), "Can't hear vol change"
        current = self.d.rpc.getStreamVolume("voice_call")
        assert current < start_vol,\
                "Vol- failed. vol before: %d, vol after: %d"%(start_vol, current)

    def testVOIPCall_VideoEnabled_RotateScreen(self):
        """
        Verify that rotation of graphics function works while duriing video VOIP call
        1. Receive a hangouts video call
        2. Rotate screen
        """
        try:
            self.mt_call("video")
            self.d.get_device().freeze_rotation(False)
            self.d.get_device().orientation = "l"
            time.sleep(.5)
            assert self.d.hangouts._is_in_call(), "Failed in Video call while rotate screen: Left"
            self.d.get_device().orientation = "r"
            assert self.d.hangouts._is_in_call(), "Failed in Video call while rotate screen: Right"
            self.d.get_device().orientation = "n"
            assert self.d.hangouts._is_in_call(), "Failed in Video call while rotate screen: Nature"
        finally:
            self.d.get_device().orientation = "n"
            self.d.get_device().freeze_rotation(True)

    def testVOIPCall_Switch_Audio_Video(self):
        """
        VOIP call switch Audio Video
        1. Make an VOIP call
        2. Video mute/unmute
        """
        self.mo_call("video")
        time.sleep(1)
        self.d.skip_use_hints()
        self.d.hangouts.video_mute(enable = True)
        time.sleep(1)
        self.d.hangouts.video_mute(enable = False)

    def testVOIPCall_ReveiveIM_Notfication(self):
        """
        Verify that IM is received and user can't hear the notification sound during VOIP call
        1. Receive a VOIP call
        2. Receive a msg during VOIP call
        """
        MSG = "Test Hangouts message, %f"%time.time()
        self.mt_call("voice")
        self.r_d.get_device().press.back()
        time.sleep(1)
        self.d.back_home()
        #self.d.get_device().press.back()
        #self.r_d.hangouts.launch_by_am()
        self.r_d.hangouts.send_msg(MSG)
        self.d.hangouts.waitfor_msg(text = MSG)
        assert not self.d.rpc.isStreamActive("notification", 3000)

    def testVOIPCall_PlaybackWithEffect(self):
        """
        Verify that audio playback normally with effect on during VOIP call
        1. Start play music
        2. Enable effect
        3. Make VOIP call
        4. Check Music playback
        """
        AUDIO_NAME = "mp3_sample2.mp3"
        DUT_PATH = "/sdcard/otctest/"
        try:
            self.download_push_content(self.d, AUDIO_NAME, DUT_PATH)
            self.mo_call("voice")
            self.d.rpc.enableEffect()
            self.d.rpc.playLong()
            assert self.d.rpc.isStreamActive("music", 5000), "Music not playing in VOIP call"
        finally:
            self.d.adb_cmd("rm -r %s"%DUT_PATH)

    def testVOIPCall_PlaybackMusic(self):
        """
        Verify that music playback well during VOIP call
        1. Play a video
        2. Accept a VOIP call, Video playback paused.
        3. End VOIP call
        4. Start Music playback by GMS music
        5. Accept VOIP call, music should paused.
        6. End VOIP call, music can be resumed
        """
        from testlib.photos.mum_photos_impl import PhotosImpl
        VIDEO = self.config.getConfValue(file_src=self.CONFIG_FILE, section='media_contents', key="file_2")
        AUDIO = self.config.getConfValue(file_src=self.CONFIG_FILE, section='media_contents', key="file_1")
        DUT_PATH = "/sdcard/otctest/"
        DUT_FOLDER = "otctest"
        APP_ACTIVITY = ("com.google.android.apps.photos",
                        "com.google.android.apps.photos.home.HomeActivity")
        photos = PhotosImpl()
        try:
            self.download_push_content(self.d, VIDEO, DUT_PATH)
            self.download_push_content(self.d, AUDIO, DUT_PATH)
            #self.download_install_app(self.d, app)
            self.d.refresh_storage()
            #self.d.launch_app_am(*APP_ACTIVITY)
            photos.set_orientation_n()
            photos.launchPhotos(DUT_FOLDER)
            time.sleep(3)
            uia_d = self.d.get_device()
            # uia_d(className="android.widget.ImageView").click()
            # time.sleep(2)
            # for i in range(3):#Retry 3 times
            #     if uia_d(resourceId = "com.google.android.apps.photos:id/video_current_time").exists:
            #         break
            #     uia_d(resourceId = "com.google.android.apps.photos:id/photo_view").click.wait()
            #     time.sleep(.5)
            for _ in range(3):  # Try 3 times
                if uia_d(descriptionContains="Video").exists:
                    uia_d(descriptionContains="Video").click.wait()
                    time.sleep(2)
                    break
            self.d.skip_use_hints()  # For skipping full screen operation hints.
            assert self.d.rpc.isStreamActive("music", 0), "Video not playing!"

            self.mt_call("voice")
            time.sleep(2)
            assert not self.d.rpc.isStreamActive("music", 0), "Video still playing while in VOIP call"
            self.d.hangouts.end_call()
            time.sleep(3)
            assert uia_d.info.get("currentPackageName") == APP_ACTIVITY[0],\
                    "Back to video playback failed"

            playing = False
            for i in range(4):#Retry 4 times
                time.sleep(1)
                if self.d.rpc.isStreamActive("music", 0):
                    playing = True
                    break
                elif uia_d(description = 'Play video').exists:
                    uia_d(description = 'Play video').click.wait()
                    continue
                else:
                    uia_d(resourceId="com.google.android.apps.photos:id/photo_hashtag_fragment_container").click.wait()
                    continue
            time.sleep(1)
            assert playing, "Resume Video playback failed"

            self.d.stop_app_am(APP_ACTIVITY[0])
            time.sleep(.5)
            self.d.gmusic.launch_music_am()
            self.d.gmusic.enterSongsPage()
            self.d.gmusic.playMusic(AUDIO.split(".")[0])
            self.d.gmusic.setRepeat(2)
            assert self.d.rpc.isStreamActive("music", 0), "Audio playback failed"
            self.mt_call("voice")
            assert not self.d.rpc.isStreamActive("music", 0), "Audio playback not paused while VOIP call"
            self.d.hangouts.end_call()
            time.sleep(1)
            assert self.d.rpc.isStreamActive("music", 0), "Audio not resume after end VOIP call"

        finally:
            self.d.stop_app_am(APP_ACTIVITY[0])
            self.d.gmusic.cleanUpData()

    def testVOIPCall_MT_wsHS(self):
        """
        Verify that MT a VOIP call, audio is route to wsHS when wired headset is connected
        1, GS connnect
        2. Make a VOIP call and check the voice on HS
        """
        symbol = "12345"
        wav = self.make_dtmf_wave(symbol)
        remote_path = os.path.join(self.DUT_PATH, os.path.basename(wav))
        self.r_d.push_file(wav, remote_path)
        self.r_d.refresh_storage()
        self.d.rpc.setStreamVolume("voice_call", 10)
        self.r_d.rpc.setStreamVolume("voice_call", 10)
        try:
            devnode = self.getHsNode()
            self.hs = Headset(devnode, self.logger)
            self.hs.reset()
            self.hs.plug_in()
            #self.hs.enable_mono_mode()
            self.mo_call("voice")
            time.sleep(1)
            self.waitfor(8, True, self.r_d.rpc.isStreamActive,"voice_call", 0)
            time.sleep(3)
            rec_wav = tempfile.mktemp(suffix=".wav")

            self.hs.start_record(rec_wav, rate = 44100)
            self.r_d.rpc.playFile(remote_path, False)
            time.sleep(10)
            self.hs.stop_record()
            time.sleep(1)
            self.assert_dtmf_exists(rec_wav, symbol, similar= 0.8)

        finally:
            self.d.rpc.resetPlayer()
            self.hs.reset()

    def testVOIPCall_MT_While_AudioPlayback(self):
        """
        Verify that audio playback is stoppend when receive a VOIP call
        1. Start audio playback
        2. Receive a MT call and check audio playback
        """
        AUDIO_NAME = "mp3_sample2.mp3"
        DUT_PATH = "/sdcard/otctest/"
        try:
            self.download_push_content(self.d, AUDIO_NAME, DUT_PATH)
            self.d.refresh_storage()

            self.d.gmusic.launch_music_am()
            self.d.gmusic.enterSongsPage()
            self.d.gmusic.playMusic(AUDIO_NAME.split(".")[0])
            self.d.gmusic.setRepeat(2)
            assert self.d.rpc.isStreamActive("music", 0), "Audio playback failed"

            self.mt_call("voice")
            time.sleep(1)
            assert not self.d.rpc.isStreamActive("music", 0), "Music still playing in VOIP call"
            self.d.hangouts.end_call()
            time.sleep(1)
            assert self.d.rpc.isStreamActive("music", 0), "Music not resume after end VOIP call"
        finally:
            self.d.gmusic.cleanUpData()
            self.d.adb_cmd("rm -r %s"%DUT_PATH)

    def testVOIPCall_MO_While_AudioPlayback(self):
        """
        Verify that audio playback is stoppend when receive a VOIP call
        1. Start audio playback
        2. Make a MO call and check audio playback
        """
        AUDIO_NAME = "mp3_sample2.mp3"
        DUT_PATH = "/sdcard/otctest/"
        try:
            self.download_push_content(self.d, AUDIO_NAME, DUT_PATH)

            self.d.gmusic.launch_music_am()
            self.d.gmusic.enterSongsPage()
            self.d.gmusic.playMusic(AUDIO_NAME.split(".")[0])
            self.d.gmusic.setRepeat(2)
            assert self.d.rpc.isStreamActive("music", 0), "Audio playback failed"
            self.d.back_home()

            self.mo_call("voice")
            assert not self.d.rpc.isStreamActive("music", 0), "Music still playing in VOIP call"
        finally:
            self.d.gmusic.cleanUpData()
            self.d.adb_cmd("rm -r %s"%DUT_PATH)

    def testVOIPCall_MO_IHF_Iterative_20cycles(self):
        """
        Verify that user can establish VOIP call on WLAN for 20 times
        """
        cycle = 20
        self.d.hangouts.launch_by_am()
        self.d.hangouts.search_contact(self.r_d.hangouts.get_account()[0])

        for i in range(cycle):
            #time.sleep(2)
            self.d.hangouts.make_voice_call()
            self.r_d.hangouts.waitfor_call()
            self.r_d.hangouts.accept_call()
            time.sleep(1)
            self.d.hangouts.end_call()
            if not self.waitfor(10, False, self.r_d.hangouts._is_in_call):
                raise Exception("Call not ended on reference phone after hangup on test device")
            self.r_d.back_home()
            time.sleep(2)

    def testVOIPCall_MediaVolumeControl_InSetting(self):
        """
        Verify that media volume could be adjusted using settings menu during VOIP call
        1. Make a VOIP call
        2. Adjust vol in settings
        """
        self.mt_call("voice")
        self.d.back_home()
        self.d.launch_sound_settings()
        uia_d = self.d.get_device()
        uia_d(text="Media volume").wait.exists()

        seek_bar = uia_d(text="Media volume").down(resourceId="android:id/seekbar").info["bounds"]
        y = (seek_bar["top"] + seek_bar["bottom"])/2
        start = seek_bar["left"] + 10
        end = seek_bar["right"] -10
        uia_d.click((end - start)/2, y)# volume middle
        time.sleep(1)
        start_vol = self.d.rpc.getStreamVolume("music")
        uia_d.click(end, y)# Volume up
        time.sleep(1)
        current_vol = self.d.rpc.getStreamVolume("music")
        assert current_vol > start_vol,\
            "VOIP call Volume up failed by settings. Before: %d, after: %d"%(current_vol, start_vol)

    def testVOIPCall_LongLasting_1hr(self):
        """
        Verify that user could make 1 hr VOIP call without any error
        """
        timeout = 60*60
        self.mo_call("voice")

        start = time.time()
        while time.time() - start < timeout:
            time.sleep(5)
            assert self.d.hangouts._is_in_call() and self.r_d.hangouts._is_in_call(),\
                    "VOIP call long lasting failed!"
        self.logger.debug("1 hr longlasting finished!!")

    def testVOIPCall_LockUnlockScreenSound(self):
        """
        Verify that user can hear screen lock sound during VOIP call
        1. Enable Screen lock sound in settings
        2. Make a VOIP call
        3. Lock/unlock screen
        """
        self.d.enable_screen_lock()

        self.mo_call("voice")
        assert self.d.hangouts._is_in_call(), "Make VOIP call failed!"
        self.d.check_screen_lock_sound()

    def testVOIPCall_IHF_Mute_Switch_Iterative_20cycles(self):
        """
        Verify that accessory change- Mute-to/from IHF 20 times succ during VOIP call
        """
        cycle = 20
        self.mo_call("voice")
        assert self.d.hangouts._is_in_call(), "Make VOIP call failed!"

        for i in range(cycle):
            self.d.hangouts.audio_mute(enable = True)
            time.sleep(.5)
            self.d.hangouts.audio_mute(enable = False)
            time.sleep(.5)

    def testVOIPCall_HoldUnhold(self):
        """
        Verify that VOIP call hold/unhold function works well
        """
        pass

    def testVOIPCall_BTHS_SwitchOffOn(self):
        """
        Verify that turn off/on BTHS normally and audio route correctly during VOIP call
        1. Connect BTHS
        2. Receive a VOIP call
        3. Check audio route is BT
        4. Disconnect BT
        5. Check audio route is IHF
        6. Connect BT during VOIP call
        7. Check Audio route is BTHS
        """
        self.bt = BTAudioAdapter()
        self.bt.setup()
        TIMEOUT = 8

        try:
            self.logger.debug("Connect BT then receive a MO call")
            self.bt.connect(True)
            self.mo_call("voice")
            assert self.waitfor(TIMEOUT, True, self.bt.sco.is_active),\
                    "BT SCO not enabled during hangouts with BT connected"
            self.logger.debug("Disconnect BT during VOIP call")
            self.bt.adapter.Discoverable = False
            self.bt.adapter.Powered = False
            assert self.waitfor(TIMEOUT, False, self.bt.sco.is_active),\
                    "BT SCO still enabled during hangouts after BT disconnected"
            time.sleep(2)
            self.bt.reset_adapter()
            self.logger.debug("Connect BT during VOIP call")
            self.bt.connect(True)
            self.d.launch_app_from_home_sc("Hangouts")
            time.sleep(1)
            self.waitfor(5, True, self.d.hangouts._is_in_call)
            self.d.hangouts.set_audio_route_bt()
            time.sleep(1)
            assert self.waitfor(TIMEOUT, True, self.bt.sco.is_active),\
                    "BT SCO not enabled after BT connected during hangouts"
        finally:
            self.bt.teardown()

    def testVOIPCall_Accessory_Switch_wsHS_BTHSP(self):
        """
        Verify that accessory change wsHS to/from BT HSP successfully during hangouts VOIP call
        """
        self.bt = BTAudioAdapter()
        self.bt.setup()

        symbol = "12345"
        wav = self.make_dtmf_wave(symbol)
        remote_path = os.path.join(self.DUT_PATH, os.path.basename(wav))
        self.r_d.push_file(wav, remote_path)
        self.r_d.refresh_storage()

        self.d.rpc.setStreamVolume("voice_call", 10)
        self.r_d.rpc.setStreamVolume("voice_call", 10)
        self.r_d.rpc.setStreamVolume("music", 10)
        try:
            devnode = self.getHsNode()
            self.hs = Headset(devnode, self.logger)
            self.hs.reset()
            #Make a VOIP call
            self.mo_call("voice")
            #HS plugin
            self.hs.plug_in()
            self.waitfor(8, True, self.r_d.rpc.isStreamActive,"voice_call", 0)
            #Check audio route is HS
            rec_wav = tempfile.mktemp(suffix=".wav")
            self.hs.start_record(rec_wav, rate = 44100)
            self.r_d.rpc.playFile(remote_path, False)
            time.sleep(8)
            self.hs.stop_record()
            self.assert_dtmf_exists(rec_wav, symbol, similar= 0.8)
            #Connect to BT HSP and tap BT icon
            self.bt.connect(True)
            self.d.launch_app_from_home_sc("Hangouts")
            time.sleep(1)
            self.d.hangouts.set_audio_route_bt()
            time.sleep(1)
            #Check audio route is BT HSP
            assert self.waitfor(8, True, self.bt.sco.is_active),\
                    "BT SCO note active after connect BT during VOIP call"
        finally:
            self.bt.teardown()
            self.hs.reset()

    def testVOIPCall_Accessory_Switch_IHF_wsHS(self):
        """
        Verify that accessory change - IHF to/from wsHS successfully during hangouts VOIP call
        """
        symbol = "12345"
        wav = self.make_dtmf_wave(symbol)
        remote_path = os.path.join(self.DUT_PATH, os.path.basename(wav))
        self.r_d.push_file(wav, remote_path)
        self.r_d.refresh_storage()
        self.d.rpc.setStreamVolume("voice_call", 10)
        self.r_d.rpc.setStreamVolume("voice_call", 10)
        try:
            devnode = self.getHsNode()
            self.hs = Headset(devnode, self.logger)
            self.hs.reset()
            #Make a VOIP call
            self.mo_call("voice")
            #Plug in HS
            self.hs.plug_in()
            self.waitfor(8, True, self.r_d.rpc.isStreamActive,"voice_call", 0)
            #Check audio route is HS
            rec_wav = tempfile.mktemp(suffix=".wav")
            self.hs.start_record(rec_wav, rate = 44100)
            self.r_d.rpc.playFile(remote_path, False)
            time.sleep(8)
            self.hs.stop_record()
            self.assert_dtmf_exists(rec_wav, symbol, similar= 0.8)
        finally:
            self.hs.reset()
            self.d.rpc.resetPlayer()

    def testVOIPCall_Accessory_Switch_IHF_BTHSP(self):
        """
        Verify that accessory can change between IHF and BTHSP successfully during VOIP call
        """
        self.bt = BTAudioAdapter()
        self.bt.setup()

        symbol = "12345"
        sco_wav = self.make_dtmf_wave(symbol, rate = 8000)
        record_wav = "soc_record.wav"
        record_path = "/sdcard/%s"%record_wav
        tmp_dir = tempfile.mkdtemp()
        local_path = os.path.join(tmp_dir, record_wav)
        try:
            #Connect BT HSP
            self.bt.connect(True)
            #Make a VOIP call
            self.mo_call("voice")
            #Speak from BT mic
            self.waitfor(8, True, self.r_d.rpc.isStreamActive,"voice_call", 0)
            self.r_d.rpc.startCallRecording(record_path)
            self.bt.sco.send_wav_file(sco_wav)
            time.sleep(8)
            self.r_d.rpc.stopCallRecording()
            #Check remote device can heard this sound
            time.sleep(2)
            self.r_d.pull_file(local_path, record_path)
            time.sleep(2)
            assert os.path.exists(local_path), "Pull file failed. Not exists: %s"%local_path
            self.assert_dtmf_exists(local_path, symbol, similar = 0.8)
            #Change accessory to IHF
        finally:
            self.bt.teardown()
            self.r_d.adb_cmd("rm %s"%record_path)

    def getHsNode(self):
        """
        Get HS node, /dev/ttyUSB*
        Auto return the first one
        """
        for dev in os.listdir('/dev/'):
            if dev.startswith("ttyUSB"):
                devnode = os.path.join("/dev/", dev)
                break
        else:
            raise Exception("Can't find dev node /dev/ttyUSB*")
        return devnode
