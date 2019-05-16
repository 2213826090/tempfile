import os
import time
import shutil
import tempfile
from testlib.audio import resource
from testlib.audio.helper import get_hs_devnode, run_image_case, dtmf_random_str
from testlib.audio.recorder_impl import RecorderImpl
from testlib.audio.audio_test_base import AudioStubTestBase
from testlib.audio.decorator import use_bt
from testaid.sound import WaveGenerator, WaveHelper
from testaid.headset import Headset


class HeadsetTest(AudioStubTestBase):
    def setUp(self):
        super(HeadsetTest, self).setUp()
        self.tmpfiles = []
        self.tmpfiles_dut = []
        devnode = get_hs_devnode()
        self.hs = Headset(devnode, self.logger)
        self.hs.reset()

    def tearDown(self):
        for f in self.tmpfiles:
            os.system("rm -rf " + f)
        for f in self.tmpfiles_dut:
            self.adb.adb_cmd("rm " + f)
        self.hs.reset()
        super(HeadsetTest, self).tearDown()

    def _assert_freq(self, exists, wavfile, freq, errmsg, channel, start):
        try:
            if exists:
                pos = WaveHelper.find_freq(wavfile, freq,
                                           start=start, channel=channel)
                assert pos >= 0, errmsg
            else:
                pos = WaveHelper.find_sound(wavfile, start=start)
                if pos >= 0:  # make sure we have sound first before check freq
                    pos = WaveHelper.find_freq(wavfile, freq,
                                               start=pos, channel=channel)
                    assert pos < 0, errmsg
        except:
            if os.path.exists(wavfile):
                fname = os.path.basename(wavfile)
                fpath = os.path.join(self.get_log_dir(), fname)
                self.logger.info("save wave to " + fpath)
                shutil.copy(wavfile, fpath)
            raise
        return pos

    def assert_freq_exist(self, wavfile, freq, errmsg, channel=0, start=1):
        '''
        assert 'freq' exist in 'wavfile'

        params:
            wavfile: .wav file to check
            freq: frequency in Hz to search
            errmsg: message to display if fails
            channel: search on which channel
            start: start position in sec to search

        return pos if not exception happens
        '''
        return self._assert_freq(True, wavfile, freq, errmsg, channel, start)

    def assert_freq_not_exist(self, wavfile, freq, errmsg, channel=0, start=1):
        '''
        assert "freq" not exists in 'wavfile'

        the inverse version of *assert_freq_exist*
        '''
        return self._assert_freq(False, wavfile, freq, errmsg, channel, start)

    def mktemp_wav(self):
        wav = tempfile.mktemp(suffix=".wav")
        self.tmpfiles.append(wav)
        return wav

    def hs_mic_play(self, freq, duration=120):
        '''
        play wav through headset mic

        params:
            freq: frequency of wav
            duration: wav file duration
        '''
        wav = self.mktemp_wav()
        wg = WaveGenerator(32000, 1, duration)
        wg.add_sin(0, freq, 0, duration)
        wg.save(wav)
        self.hs.start_play(wav)

    def hs_record(self, duration, rate=16000):
        '''
        record audio from headset

        params:
            duration: record duration in seconds
            rate: sampling rate

        return: record file path
        '''
        wav = self.mktemp_wav()
        self.hs.start_record(wav)
        time.sleep(duration)
        self.hs.stop_record()
        WaveHelper.resample(wav, rate)
        return wav

    def dut_play(self, left, right=None, duration=120, stream="music"):
        '''
        generate wave file and start play in DUT

        params:
            left: left channel frequency in Hz
            right: right channel frequency in Hz,
                if not set, means mono audio (left channel only)
            duraton: wav file duration in seconds
            stream: play wav on which stream
        '''
        # generate wave file
        wav = self.mktemp_wav()
        channels = 2 if right is not None else 1  # mono or stereo
        rate = 16000
        wg = WaveGenerator(rate, channels, duration)
        wg.add_sin(0, left, 0, duration)
        if channels == 2:
            wg.add_sin(1, right, 0, duration)
        wg.save(wav)

        # push to dut and play
        dut_path = "/sdcard/" + os.path.basename(wav)
        self.tmpfiles_dut.append(dut_path)
        self.adb.push_file(wav, dut_path)
        self.rpc.playFileOnStream(dut_path, stream)

    def trigger_notify(self, freq, noti_type, duration=120):
        wav = self.mktemp_wav()
        rate = 16000
        channels = 1
        wg = WaveGenerator(rate, channels, duration)
        wg.add_sin(0, freq, 0, duration)
        wg.save(wav)

        # push to dut and play
        dut_path = "/sdcard/" + os.path.basename(wav)
        self.tmpfiles_dut.append(dut_path)
        self.adb.push_file(wav, dut_path)

        if noti_type == "alarm":
            self.rpc.triggerNotificationAlarm(dut_path)

    def dut_record(self, duration, rate=16000, channels=1):
        '''
        record a duration of wav in dut

        params:
            duration: record duration in seconds, if equal 0, means async mode
            rate: sampling rate
            channels: record channel

        return: record file saved in PC if in sync mode,
                else in async mode return file path in DUT
        '''
        wav = self.mktemp_wav()

        dut_wav = "/sdcard/" + os.path.basename(wav)
        self.tmpfiles_dut.append(dut_wav)
        self.rpc.startWavRecording(dut_wav, rate, channels)
        if duration != 0:
            time.sleep(duration)
            self.rpc.stopWavRecording()

            self.adb.pull_file(wav, dut_wav)
            return wav
        else:
            return dut_wav

    def _dut_play_hs_record(self, left, right=None, duration=15, mono=False):
        '''
        plug headset, dut play wav file and record from headset

        return the recod file
        '''
        if mono:
            self.hs.enable_mono_mode()  # mono
        else:
            self.hs.disable_mic()  # stereo
        self.hs.plug_in()
        time.sleep(2)

        self.dut_play(left=left, right=right)
        time.sleep(2)
        record = self.hs_record(duration)
        self.rpc.resetPlayer()
        return record

    def assert_dtmf_exists(self, wavfile, ref_sym, exists=True,
                           similar=1.0, msg=''):
        '''
        decode DTMF code from wavfile, and compare to reference symbol

        params:
            wavfile: .wav file to extract DTMF code
            ref_sym: a string of reference symbols
            exists: whether dmtf should exists
            similar: similarity, 1.0 means fully matched to ref symbols
        '''
        from testaid.sound import DTMF

        decoded = DTMF.decode(wavfile, gap=0.3)  # min gap 0.3s between two tone
        result = DTMF.similar(ref_sym, decoded)
        err_msg = '\n'.join([msg,
                             "check DTMF fails:",
                             "ref sym: %s" % ref_sym,
                             "exists: %s" % exists,
                             "decoded sym: %s" % decoded,
                             "similar: %s vs %s (target)" % (result, similar),
                             "wav file: %s" % os.path.basename(wavfile)]
                            )
        try:
            if exists:
                assert result >= similar, err_msg
            else:
                assert result < similar, err_msg
        except:
            if os.path.exists(wavfile):
                fname = os.path.basename(wavfile)
                fpath = os.path.join(self.get_log_dir(), fname)
                self.logger.info("save DTMF wav to " + fpath)
                shutil.copy(wavfile, fpath)
            raise

    def dut_play_dtmf(self, syms, effect=False, async=False):
        from testaid.sound import DTMF

        wav = self.mktemp_wav()
        DTMF.encode(syms, wav)

        # push to dut and play
        dut_path = "/sdcard/" + os.path.basename(wav)
        self.tmpfiles_dut.append(dut_path)
        self.adb.push_file(wav, dut_path)
        self.rpc.resetPlayer()
        self.rpc.playFile(dut_path, False)
        if effect:
            self.rpc.enableEffect()
        if not async:  # wait play done
            while self.rpc.isPlaying():
                time.sleep(1)

    def _test_speaker(self):
        freq = 600
        self.dut_play(freq)
        time.sleep(3)

        record_file = self.dut_record(15)
        self.rpc.resetPlayer()  # stop playing annoying sound

        self.assert_freq_exist(record_file, freq, "speaker has no sound")

    def testAudioDecode_Left_Right_Channel(self):
        '''Verify that left/right channel music can route correct on wsHS'''
        left = 900
        right = 500
        record = self._dut_play_hs_record(left=left, right=right)

        # check result, left channel
        self.assert_freq_exist(record, left,
                               "can't find Freq %dHz in left channel" % left,
                               channel=0)
        self.assert_freq_not_exist(record, right,
                                   "right channel mixed to left channel",
                                   channel=0)
        # right channel
        self.assert_freq_exist(record, right,
                               "can't find Freq %dHz in right channel" % right,
                               channel=1)
        self.assert_freq_not_exist(record, left,
                                   "left channel mixed to right channel",
                                   channel=1)

    def testAudioDecode_Mono_Stereo(self):
        '''Verify that Audio can Playback Mono To Stereo Upmix'''
        freq = 800
        record = self._dut_play_hs_record(freq)

        # check result
        for c in [0, 1]:
            self.assert_freq_exist(record, freq,
                                   "can't find freq %dHz in channel %d" % (freq, c),  # noqa
                                   channel=c)

    def testAudioDecode_Stereo_Mono(self):
        '''
        Verify that Audio can Playback Stereo To Mono Downmix
        '''
        freq = 800
        record = self._dut_play_hs_record(freq, freq, mono=True)
        self.assert_freq_exist(record, freq,
                               "can't find freq %dHz in downmix" % freq)

    def testAudioDecode_LPCM_on_StereoHeadPhone(self):
        '''
        Verify audio playback with LPCM on StereoHeadPhone
        '''
        freq = 800
        record = self._dut_play_hs_record(freq)

        # check result
        for c in [0, 1]:
            self.assert_freq_exist(record, freq,
                                   "can't find freq %dHz in channel %d" % (freq, c),  # noqa
                                   channel=c)

    def testwmHSDecode(self):
        '''Verify that music can decode in wire mono headset (wsHS)'''
        freq = 800
        record = self._dut_play_hs_record(freq, mono=True)

        # check
        self.assert_freq_exist(record, freq,
                               "Can't find freq %dHz in mono record" % freq)

    def testRingtone_BothOnIHFwsHS(self):
        '''
        Verify that ringtone playback on both wsHS and speaker
        at the same time when wsHS is connecting.
        '''
        self.hs.plug_in()
        time.sleep(2)
        freq = 800
        self.dut_play(freq, stream="ring")  # ring tone stream
        time.sleep(2)
        hs_rec = self.hs_record(10)  # record from headset
        # ringtone volume from IHF is a little low in this situation, volume up
        vol = self.rpc.getStreamVolume("ring")
        self.rpc.setStreamVolume("ring", vol*2)
        time.sleep(1)
        ihf_rec = self.dut_record(10)  # record from IHF
        self.rpc.resetPlayer()  # stop annoying sound

        # check result
        self.assert_freq_exist(hs_rec, freq, "Can't find ringtone in headset")
        self.assert_freq_exist(ihf_rec, freq, "Can't find ringtone in IHF")

    def testEffectPlayback_AccessoryChange_IHF_Headset(self):
        '''
        Verify that accessary change between IHF and headset is correct
        during the music playback with audio effect.
        '''
        self._playback_acc_change_IHF_headset(effect=True)

    def _playback_acc_change_IHF_headset(self, effect=False):
        def assert_hs_active(active, errmsg):
            syms = dtmf_random_str(10)
            wav = self.mktemp_wav()
            self.hs.start_record(wav, rate=44100)
            self.dut_play_dtmf(syms, effect=effect)
            self.hs.stop_record()

            if active:
                self.assert_dtmf_exists(wav, syms, exists=True,
                                        similar=0.6, msg=errmsg)
            else:
                self.assert_dtmf_exists(wav, syms, exists=False,
                                        similar=0.2, msg=errmsg)

        def assert_ihf_active(active, errmsg):
            syms = dtmf_random_str(10)
            dut_wav = self.dut_record(0)
            self.dut_play_dtmf(syms, effect=effect)
            self.rpc.stopWavRecording()
            record = self.mktemp_wav()
            self.adb.pull_file(record, dut_wav)

            if active:
                self.assert_dtmf_exists(record, syms, exists=True,
                                        similar=0.5, msg=errmsg)
            else:
                self.assert_dtmf_exists(record, syms, exists=False,
                                        similar=0.3, msg=errmsg)

        # should disable mic, so it will record from DUT's mic
        self.hs.disable_mic()
        self.hs.plug_in()
        time.sleep(5)
        assert_hs_active(True, "Headset has no sound")
        assert_ihf_active(False, "IHF has music while headset in")

        # plug out headset
        self.hs.plug_out()
        time.sleep(5)
        assert_ihf_active(True, "IHF has no sound aftet headset out")

        # plug in again
        self.hs.plug_in()
        time.sleep(5)
        assert_hs_active(True, "Headset has no sound")
        assert_ihf_active(False, "IHF has music while headset in")

    @use_bt
    def _do_acc_change_headset_bta2dp(self, stereo=True, effect=False):
        def assert_hs_active(active, errmsg):
            hs_rec = self.hs_record(10)
            if active:
                self.assert_freq_exist(hs_rec, freq, errmsg)
            else:
                self.assert_freq_not_exist(hs_rec, freq, errmsg)

        bt = self.bt
        if not stereo:
            self.hs.enable_mono_mode()
        self.hs.plug_in()
        time.sleep(2)
        freq = 600
        self.dut_play(freq, duration=60*6)
        if effect:
            self.rpc.enableEffect()
        time.sleep(3)
        assert_hs_active(True, "Headset has no music")

        # connect to BT, music should active
        bt.connect(True)
        time.sleep(5)
        assert bt.a2dp_sink.is_active(), "A2DP not active"
        # headset should not active
        assert_hs_active(False, "headset is still active after BT connect")

        bt.connect(False)  # disconnect BT, bt wav saved after disconnect
        time.sleep(5)
        bt_wav = bt.a2dp_sink.get_last_wav()  # check bt wav
        self.assert_freq_exist(bt_wav, freq, "Music not route to A2DP")

        # now audio should route to headset again after BT disconnect
        assert_hs_active(True, "Headset has no music after BT disconnect")

    def testEffectPlayback_AccessoryChange_Headset_BTA2DP(self):
        '''
        Verify that accessary change between headset and BT A2DP
        is correct during the music playback with audio effect.
        '''
        self._do_acc_change_headset_bta2dp(effect=True)

    def testAudioDecodeRouteChange_wsHS_BTA2DP(self):
        '''
        Verify that can change accessory between wsHS BT A2DP
        during music playback
        '''
        self._do_acc_change_headset_bta2dp(effect=False)

    def testAudioDecodeRouteChange_IHF_wsHS(self):
        '''
        Verify that can change accessory between IHF wsHS during music playback
        '''
        self._playback_acc_change_IHF_headset()

    def testPlayingRingtone_DisconnectWiredHeadset(self):
        '''
        Verify that ringtone playing well and smoothly
        after disconnecting wired stereo headset.
        '''
        # plug headset, play ringtone
        self.hs.plug_in()
        time.sleep(2)
        freq = 600
        self.dut_play(freq, stream="ring")  # ring tone stream
        time.sleep(2)

        # disconnect headset, check ringtone on IHF
        self.hs.plug_out()
        record = self.dut_record(10)
        self.rpc.resetPlayer()  # stop playing
        # check result
        self.assert_freq_exist(record, freq,
                               "ringtone not found on IHF after HS disconnect")

    def testStreaming_Http_Live_Audio_wsHS(self):
        self.hs.plug_in()
        time.sleep(2)
        self.rpc.playStream()
        time.sleep(3)
        assert self.rpc.isStreamActive("music", 0), \
            "stream music not playing on HS"

    def _play_heaac_5_1ch_audio(self):
        fname = "HE_AAC_32kHz_5.1ch_6min_550Hz_sinewave.mp4"
        fpath = resource.get_media_content(fname)
        dut_path = "/sdcard/" + fname
        self.tmpfiles_dut.append(dut_path)
        self.adb.push_file(fpath, dut_path)
        self.rpc.playFile(dut_path)

    def testAudioDecode_5_1Channel_Stereo_Mono(self):
        '''
        Verify that DUT can play audio from 5.1 channel to stereo/mono
        '''
        self._play_heaac_5_1ch_audio()
        freq = 550
        # stereo mode
        self.hs.plug_in()
        time.sleep(2)
        hs_rec = self.hs_record(10)
        self.assert_freq_exist(hs_rec, freq, "not route to left channel",
                               channel=0)
        self.assert_freq_exist(hs_rec, freq, "not route to right channel",
                               channel=1)

        self.hs.plug_out()
        # now mono mode
        self.hs.reset()
        self.hs.enable_mono_mode()
        self.hs.plug_in()
        time.sleep(2)
        hs_rec = self.hs_record(10)
        self.assert_freq_exist(hs_rec, freq, "not route to mono channel")

    @use_bt
    def testAudioDecode_RouteChange_IHF_wsHS_BT_HEAAC_5_1Channel(self):
        '''
        Verify that route can change between IHF, wsHS and BT
        during playing he aac 5.1 channel music
        '''
        def check_record(record, route):
            '''
            check target freq is in record

            params:
                record: record file
                route: audio route name
            '''
            f = 550
            self.assert_freq_exist(record, f, "Can't route music to " + route)

        bt = self.bt
        # set volume first, it's to noisy
        vol = self.rpc.getStreamVolume("music")
        self.rpc.setStreamVolume("music", vol*2/3)

        self._play_heaac_5_1ch_audio()
        time.sleep(2)
        # check IHF
        record = self.dut_record(10)
        check_record(record, "IHF")

        # check headset
        self.hs.plug_in()
        time.sleep(2)
        record = self.hs_record(10)
        check_record(record, "Headset")

        # check BT A2DP
        bt.connect(True)
        time.sleep(5)
        assert bt.a2dp_sink.is_active(), "A2DP is not active"
        time.sleep(5)
        bt.connect(False)
        bt_wav = bt.a2dp_sink.get_last_wav()
        check_record(bt_wav, "BT A2DP")

    @use_bt
    def testPlayback_MuteAccessorySwitch_wsHSBluetoothSpeaker(self):
        '''
        Verify that audio accessory is right when Mute is switched to ON
        and switch between wsHS,BT Headset and IHF

        '''
        freq = 500
        bt = self.bt

        # IHF, The audio cannot be heard.
        self.dut_play(freq, duration=60*6)

        # first, we make sure BT/HS music volume is normal
        self.hs.plug_in()  # HS
        time.sleep(5)
        self.rpc.initEnv()
        self.logger.info("HS music vol: %s" % self.rpc.getStreamVolume("music"))
        self.hs.plug_out()
        bt.connect(True)  # BT
        self.rpc.initEnv()
        self.logger.info("BT music vol: %s" % self.rpc.getStreamVolume("music"))
        bt.connect(False)

        # mute music first
        for _ in range(10):
            self.d.press.volume_down()
            time.sleep(0.5)

        time.sleep(3)
        record = self.dut_record(10)  # IHF
        self.assert_freq_not_exist(record, freq, "Mute fail on IHF")

        # headset, The audio can be heard from wired headset
        self.hs.plug_in()
        time.sleep(5)
        self.logger.info("HS music vol: %s" % self.rpc.getStreamVolume("music"))
        record = self.hs_record(10)
        self.assert_freq_exist(record, freq, "HS: The audio can't be heard")

        # BT, The audio can be heard from BT HS.
        bt.connect(True)
        self.logger.info("BT music vol: %s" % self.rpc.getStreamVolume("music"))
        time.sleep(10)
        bt.connect(False)
        bt_wav = bt.a2dp_sink.get_last_wav()
        self.assert_freq_exist(bt_wav, freq, "BT: the audio can't be heard")

    @use_bt
    def testScreenLockSound_wsHS_While_BTA2DPConnected(self):
        '''
        Verify that screen lock sound on wsHS while BTA2DP connected.

        Make sure sound could output from BT A2DP.
        '''
        self.enable_screen_lock()
        self.hs.plug_in()  # plug in headset
        time.sleep(2)

        bt = self.bt
        bt.connect(True)  # connect BT
        time.sleep(3)

        self.check_screen_lock_sound()
        # check on a2dp
        bt.a2dp_sink.get_last_wav()
        bt.connect(False)

    def testPlayback_wsHS_VolumeControl_HWVolumeKeys(self):
        '''
        Verify that volume could be changed by pressing HW volume keys
        During Music Playback on wsHS
        '''
        self.hs.plug_in()
        freq = 500
        time.sleep(2)
        self.dut_play(freq)
        cur_vol = self.rpc.getStreamVolume("music")
        time.sleep(2)

        self.d.press.volume_down()
        time.sleep(2)
        dn_vol = self.rpc.getStreamVolume("music")
        assert dn_vol < cur_vol, "HW Vol- can't work"

        cur_vol = dn_vol
        self.d.press.volume_up()  # press volume up
        time.sleep(2)
        up_vol = self.rpc.getStreamVolume("music")
        assert up_vol > cur_vol, "HW Vol+ can't work"

        # check music is still playing
        record = self.hs_record(10)
        self.assert_freq_exist(record, freq,
                               "Music not playing after HW volume key")

    def testPlayback_wsHS_VolumeControl_InSettings(self):
        self.hs.plug_in()
        freq = 900
        time.sleep(2)
        self.dut_play(freq)
        self.rpc.getStreamVolume("music")
        time.sleep(2)

        self.check_volume_in_settings()

        # check music is still playing
        record = self.hs_record(10)
        self.assert_freq_exist(record, freq,
                               "Music not playing after HW volume key")

    def testAudioEncode_HeadsetMic(self):
        '''
        Verify that can encode via headset mic
        '''
        self.hs.plug_in()
        time.sleep(2)
        freq = 160
        self.hs_mic_play(freq)
        record = self.dut_record(15)
        self.hs.stop_play()  # stop mic playing
        self.assert_freq_exist(record, freq, "Can't record from headset Mic")

    def testAudioEncode_AccessorySwitch_IHF_wsHS(self):
        '''
        Verify that sound can encode by DUT'S mic and wsHS's mic.
        '''
        hs_freq = 800
        ihf_freq = 500

        self.hs.plug_in()
        time.sleep(2)

        duration = 60*5
        self.hs_mic_play(hs_freq, duration=duration)
        time.sleep(2)

        dut_wav = self.dut_record(0)  # async record
        time.sleep(15)  # record 15s in headset mic
        self.hs.stop_play()  # stop mic play
        time.sleep(2)

        self.hs.plug_out()  # plug out headset
        time.sleep(2)
        self.dut_play(ihf_freq, duration=duration)
        time.sleep(15)  # record 15s in DUT mic
        self.rpc.stopWavRecording()  # stop record
        self.rpc.resetPlayer()  # stop playing in IHF

        record = self.mktemp_wav()
        self.adb.pull_file(record, dut_wav)

        pos = self.assert_freq_exist(record, hs_freq, "Can't record from Mic")
        self.assert_freq_exist(record, ihf_freq,
                               "Can't record from DUT mic after HS out",
                               start=pos)

    def testAudioEncode_Headset_Foreground_Background_Clock(self):
        '''
        Verify that clock alarm can play
        while recording voice(forground+background) from Headset micro
        '''
        self.hs.plug_in()
        time.sleep(2)

        psi = RecorderImpl()
        psi.install(force=True)
        psi.force_stop()

        psi.launch()
        time.sleep(5)
        try:

            psi.start_record()  # start record in PSI
            time.sleep(5)

            # trigger alarm
            alarm_freq = 400
            self.trigger_notify(alarm_freq, "alarm")
            time.sleep(5)
            record = self.hs_record(10)
            self.assert_freq_exist(record, alarm_freq,
                                   "can't hear alarm from headset \
                                   (foreground record)")

            self.d.press.home()  # goto home, background record
            time.sleep(3)
            record = self.hs_record(10)
            self.assert_freq_exist(record, alarm_freq,
                                   "can't hear alarm from headset \
                                   (background record)")
        finally:
            psi.force_stop()

    def testChange_VolumeUp_VolumeDown_HeadsetButtons(self):
        '''
        Verify that DUT could change volume using headset volume up/down key
        '''
        self.hs.plug_in()
        time.sleep(2)

        # some platforms, keyevent only works on screen on
        self.d.wakeup()
        time.sleep(0.5)
        # volume up
        cur_vol = self.rpc.getStreamVolume("ring")
        self.hs.volume_up()  # pop up volume UI
        time.sleep(2)
        self.hs.volume_up()  # change volume
        time.sleep(2)
        vol = self.rpc.getStreamVolume("ring")
        assert vol > cur_vol, "headset volume up can't work"

        self.d.wakeup()
        time.sleep(0.5)
        # volume down
        cur_vol = vol
        self.hs.volume_down()
        time.sleep(2)
        self.hs.volume_down()  # change volume
        time.sleep(2)
        vol = self.rpc.getStreamVolume("ring")
        assert vol < cur_vol, "headset volume up can't work"

    def testPlaybackPause_ResumeControl_HeadsetHookKey(self):
        '''
        Verify that headset hook key
        could control music pause and resume correctly.
        '''
        #self.hs.plug_in()
        self.deploy_play_music_content()
        self.hs.plug_in()
        self.launch_play_music_n_play("mp3_sample2")
        time.sleep(3)
        assert self.audio.isMusicPlaying() is True, "Music not playing"
        # press media_key, music should pause
        self.hs.media_key(0.5)
        time.sleep(3)
        assert self.audio.isMusicPlaying() is False, \
            "Music not pause by hook key"
        # press again, music should resume
        self.hs.media_key(0.5)
        time.sleep(3)
        assert self.audio.isMusicPlaying() is True, \
            "Music not resume by hook key"

    def testLaunchGoogleSearch_LongPressHeadsetHookKey(self):
        '''
        Verify that Google search could be launch
        by long press headset hook key.
        '''
        time.sleep(2)
        self.hs.plug_in()
        time.sleep(2)
        pkg = "com.google.android.googlequicksearchbox"
        self.hs.media_key(2)  # press hook key for 2 second
        time.sleep(3)
        cur_pkg = self.d.info['currentPackageName']
        assert pkg == cur_pkg, "long press hook key can't launch google search"
        self.d.press.home()

    def testScreenLockSound_wsHS(self):
        '''
        Verify that screen lock sound on wsHS
        '''
        self.hs.plug_in()
        time.sleep(2)
        self.enable_screen_lock()

        # now start testing
        record = self.mktemp_wav()
        self.hs.start_record(record)
        time.sleep(5)
        self.check_screen_lock_sound()
        self.hs.stop_record()

        # check result
        pos = WaveHelper.find_sound(record, start=3,
                                    duration=0.02, threshold=10)
        self.save_file(record)
        assert pos > 0, "Can't hear screen lock sound in wsHS"

    def testKeypressTone_Headset(self):
        '''
        Verify that user could heard the keypress tone on headset.
        '''
        self.hs.plug_in()
        time.sleep(2)

        # enable sound first
        self.launch_sound_settings()
        self.scroll_n_click("Other sounds")
        self.set_option("Touch sounds", True)

        record = self.mktemp_wav()
        self.hs.start_record(record)
        time.sleep(3)
        self.d(text="Other sounds").left().click()
        time.sleep(3)
        self.hs.stop_record()

        # check result
        pos = WaveHelper.find_sound(record, start=2,
                                    duration=0.02, threshold=10)
        self.save_file(record)
        assert pos > 0, "Can't hear screen lock sound in wsHS"

    def testPlayback_ColckAlarmAlert_HS(self):
        '''
        Verify that clock alarm alert could be heard
        when playing music on HS(stereo,mono,with microphone)
        '''
        self.hs.plug_in()
        time.sleep(2)

        freq_music = 1000
        freq_alarm = 600
        self.dut_play(freq_music)
        time.sleep(5)
        self.trigger_notify(freq_alarm, "alarm")  # trigger alarm
        time.sleep(2)
        record = self.hs_record(15)

        # check result
        self.assert_freq_exist(record, freq_alarm,
                               "Can't hear alarm when music playing")

        assert self.rpc.isPlaying(), "Music not playing when alarm active"

    def testAudioDecode_Stereo_VirtualSurround_ConnectBT(self):
        '''
        Verify that Virtual surround effect is effective
        when Stereo local audio playback via stereo headphone
        and route can change to BT after connecting BT headset
        '''
        self._do_acc_change_headset_bta2dp(stereo=True, effect=True)

    def _wsHS_connect_disconnect(self, delay, count):
        '''
        connect/disconnect wired HS iterative
        during DUT idle/music playback/memo record
        '''
        # idle
        for _ in range(count):
            self.hs.plug_in()
            time.sleep(delay)
            self.hs.plug_out()
            time.sleep(delay)
        # music playback
        self.rpc.playLong()
        for _ in range(count):
            self.hs.plug_in()
            time.sleep(delay)
            assert self.rpc.isStreamActive("music", 500), \
                "music not active when headset plug in"
            self.hs.plug_out()
            time.sleep(delay)
            assert self.rpc.isStreamActive("music", 500), \
                "music not active when headset plug out"
        self.rpc.resetPlayer()
        # record
        memo_name = 'AMR-NB_mono_8KHz_12.2Kbps.amr'
        memo_record = resource.get_media_content(memo_name)
        dut_path = '/sdcard/' + memo_name
        self.adb.push_file(memo_record, dut_path)
        self.rpc.playFile(dut_path)
        for _ in range(count):
            self.hs.plug_in()
            time.sleep(delay)
            assert self.rpc.isStreamActive("music", 500), \
                "memo not active when headset plug in"
            self.hs.plug_out()
            time.sleep(delay)
            assert self.rpc.isStreamActive("music", 500), \
                "memo not active when headset plug out"

    def testAudioDecode_wsHS_Connect_Disconnect_Quickly_20Times(self):
        '''
        Verify that can connect/disconnect wired HS quickly iterative
        during DUT idle/music playback/memo record
        '''
        self._wsHS_connect_disconnect(1, count=20)

    def testAudioDecode_wsHS_Connect_Disconnect_Slowly_20Times(self):
        '''
        Verify that can connect/disconnect wired HS solowly iterative
        during DUT idle/music playback/memo record
        '''
        self._wsHS_connect_disconnect(10, count=20)

    def testAudioDecode_Headset_EditePicture(self):
        '''
        Verify that music play normaly on Headset when editing a picture
        '''
        c = "Edit.image_api_test.ImageAPITest.testImage_View_Edit_JPG"
        self.hs.plug_in()
        time.sleep(3)
        self.dut_play(500)
        run_image_case(c)
        time.sleep(3)
        assert self.rpc.isStreamActive("music", 500), \
            "music not playing when edit pictures"

    def testAudioDecode_Headset_BrowsePicture(self):
        '''
        Verify that music play normaly on Headset
        when opening and browsing pictures
        '''
        c = "Decode.image_api_test.ImageAPITest.testImage_View_RotatePicture"
        self.hs.plug_in()
        time.sleep(3)
        self.dut_play(500)
        run_image_case(c)
        time.sleep(3)
        assert self.rpc.isStreamActive("music", 500), \
            "music not playing when browse pictures"

    def do_effect_test_wsHS(self, effect):
        self.hs.plug_in()
        time.sleep(3)
        freq = 500
        self.dut_play(freq)
        self.rpc.enableEffect(effect)
        time.sleep(10)
        assert self.rpc.isPlaying(), \
            "Music not playing after enable effect: " + effect
        record = self.hs_record(10)
        self.assert_freq_exist(record, freq, "can't route to HS")

    def testAudioEffect_Equalizer_wsHS(self):
        self.do_effect_test_wsHS("Equalizer")

    def testAudioEffect_BassBoost_wsHS(self):
        self.do_effect_test_wsHS("BassBoost")

    def testAudioEffect_EnvironmentalReverb_wsHS(self):
        self.do_effect_test_wsHS("EnvironmentalReverb")

    def testAudioEffect_PresetReverb_wsHS(self):
        self.do_effect_test_wsHS("PresetReverb")

    def testAudioEffect_Virtualizer_wsHS(self):
        self.do_effect_test_wsHS("Virtualizer")
