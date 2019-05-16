'''
Created on Feb 24, 2016

@author: bob
'''
from testlib.audio.audio_test_base import TelephonyTestBase
from testlib.audio.helper import dtmf_random_str
import time


class CSV_Call_Test(TelephonyTestBase):
    VOICE_CALL = 'voice_call'
    RING = 'ring'
    ALARM = 'alarm'

    def setUp(self):
        super(CSV_Call_Test, self).setUp()

    def tearDown(self):
        super(CSV_Call_Test, self).tearDown()

    def testST_AUDIOCOMMS_CSV_ARM_001(self):
        '''
        Adjust Alarm volume using settings menu during CSV call
        '''
        self.start_mt_call(answer = True)
        self.d.launch_sound_settings()
        self.d.uia_device(resourceId="android:id/seekbar").wait.exists()

        seek_bar = self.d.uia_device(text="Alarm volume").down(resourceId="android:id/seekbar").info["bounds"]
        y = (seek_bar["top"] + seek_bar["bottom"])/2
        start = seek_bar["left"] + 10
        end = seek_bar["right"] -10
        self.d.uia_device.click((end - start)/2, y)# volume middle
        time.sleep(1)
        start_vol = self.d.rpc.getStreamVolume(self.ALARM)
        self.d.logger.debug("Set vol middle, vol is %d"%start_vol)

        self.d.uia_device.click((end - start)/4, y)# Volume down
        time.sleep(1)
        current_vol = self.d.rpc.getStreamVolume(self.ALARM)
        self.d.logger.debug("Vol down, vol is %d"%current_vol)

        assert current_vol < start_vol,\
            "Volume down failed by settings. Before: %d, after: %d"%(current_vol, start_vol)

        self.d.uia_device.click((end - start)/2, y)#Volume middle
        time.sleep(1)
        last_vol = self.d.rpc.getStreamVolume(self.ALARM)
        self.d.logger.debug("Vol up, vol is %d"%last_vol)
        assert last_vol > current_vol,\
            "Volume up failed by settings. Before: %d, after: %d"%(current_vol, last_vol)

    def testST_AUDIOCOMMS_CSV_DTMF_001(self):
        '''
        Short DTMF tone played to a distant user during voice call
        '''
        self.start_mo_call()
        syms = '0123456789*#'
        # check DTMF send
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar = 0.6)
        # check DTMF recv
        record = self.remote_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar = 0.6)

    def testST_AUDIOCOMMS_CSV_MO_009(self):
        '''
        MO 2G Volume Control earpiece/mono MIC
        '''
        # Initiate a MO CSV call
        self.start_mo_call()
        # check voice can be heard
        syms = dtmf_random_str(5)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar= 0.6)
        # adjust volume level using HW keys
        start = self.d.rpc.getStreamVolume(self.VOICE_CALL)
        self.d.uia_device.press.volume_up()
        self.d.uia_device.press.volume_up()
        time.sleep(1)
        up = self.d.rpc.getStreamVolume(self.VOICE_CALL)
        assert up > start, 'Volume up failed. Before:%d, after: %d'%(start, up)
        self.d.uia_device.press.volume_down()
        self.d.uia_device.press.volume_down()
        time.sleep(1)
        down = self.d.rpc.getStreamVolume(self.VOICE_CALL)
        assert down < up, 'Volume down failed, Before:%d, after: %d'%(up, down)

    def testST_AUDIOCOMMS_CSV_MT_139(self):
        '''
        Receive and answer/mute/unmute a CSV call through Phone App
        '''
        # start a MT call and answer
        self.start_mt_call()
        # mute call
        self.d.telephony.mute_call(True)
        time.sleep(2)
        self.assert_voice_mute()

        # unmute
        self.d.telephony.mute_call(False)
        time.sleep(2)
        assert not self.d.telephony.is_audio_mute(), 'Unmute fails'
        syms = dtmf_random_str(6)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar= 0.7)

    def testST_AUDIOCOMMS_CSV_MT_016(self):
        '''
        MT 2G call IHF
        '''
        # start a MT call and answer
        self.start_mt_call()
        assert self.d.telephony.get_current_route() == 'EARPIECE', \
            'default no route to EARPIECE'
        # enable IHF
        self.d.telephony.speaker_on(True)
        time.sleep(2)
        assert self.d.telephony.get_current_route() == 'SPEAKER', \
            'audio no route to SPEAKER'
        self.assert_audio_active_both_side()

    def testST_AUDIOCOMMS_CSV_MT_402(self):
        '''
        Mute-Hold-Unhold-Unmute MT CSV Call through Phone App on Headphone
        '''
        # start a MT call and answer
        self.start_mt_call()
        # mute the call
        self.d.telephony.mute_call(True)
        time.sleep(2)
        self.assert_voice_mute()
        # hold the call
        self.d.telephony.hold_call(True)
        # check no audio heard in both side
        syms = dtmf_random_str(5)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, exists=False, similar=0.4)
        record = self.remote_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, exists=False, similar=0.4)
        # unhold the call
        self.d.telephony.hold_call(False)
        time.sleep(1)
        # check call still unmute
        self.assert_voice_mute()
        # unmute call, check if call working
        self.d.telephony.mute_call(False)
        time.sleep(2)
        assert not self.d.telephony.is_audio_mute(), 'Unmute fails'

    def testST_AUDIOCOMMS_CSV_MT_003(self):
        """MT 2G call handset Iterative"""
        for i in range(10):
            self.start_mt_call(answer = True)
            assert self.d.telephony.is_in_call(), "Make MT failed, Not in call"
            self.d.telephony.hangup_call()
            time.sleep(2)
            assert not self.d.telephony.is_in_call(), "Huangup call failed"

    def testST_AUDIOCOMMS_CSV_DTMF_010(self):
        """Long DTMF tone play to local user during voice call"""
        self.start_mt_call(answer = True)
        syms = dtmf_random_str(5)
        dtmf = self.remote_press_dtmf(syms, long_click = True)
        time.sleep(1)
        self.d.telephony.hangup_call()
        time.sleep(.5)
        self.assert_dtmf_exists(dtmf, syms, True, similar = 0.6)

    def testST_AUDIOCOMMS_CSV_DTMF_006(self):
        '''
        The purpose is to verify that the handset is able to generate short
        DTMF tone generation during a voice call.
        Iteratively (at least 10 times)
        '''
        count = 1  # indicator for local / remote device
        self.start_mo_call()
        try:
            for i in range(1, 11):
                self.logger.info("Loop: %d" % i)
                syms = dtmf_random_str(5)
                wav = self.local_press_dtmf(syms)
                self.assert_dtmf_exists(wav, syms, exists=True, similar=0.4)
                count += 1
                time.sleep(2)
                wav2 = self.remote_press_dtmf(syms)
                self.assert_dtmf_exists(wav2, syms, exists=True, similar=0.4)
                count += 1
        finally:
            if count > 20:
                self.logger.info("No exception in all loops!")
                return
            tg = "local" if count % 2 == 1 else "remote"
            self.logger.info("Oops, failed in loop %d from %s DTMF!" % (i, tg))

    def testST_AUDIOCOMMS_CSV_DTMF_003(self):
        '''
        Long DTMF tone played to a distant user during voice call
        '''
        self.start_mo_call()
        syms = dtmf_random_str(5)
        wav = self.local_press_dtmf(syms, long_click= True)
        self.assert_dtmf_exists(wav, syms, exists = True, similar = 0.6)
        syms = dtmf_random_str(5)
        wav = self.remote_press_dtmf(syms, long_click= True)
        self.assert_dtmf_exists(wav, syms, exists = True, similar = 0.6)

    def testST_AUDIOCOMMS_CSV_DTMF_002(self):
        '''
        Short DTMF tone played to a local user during voice call
        '''
        self.start_mo_call()
        syms = dtmf_random_str(5)
        wav = self.local_press_dtmf(syms, long_click= False)
        self.assert_dtmf_exists(wav, syms, exists = True, similar = 0.6)
        syms = dtmf_random_str(5)
        wav = self.remote_press_dtmf(syms, long_click= False)
        self.assert_dtmf_exists(wav, syms, exists = True, similar = 0.6)

    def testST_AUDIOCOMMS_CSV_ARM_002(self):
        '''
        Adjust media volume using settings menu during CSV call
        '''
        self.start_mo_call()
        self.d.launch_sound_settings()
        self.d.check_volume_in_settings(mode = 1)

    def testST_AUDIOCOMMS_CSV_MO_063(self):
        '''
        Mute-Unmute MO 3G CSV Call through Phone App on earpiece
        '''
        self.start_mo_call()
        self.d.telephony.mute_call(True)
        time.sleep(.5)
        self.assert_voice_mute()
        self.d.telephony.mute_call(False)
        assert not self.d.telephony.is_audio_mute(), "Unmute telephony failed!"

    def testST_AUDIOCOMMS_CSV_MO_001(self):
        '''
        MO 2G call handset
        '''
        self.start_mo_call()
        self.assert_audio_active_both_side()

    def testST_AUDIOCOMMS_CSV_MO_012(self):
        '''
        MO 3G call IHF
        '''
        self.start_mo_call()
        self.d.telephony.speaker_on(True)
        self.assert_audio_active_both_side()

    def testST_AUDIOCOMMS_CSV_MT_004(self):
        '''
        MT 3G Call handset
        '''
        self.start_mt_call(answer = True)
        self.assert_audio_active_both_side()

    def testST_AUDIOCOMMS_CSV_MO_056(self):
        '''
        Mute-Unmute MO 3G CSV Call through Phone App on IHF
        '''
        self.start_mo_call()
        self.d.telephony.speaker_on(True)
        for i in range(3):
            self.d.telephony.mute_call(True)
            time.sleep(1)
            self.assert_voice_mute()
            self.d.telephony.mute_call(False)
            time.sleep(1)
            syms = dtmf_random_str(5)
            record = self.local_press_dtmf(syms)
            self.assert_dtmf_exists(record, syms, similar=0.6)

    def testST_AUDIOCOMMS_CSV_MO_059(self):
        '''
        Check volume change during MO 3G CSV call on IHF - Screen ON
        '''
        self.start_mo_call()
        self.d.telephony.speaker_on(True)
        self.assert_audio_active_both_side()
        # set voice call volume to middle
        self.logger.info("Set volume to middle")
        for _ in range(15):
            self.d.uia_device.press.volume_up()
        max_vol = self.d.rpc.getStreamVolume(self.VOICE_CALL)  # max volume should be 5, not equal to sys volume.
        middle_vol = (max_vol + 1)/2 if max_vol % 2 == 1 else max_vol/2
        for _ in range(middle_vol-1):
            self.d.uia_device.press.volume_down()
            time.sleep(.5)
        # check if vol is in middle
        cur_vol = self.d.rpc.getStreamVolume(self.VOICE_CALL)
        assert cur_vol == middle_vol, "CSV call isn\'t set to half of the max value."
        time.sleep(2)
        # start loop.
        for i in range(1, 4):  # Loop 3 times
            self.logger.info("Start loop: %d" % i)
            timeout = time.time() + 60  # set time out interruption to 1 min.
            while cur_vol < max_vol:
                self.assert_volume_change_by_HWKeypress(key_event='volume_up')
                cur_vol = self.d.rpc.getStreamVolume(self.VOICE_CALL)  # reset cur_vol
                assert time.time() < timeout, "Volume setup time out."
            self.logger.info("Waiting for 10s.")
            time.sleep(10)
            timeout = time.time() + 60  # reset time out.
            while cur_vol > 1:  # vol in voice call cannot be set to 0.
                self.assert_volume_change_by_HWKeypress(key_event='volume_down')
                cur_vol = self.d.rpc.getStreamVolume(self.VOICE_CALL)  # reset cur_vol
                assert time.time() < timeout, "Volume setup time out."
