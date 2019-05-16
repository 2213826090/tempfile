import time
from testlib.audio.helper import dtmf_random_str
from testlib.audio.audio_test_base import TelephonyTestBase
from testaid.sound import DTMF


class VoiceCallTest(TelephonyTestBase):
    def testVoiceCall_VolumeControl(self):
        '''
        Verity that voice sound could be adjust during voice call
        '''
        self.start_mo_call()
        self.assert_audio_active_both_side()
        rpc = self.d.rpc
        orig_vol = rpc.getStreamVolume("voice_call")
        rpc.setStreamVolume("voice_call", orig_vol/2)
        time.sleep(3)
        vol = rpc.getStreamVolume("voice_call")
        assert orig_vol != vol, "Can't change voice volume"

    def testVoiceCall_DTMF(self):
        '''
        Verify that the DTMF tone work normally during Voice call.
        '''
        self.start_mo_call()
        self.d.telephony.enter_ongoing_call()
        # DUT press button, and remote record
        syms = '12345'
        record = self.local_press_dtmf(syms)
        # now check the results
        self.assert_dtmf_exists(record, syms, exists=True, similar=0.7)

    def testVoiceCall_RingtoneAlert_DuringMusicPlayback(self):
        '''
        Verify that music is stoppd during the Voice call incoming.
        '''
        rpc = self.d.rpc
        # 1. DUT playback music via default music app
        self.gapp_play_music()
        # Music playback normal
        assert rpc.isStreamActive('music', 0), "Muisc not playing"

        # Incoming one voice call
        self.start_mt_call(answer=False)
        # The call incoming and music stopped in background
        assert not rpc.isStreamActive('music', 0), \
            "Music still playing when call incoming"
        # can hear the ringtone normally
        assert rpc.isStreamActive('ring', 0), "Can't hear ringtone"

        # Can answer successfully and voice call work nornally
        self.d.telephony.answer_call()
        self.assert_audio_active_both_side()
        self.d.telephony.hangup_call()

        # After the call ended, the music go on playback.
        time.sleep(5)
        assert rpc.isStreamActive('music', 0), \
            "Muisc not continue playing after call end"

    def testVoiceCall_ClockAlert(self):
        '''
        Verity that clock alert could be hard during voice call
        '''
        self.d.set_alarm(1)  # setup alarm
        try:
            self.start_mt_call()  # wait for incoming call
            self.d.wait_alarm()  # wait alarm sound
            time.sleep(5)
            self.d.telephony.hangup_call()
        finally:
            self.d.delete_alarms()

    def testVoiceCall_SMSArrive(self):
        '''
        Verity that DUT could receive SMS during voice call,
        but can not heard notification during call
        '''
        self.start_mo_call()
        self.recv_sms()
        assert not self.d.rpc.isStreamActive('notification', 5000), \
            "SMS notification heard during call"

    def testVoiceCall_HoldUnhold(self):
        '''
        Verify that can hold and un-hold Voice call successfully.
        '''
        syms = '23456'
        self.start_mo_call()
        # hold call, remote should not hear local sound
        self.d.telephony.hold_call(True)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, exists=False, similar=0.5)
        # unhold, voice should work normally
        self.d.telephony.hold_call(False)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, exists=True, similar=0.8)

    def hs_play_dtmf(self, syms):
        '''
        encode DTMF syms as sound, and play from MIC
        recording in remote side.

        return the recorded file
        '''
        wav = self.d.mktemp_wav(syms)
        DTMF.encode(syms, wav)
        self.r_d.record_call()
        self.hs.play_file(wav)
        time.sleep(1)
        record = self.r_d.stop_record_call()
        return record

    def testVoiceCall_MuteUnmute(self):
        '''
        Verify that user can mute and un-mute Voice Call.
        '''
        #self.hs.plug_in()
        #time.sleep(3)
        self.start_mo_call()
        self.assert_audio_active_both_side()
        # mute call
        self.d.telephony.mute_call(True)
        time.sleep(1)
        self.assert_voice_mute()
        # umute
        self.d.telephony.mute_call(False)
        time.sleep(1)
        assert not self.d.telephony.is_audio_mute(), 'Unmute not success'

    def __testVoiceCall_MusicPlayback(self):
        '''
        Verify that music playback normally during voice call
        '''
        rpc = self.d.rpc
        self.start_mo_call()
        # check if audio is OK under music playback
        rpc.playLong()
        self.assert_audio_active_both_side()

        rpc.resetPlayer()
        # check if music can be heard in local side
        # record local sound from MIC
        # (need set speaker on to have better record)
        self.d.telephony.speaker_on(True)
        syms = dtmf_random_str(15)
        wav = self.d.mktemp_wav()
        DTMF.encode(syms, wav)
        self.d.record_mic()
        self.d.play_file(wav)
        record = self.d.stop_record_mic()
        self.assert_dtmf_exists(record, syms, similar=0.5)
