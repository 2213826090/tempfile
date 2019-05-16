'''
Created on Feb 23, 2016

@author: bob
'''

from testlib.audio.audio_test_base import TelephonySIPTestBase
from testlib.audio.helper import get_hs_devnode
from testaid.headset import Headset
from testlib.audio.helper import dtmf_random_str
import time

class VOIP_WLAN_Test(TelephonySIPTestBase):
    VOICE_CALL = 'voice_call'
    RING = 'ring'
    ALARM = 'alarm'
    MUSIC = 'music'

    def setUp(self):
        super(VOIP_WLAN_Test, self).setUp()

    def tearDown(self):
        super(VOIP_WLAN_Test, self).tearDown()

    def testST_AUDIOCOMMS_FIT_WLVOIP_023(self):
        '''
        DTMF tong played to a local user during VOIP call using Phone app
        '''
        self.mo_call_sip(answer = True)
        self.check_dtmf_local()

    def testST_AUDIOCOMMS_VOIP_WLAN_113(self):
        '''
        Adjust media volume using hardware key during VOIP call
        '''
        #Make VOIP call by Phone APP
        self.mt_call_sip()
        #Adjust volume
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

    def testST_AUDIOCOMMS_VOIP_WLAN_114(self):
        '''
        Adjust ringtone volume hardware key during MT VOIP call alerting
        '''
        try:
            self.hs = None
            devnode = get_hs_devnode()
            self.hs = Headset(devnode, self.logger)
            self.mt_call_sip(answer = False)
            assert self.d.rpc.isStreamActive(self.RING, 0), "No phone call comes"
            self.d.uia_device.press.volume_up()
            self.d.uia_device.press.volume_up()
            time.sleep(1)
            assert self.d.telephony.is_audio_mute(), "Volume up to mute failed"
        finally:
            if self.hs:
                self.hs.reset()

    def testST_AUDIOCOMMS_VOIP_WLAN_115(self):
        '''
        Adjust media volume using settings menu during VOIP call
        '''
        #Make VOIP call by Phone APP
        self.mt_call_sip()
        #Adjust volume
        self.d.launch_sound_settings()
        self.d.uia_device(resourceId="android:id/seekbar").wait.exists()

        seek_bar = self.d.uia_device(text="Media volume").sibling(resourceId="android:id/seekbar").info["bounds"]
        y = (seek_bar["top"] + seek_bar["bottom"])/2
        start = seek_bar["left"] + 10
        end = seek_bar["right"] -10
        self.d.uia_device.click((end - start)/2, y)# volume middle
        time.sleep(1)
        start_vol = self.d.rpc.getStreamVolume(self.MUSIC)
        self.d.log_debug("Set vol middle, vol is %d"%start_vol)

        self.d.uia_device.click((end - start)/4, y)# Volume down
        time.sleep(1)
        current_vol = self.d.rpc.getStreamVolume(self.MUSIC)
        self.d.log_debug("Vol down, vol is %d"%current_vol)

        assert current_vol < start_vol,\
            "Volume down failed by settings. Before: %d, after: %d"%(current_vol, start_vol)

        self.d.uia_device.click((end - start)/2, y)#Volume middle
        time.sleep(1)
        last_vol = self.d.rpc.getStreamVolume(self.MUSIC)
        self.d.log_debug("Vol up, vol is %d"%last_vol)
        assert last_vol > current_vol,\
            "Volume up failed by settings. Before: %d, after: %d"%(current_vol, last_vol)

    def testST_AUDIOCOMMS_VOIP_WLAN_609(self):
        '''
        MT VOIP call on IHF, Volume change while screen is off
        '''
        try:
            self.mt_call_sip(answer = True)
            self.d.uia_device.sleep()
            time.sleep(.5)
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
        finally:
            self.d.unlock_screen()

    def testST_AUDIOCOMMS_VOIP_WLAN_301(self):
        '''
        VOIP call hold and unhold
        '''
        self.mo_call_sip(answer= True)
        for i in range(3):
            self.d.telephony.hold_call(True)
            time.sleep(1)
            assert not self.d.rpc.isStreamActive(self.VOICE_CALL, 0), 'Hold on VOIP call failed'
            self.d.telephony.hold_call(False)
            time.sleep(1)
            assert self.d.rpc.isStreamActive(self.VOICE_CALL, 0), 'Unhold VOIP call failed'

    #@use_hs
    def testST_AUDIOCOMMS_VOIP_WLAN_809(self):
        '''
        MO VOIP call on headphone -- Volume change interaction with mute func while screen is on
        #1.MO call
        #2.set volume to half and plya an 30's speech.
        #3.Mute on DUT
        #4.Increase volume by hw key until max
        #5.decrease volume by HW key until min
        #6.repeat 4~5 3 times
        #7.Unmute on DUT
        #8.repeat 4~5 3 times
        #9. repeat 3~8 3 times
        #10.hangup call
        '''
        LOOP = 3
        def volume_change_test(up = True):
            start = self.d.rpc.getStreamVolume(self.VOICE_CALL)
            if up:
                self.logger.debug("Check HS volume up")
                f = self.hs.volume_up
            else:
                self.logger.debug("Check HS volume down")
                f = self.hs.volume_down
            f()
            f()
            time.sleep(2)
            end = self.d.rpc.getStreamVolume(self.VOICE_CALL)
            assert (end > start) is up, "Check volume up/down failed."

        try:
            self.hs = None
            devnode = get_hs_devnode()
            self.hs = Headset(devnode, self.logger)
            self.d.deploy_play_music_content()
            self.mo_call_sip(answer = True)

            for i in range(LOOP):
                self.d.telephony.mute_call(True)
                for j in range(LOOP):
                    volume_change_test(up = True)
                    volume_change_test(up = False)
                self.d.telephony.mute_call(False)
                for k in range(LOOP):
                    volume_change_test(up = True)
                    volume_change_test(up = False)
        finally:
            if self.hs:
                self.hs.reset()

    def testST_AUDIOCOMMS_VOIP_WLAN_800(self):
        '''
        MO VoIP call over WLAN using Headphone
        '''
        self.mo_call_sip(answer=True)
        route = self.d.telephony.get_current_route()
        assert route == "EARPIECE", "audio not route to headphone"
        # check local voice active
        assert self.d.rpc.isStreamActive('voice', 0), 'local voice not active'

    def testST_AUDIOCOMMS_VOIP_WLAN_603(self):
        '''
        MO VOIP call over wlan Using Android phone app withing IHF
        '''
        #mo voip call
        self.mo_call_sip(answer = True)
        #activite louderspeaker
        self.d.telephony.speaker_on(True)
        time.sleep(1)
        info = self.d.get_stream_info('STREAM_VOICE_CALL')
        self.logger.debug(info)
        assert info.get('Devices') == 'speaker', 'Activate speaker failed'
        self.logger.debug(self.d.get_stream_info('STREAM_VOICE_CALL'))
        self.d.telephony.speaker_on(False)
        time.sleep(1)
        info = self.d.get_stream_info('STREAM_VOICE_CALL')
        self.logger.debug(info)
        assert info.get('Devices') == 'earpiece', 'Disactivate speaker failed.'

    def testST_AUDIOCOMMS_VOIP_WLAN_116(self):
        '''
        Adjust Alarm volume using settings menu during VoIP Call
        '''
        self.mo_call_sip(answer = True)
        self.d.launch_sound_settings()
        self.d.check_volume_in_settings(mode = 2)

    def testST_AUDIOCOMMS_VOIP_WLAN_101(self):
        '''
        MO VOIP call over WLAN using Android Phone App
        '''
        self.mo_call_sip(True)
        syms = dtmf_random_str(5)
        self.check_dtmf_local(syms)
        self.check_dtmf_remote(syms)

    def testST_AUDIOCOMMS_VOIP_WLAN_134(self):
        '''
        MO VOIP call over WLAN using Android Phone App
        '''
        self.mo_call_sip(answer = True)
        syms = dtmf_random_str(5)
        self.check_dtmf_remote(syms)

    def testST_AUDIOCOMMS_VOIP_WLAN_117(self):
        '''
        Adjust notification volume using settings menu during VOIP call
        '''
        self.mo_call_sip(answer = True)
        self.d.check_volume_in_settings(mode = 3)

    def testST_AUDIOCOMMS_FIT_WLVOIP_021(self):
        '''
        Press power key to lock screen with lock sound during VOIP call
        '''
        self.d.enable_screen_lock()
        self.mo_call_sip(answer = True)
        self.d.check_screen_lock_sound()

    def testST_AUDIOCOMMS_VOIP_WLAN_135(self):
        '''
        VOIP call established from call log
        '''
        self.mt_call_sip(answer = False)
        self.r_d.telephony.hangup_sip_call()
        time.sleep(1)
        self.d.telephony.launch_dialer()
        time.sleep(1)
        self.d.telephony.enter_call_log()
        self.d.telephony.make_call_from_log(
                self.r_d.telephony.get_default_account(),
                usenumber = self.d.telephony.get_default_account()
                )
        self.r_d.telephony.wait_sip_call()
        self.r_d.telephony.answer_sip_call(enter_phone = True)

    def testST_AUDIOCOMMS_VOIP_WLAN_102(self):
        '''
        MT VOIP Call over WLAN using Android Phone App
        '''
        self.mt_call_sip(answer = True)
        self.check_dtmf_local()