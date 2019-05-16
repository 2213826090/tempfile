from testlib.audio.audio_test_base import TelephonySIPTestBase
from testlib.audio.helper import dtmf_random_str
from testaid.sound import WaveHelper
import time


class Call_Swap_Test(TelephonySIPTestBase):
    VOICE_CALL = 'voice_call'
    RING = 'ring'
    ALARM = 'alarm'
    MUSIC = 'music'

    def setUp(self):
        super(Call_Swap_Test,self).setUp()
        self.tele_dev = None
        self.tele_dev = self.get_remote_telephony()

    def tearDown(self):
        hangup_cmd = "input keyevent 6"
        if self.tele_dev:
            self.tele_dev.adb_cmd(hangup_cmd)
            self.tele_dev.clean_up()
        self.d.adb_cmd(hangup_cmd)
        super(Call_Swap_Test,self).tearDown()

    def mt_scnd_csv_call(self, answer = True):
        self.logger.debug("MT secondary CSV call")
        self.tele_dev.telephony.make_call_am(self.d.telephony.get_number())
        self.d.telephony.wait_call(answer = False)
        if answer:
            self.d.telephony.answer_call_ui()
            crt_call = self.d.uia_device(resourceId = 'com.android.dialer:id/name')
            assert crt_call.exists and \
                crt_call.text.replace(" ","") in self.tele_dev.telephony.get_number(),\
                "MT scnd csv call failed"

    def mt_scnd_voip_call(self, answer = True):
        self.logger.debug("MT secondary VOIP call")
        number = self.d.telephony.get_default_account()
        self.r_d.telephony.call_with_sip(number, self.r_d.telephony.get_default_account())
        self.d.telephony.wait_call(answer = False)
        if answer:
            self.d.telephony.answer_call_ui()
            crt_call = self.d.uia_device(resourceId = 'com.android.dialer:id/name')
            assert crt_call.exists and \
                crt_call.text.replace(" ","") in self.r_d.telephony.get_default_account(),\
                "MT scnd VOIP call failed"
            scnd_call = self.d.uia_device(resourceId = "com.android.dialer:id/secondaryCallName")
            assert scnd_call.wait.exists(), "Can't find secondary call"

    def check_multi_call_status(self, ongoing, onhold):
        '''
        Check the status of ongoing call and secondary call
        @param ongoing: AudioDevice object with telephony propt
        @param holdon: AudioDevice object with telephony propt
        '''
        self.logger.debug('Check DTMF tone on ongoing call and onhold call')
        assert ongoing.telephony.is_in_call()
        assert onhold.telephony.is_in_call()
        self.d.telephony.enter_dial_pad()
        ongoing.record_call()
        onhold.record_call()
        syms = dtmf_random_str(5)
        self.press_dtmf(self.d, syms)
        ongoing_record = ongoing.stop_record_call()
        holdon_record = onhold.stop_record_call()
        self.assert_dtmf_exists(ongoing_record, syms, similar=0.6)
        self.assert_dtmf_exists(holdon_record, syms, exists = False, similar=0.3)

    def switch_call(self):
        '''
        Switch to call.
        '''
        scnd_call = self.d.uia_device(resourceId = "com.android.dialer:id/secondaryCallName")
        assert scnd_call.wait.exists(), "Can't find secondary call"
        scnd_call.click.wait()
        time.sleep(3)

    def testST_AUDIOCOMMS_VOIP_WLAN_118(self):
        '''
        MT 3G voice call during VOIP call is ongoing with Phone App
        '''
        self.mo_call_sip(answer = True)
        self.tele_dev.telephony.make_call_am(self.d.telephony.get_number())
        self.d.telephony.wait_call(answer = True)
        self.d.telephony.hangup_call()
        time.sleep(1)
        self.d.telephony.hangup_call()

    def testST_AUDIOCOMMS_VOIP_WLAN_119(self):
        '''
        MT VoIP call with Phone app during 3G voice call is ongoing
        '''
        self.d.telephony.make_call_am(self.tele_dev.telephony.get_number())
        self.tele_dev.telephony.wait_call()
        time.sleep(3)
        assert self.d.telephony.is_in_call(), "MT 3G call failed"
        self.mt_scnd_voip_call(answer = True)
        self.d.uia_device(resourceId = "com.android.dialer:id/secondaryCallName").wait.exists()
        assert self.d.uia_device(resourceId = "com.android.dialer:id/name").text\
            in self.r_d.telephony.get_default_account(),\
            "Answer SIP call failed"
        self.d.telephony.hangup_call()

    def testST_AUDIOCOMMS_VOIP_WLAN_138(self):
        '''
        Switch between VOIP and 2G CSV Call
        '''
        csv_call = self.tele_dev.telephony.get_number()
        self.d.telephony.make_call_am(csv_call)
        self.tele_dev.telephony.wait_call()
        time.sleep(3)
        assert self.d.telephony.is_in_call(), "MT 3G call failed"
        self.mt_scnd_voip_call(answer = True)
        scnd_call = self.d.uia_device(resourceId = "com.android.dialer:id/secondaryCallName")
        crt_call = self.d.uia_device(resourceId = "com.android.dialer:id/name")
        scnd_call.wait.exists()

        self.logger.debug("Switch between VOIP call and CSV call")
        for i in range(20):
            ongoing = crt_call.text.replace(" ","")
            self.logger.debug("Loop %d, Current Call: %s"%(i, ongoing))
            self.switch_call()
            assert scnd_call.exists and\
                ongoing not in crt_call.text.replace(" ",""), "Switch call failed!"

    def testST_AUDIOCOMMS_FIT_WLVOIP_025(self):
        '''
        Play Ringtone for MT CSV Call on IHF only during VOIP call
        '''
        self.mo_call_sip(answer = True)
        self.d.telephony.speaker_on(True)
        self.r_d.telephony.mute_call(True)
        self.mt_scnd_csv_call(answer = False)
        record = self.d.record_call(duration = 5)
        self.logger.debug("Record wav: %s"%record)
        assert WaveHelper.find_sound(record) != -1, "Can't find sound while secondary call comes"

    def testST_AUDIOCOMMS_VOIP_WLAN_137(self):
        '''
        Switch between VOIP call and 3G CSV call  -- interactive
        '''
        self.d.telephony.make_call_am(self.tele_dev.telephony.get_number())
        self.tele_dev.telephony.wait_call(answer = True)
        self.mt_scnd_voip_call(answer = True)
        self.check_multi_call_status(ongoing = self.r_d, onhold = self.tele_dev)
        self.switch_call()
        self.check_multi_call_status(ongoing = self.tele_dev, onhold = self.r_d)