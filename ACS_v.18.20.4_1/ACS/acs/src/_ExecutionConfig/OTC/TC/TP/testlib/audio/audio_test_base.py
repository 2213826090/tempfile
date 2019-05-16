'''
Created on Dec 7, 2015

@author: bob
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.audio import resource
from testlib.audio.audio_log import AudioLogger
from testlib.audio.helper import dtmf_random_str
from testlib.audio.audio_hangouts import AudioHangouts
from testlib.audio.audio_device import AudioDevice
from testlib.audio.telephony import Telephony, TelephonySIP
# from testlib.system.system_impl import SystemImpl
# from testlib.systemui.systemui_impl import SystemUI
from testlib.util.remote import RemoteManager
import traceback

try:
    from testaid.sound import DTMF
    from testaid.sound import WaveHelper
except:
    pass#Pass for CTI exec

import os
import time
import shutil


class AudioTestBase(UIATestBase):
    '''
    Audoio uiauto test base
    '''
    @classmethod
    def setUpClass(cls):
        super(AudioTestBase, cls).setUpClass()

    @classmethod
    def tearDownClass(cls):
        super(AudioTestBase, cls).tearDownClass()

    def setUp(self):
        super(AudioTestBase, self).setUp()
        self.logger = AudioLogger.getLogger()
        self.logger.addFileHandler()

        self.__test_device = None
        #self.__reference_device = None

    def tearDown(self):
        self.logger.removeHanlders()
        super(AudioTestBase, self).tearDown()

    def get_test_device(self):
        """
        @summary: Get test device obj. Refer to testlib.util.device.TestDevice
        """
        if not self.__test_device:
            if hasattr(self, "contexts"):
                self.__test_device = AudioDevice(self.contexts.serial)
            else:
                self.__test_device = AudioDevice(g_common_obj.globalcontext.device_serial)
        return self.__test_device
    """
    def get_reference_device(self):
        '''
        @summary: Get reference device obj, Refer to testlib.util.device.TestDevice
        '''
        if not self.__reference_device:
            REFERENCE_PHONE = "reference_phone"
            ref_device = self.config.read(section = REFERENCE_PHONE)
            assert ref_device, "Get reference device serial failed. %s section not found in sys.conf"%REFERENCE_PHONE
            self.__reference_device = AudioDevice(ref_device.get("serial"))#Get serial from sys.conf
        return self.__reference_device
    """
    def get_log_dir(self):
        '''
        get log directory
        '''
        return g_common_obj.globalcontext.user_log_dir

    def download_install_app(self, device, apk_name):
        """
        Download apks and install
        """
        self.logger.debug("Download and install apk: %s"%apk_name)
        if not device:
            device = self.get_test_device()
        if "/" in apk_name:
            apk = resource.get_other_content(apk_name)
        else:
            apk = resource.get_app(apk_name)
        msg = device.adb_cmd_common("install -r %s"% apk)
        if "Success" in msg: return True
        else: return False

    def download_push_content(self, device, content, path):
        """
        Download and push file to DUT
        """
        self.logger.debug("Download and push content: %s"%content)
        if not device:
            device = self.get_test_device()
        if "/" in content:
            cnt = resource.get_other_content(content)
        else:
            cnt = resource.get_media_content(content)
        push_path = os.path.join(path, os.path.basename(cnt))
        if not device.push_file(cnt, push_path):
            self.logger.error("Push media content failed! %s"%content)
            return False
        return True

    def save_file(self, f):
        '''
        save file to log dir
        '''
        if os.path.exists(f):
            fname = os.path.basename(f)
            fpath = os.path.join(self.get_log_dir(), fname)
            self.logger.info("save to log dir: " + fpath)
            shutil.copy(f, fpath)

    def assert_dtmf_exists(self, wavfile, ref_sym, exists=True, similar=1.0):
        '''
        decode DTMF code from wavfile, and compare to reference symbol

        params:
            wavfile: .wav file to extract DTMF code
            ref_sym: a string of reference symbols
            exists: whether dmtf should exists
            similar: similarity, 1.0 means fully matched to ref symbols
        '''
        self.logger.debug("Check DTMF '%s' in wav file %s"%(ref_sym, wavfile))
        decoded = DTMF.decode(wavfile, gap=0.3)  # min gap 0.3s between two tone
        result = DTMF.similar(ref_sym, decoded)
        err_msg = '\n'.join(["check DTMF fails:",
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
            self.save_file(wavfile)
            raise

    def waitfor(self, timeout, expect, func, *args):
        """
        Wait for expected result(return by func), Return True once reached
        @return: Boolean
        """
        start = time.time()
        while time.time() - start < timeout:
            if func(*args) == expect:
                return True
            time.sleep(1)
        return False

class MultiDeviceTestBase(AudioTestBase):
    """
    Audio test base for multi device.
    """
    def setUp(self):
        super(MultiDeviceTestBase, self).setUp()
        self.rmt_mgr = RemoteManager()

        self.d = self.get_test_device()
        #self.r_d = self.x`()

        self.logger.debug("TestDevice: %s"%self.d)
        #self.logger.debug("ReferenceDevice: %s"%self.r_d)

        #self.init_audio_stub(self.d)
        #self.init_audio_stub(self.r_d)

    def tearDown(self):
        self.d.clean_up()
        self.r_d.clean_up()
        self.rmt_mgr.cleanup()
        super(MultiDeviceTestBase, self).tearDown()

    @property
    def failureException(self):
        self.logger._logobj.exception("------------------FAIL-----------------")
        try:
            self.d.root_on_device()
            self.r_d.root_on_device()
            test_dut_screenshot = "test_device_screenshot.png"
            reference_dut_screenshot = "reference_device_screenshot.png"
            self.d.screenshot(fname = test_dut_screenshot)
            self.r_d.screenshot(fname = reference_dut_screenshot)
        except Exception as e:
            #print e.message, e.args
            traceback.print_exc()

    def get_remote_device(self, service):
        '''
        Get remote device obj of specific service
        '''
        self.logger.debug('Get remote device of service %s'%service)
        s = self.rmt_mgr.get_service(service)
        d = AudioDevice(serial = s.test_device.serial,
                        adb_server_host = s.test_device.adb_server_host,
                        adb_server_port = s.test_device.adb_server_port)
        return s.data, d

class AudioHangoutsTestBase(MultiDeviceTestBase):
    """
    Audio Hangtouts Test Base
    """
    def setUp(self):
        super(AudioHangoutsTestBase, self).setUp()
        test_google_account = self.config.read(section = "google_account").get("user_name")
        test_google_psw = self.config.read(section = "google_account").get("user_name")
        assert all((test_google_account, test_google_psw)), "Can't get Google account info."
        #ref_google_account = self.config.read(section = "reference_google_account").get("user_name")
        #ref_google_psw = self.config.read(section = "reference_google_account").get("password")

        self.d.unlock_screen()
        self.init_hangouts(test_google_account, test_google_psw, self.d)

        data , self.r_d = self.get_remote_device('Hangouts')
        ref_google_account = data.get('gaccount')
        ref_google_psw = data.get('gpassword')
        assert all((ref_google_account, ref_google_psw)), "Can't get Google account on remote device"
        self.r_d.unlock_screen()
        self.init_hangouts(ref_google_account, ref_google_psw, self.r_d)

        self.logger.debug("TestDevice: %s, GoogleAccount: %s"%(self.d.serial, self.d.hangouts.get_account()[0]))
        self.logger.debug("ReferenceDevice: %s, GoogleAccount: %s"%(self.r_d.serial, self.r_d.hangouts.get_account()[0]))

    def tearDown(self):
        self.d.hangouts.stop_by_am()
        self.r_d.hangouts.stop_by_am()
        super(AudioHangoutsTestBase, self).tearDown()
        self.d.back_home()
        self.r_d.back_home()

    def init_hangouts(self, account, password, device = None):
        """
        Set up hangouts on device
        """
        if not device:
            device = self.d
        if not hasattr(device, "hangouts"):
            setattr(device, "hangouts", AudioHangouts(account, password, device = device))
        device.hangouts.clean_up_data()
        device.hangouts.grant_permissions()
        device.hangouts.launch_by_am()
        device.back_home()

    def make_dtmf_wave(self, symbol, rate = 44100, duration=500, interval=500):
        """
        encode a string of symbol to DTMF Tone, and saved as .wav file
        @param symbol: String, substring of "123456789*#"
        """
        import tempfile
        wav = tempfile.mktemp(suffix=".wav")
        DTMF.encode(symbol, wav, rate = rate, duration = duration, interval = interval)
        return wav

    def mo_call(self, Type = "voice"):
        """Make a MO call and accept"""
        self.logger.info("MO call: From %s to %s"%(self.d, self.r_d))
        self.d.hangouts.launch_by_am()
        self.d.hangouts.search_contact(self.r_d.hangouts.get_account()[0])
        if Type =="voice":
            self.d.hangouts.make_voice_call()
        elif Type == "video":
            self.d.hangouts.make_video_call()
        else: raise Exception("Unknown type of hangouts call: %s"%Type)
        self.r_d.hangouts.waitfor_call()
        self.r_d.hangouts.accept_call()

    def mt_call(self, Type = "voice"):
        """Recieve a MT call and accept"""
        self.logger.info("MT call: From %s to %s"%(self.r_d, self.d))
        self.r_d.hangouts.launch_by_am()
        self.r_d.hangouts.search_contact(self.d.hangouts.get_account()[0])
        if Type =="voice":
            self.r_d.hangouts.make_voice_call()
        elif Type == "video":
            self.r_d.hangouts.make_video_call()
        else: raise Exception("Unknown type of hangouts call: %s"%Type)
        self.d.hangouts.waitfor_call()
        self.d.hangouts.accept_call()


class TelephonyTestBase(MultiDeviceTestBase):
    def setUp(self):
        super(TelephonyTestBase, self).setUp()
        tele_config = self.config.read(section = 'telephony')
        number = tele_config.get('sim_card')
        data, self.r_d = self.get_remote_device('Telephony')
        self.init_device(self.d, number)
        self.init_device(self.r_d, data.get('number'))

    def tearDown(self):
        # force hangup any ongoing call
        hangup_cmd = "input keyevent 6"  # KEYCODE_ENDCALL = 6
        self.d.adb_cmd(hangup_cmd)
        self.r_d.adb_cmd(hangup_cmd)
        super(TelephonyTestBase, self).tearDown()

    def init_device(self, device,number = None):
        device.unlock_screen()
        tele = Telephony(device, number = number)
        device.telephony = tele
        device.log_debug("Phone number: " + tele.get_number())

    def start_mo_call(self):
        self.logger.info("start MO call")
        remote = self.r_d.telephony.get_number()
        self.d.telephony.make_call_am(remote)
        self.r_d.telephony.wait_call()
        time.sleep(3)

    def start_mt_call(self, answer=True):
        self.logger.info("start MT call")
        local = self.d.telephony.get_number()
        self.r_d.telephony.make_call_am(local)
        self.d.telephony.wait_call(answer=answer)
        time.sleep(3)

    def local_press_dtmf(self, syms, long_click = False):
        '''
        press dtmf in local, and record in remote
        return the recorded file
        '''
        self.logger.info("local press DTMF, syms: %s" % syms)
        self.d.uia_device.wakeup()
        self.r_d.record_call()
        for s in syms:
            self.d.telephony.dtmf(s, long_click = long_click)
            time.sleep(0.5)
        time.sleep(3)#Sleep 3 secends for phone call delay
        record = self.r_d.stop_record_call()
        return record

    def remote_press_dtmf(self, syms, long_click=False):
        '''
        press dtmf in remote, and record in local
        return the recorded file
        '''
        self.logger.info("remote press DTMF, syms: %s" % syms)
        self.r_d.uia_device.wakeup()
        self.d.record_call()
        for s in syms:
            self.r_d.telephony.dtmf(s, long_click = long_click)
            time.sleep(0.5)
        record = self.d.stop_record_call()
        return record

    def assert_audio_active_both_side(self):
        self.logger.info("check if audio active in both side")
        syms = dtmf_random_str(8)
        record = self.local_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar=0.6)
        record = self.remote_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar=0.6)

    def recv_sms(self):
        local = self.d.telephony.get_number()
        sms_text = str(int(time.time()))
        self.r_d.telephony.send_sms_to(local, sms_text)
        self.d.telephony.wait_sms(sms_text)

    def gapp_play_music(self):
        '''
        play music using Google App (Music Player)
        '''
        self.logger.info("play music in Google Music Player")
        song = self.d.DEFAULT_CONTENTS[0]
        self.d.deploy_play_music_content()
        self.d.launch_play_music_n_play(song)

    def assert_voice_mute(self):
        '''
        check if voice is muted
        '''
        assert self.d.telephony.is_audio_mute(), 'Voice is not muted'
        # the remote peer should not heard sound
        self.d.record_call()
        time.sleep(10)
        record = self.d.stop_record_call()
        pos = WaveHelper.find_sound(record)
        assert pos < 0, 'remote can still heard sound after mute'
        # remote can still be heard in local
        syms = dtmf_random_str(5)
        record = self.remote_press_dtmf(syms)
        self.assert_dtmf_exists(record, syms, similar= 0.7)

    def assert_volume_change_by_HWKeypress(self, key_event, sound_type='voice_call', sleep=2):
        '''
        check if volume can be changed by HW keypress vol +/-
        '''
        vol_before = self.d.rpc.getStreamVolume(sound_type)
        self.d.uia_device.press(key_event)
        time.sleep(sleep)
        vol_after = self.d.rpc.getStreamVolume(sound_type)
        self.logger.info("Press %s; Current volume is: %d" % (key_event, vol_after))
        assert vol_before != vol_after, "Volume not changed."


class TelephonySIPTestBase(MultiDeviceTestBase):
    def setUp(self):
        super(TelephonySIPTestBase, self).setUp()
        sip_config = self.config.read(section = 'sip_account')
        tele_config = self.config.read(section = 'telephony')
        number = tele_config.get('sim_card')
        #name, password, server = sip_config.get('user1').split('@')
        name = sip_config.get('account')
        pswd = sip_config.get('password')
        server = sip_config.get('server')
        assert all((name, pswd, server)), "Can't get SIP account"
        self.init_device(self.d, name, pswd, server, number = number)

        #name, password, server = sip_config.get('user2').split('@')
        data, self.r_d = self.get_remote_device('SIP')
        assert all(data.values()), "Can't get SIP info on remote device"
        self.init_device(self.r_d, data.get('account'), data.get('password'), data.get('server'))

    def tearDown(self):
        # force hangup any ongoing call
        hangup_cmd = "input keyevent 6"  # KEYCODE_ENDCALL = 6
        self.d.adb_cmd(hangup_cmd)
        self.r_d.adb_cmd(hangup_cmd)
        super(TelephonySIPTestBase, self).tearDown()

    def init_device(self, device, name, password, server, number = None):
        device.unlock_screen()
        device.telephony = TelephonySIP(device, number = number)
        device.telephony.clean_up()
        device.telephony.setup_default_account(name, password, server)

    def get_remote_telephony(self):
        '''
        Get remote device that with telephony service
        '''
        data, device = self.get_remote_device('Telephony')
        device.unlock_screen()
        device.telephony = TelephonySIP(device, data.get('number'))
        return device

    def mo_call_sip(self, answer = True):
        self.logger.debug("Make MO SIP call. Answer: %s"%answer)
        test_acc_name = self.d.telephony.get_default_account()
        ref_acc_name = self.r_d.telephony.get_default_account()
        self.d.telephony.call_with_sip(ref_acc_name, test_acc_name)
        self.r_d.telephony.wait_sip_call()
        if not answer:
            return
        self.r_d.telephony.answer_sip_call(enter_phone = True)
        time.sleep(1)

    def mt_call_sip(self, answer = True):
        self.logger.debug("Make MT SIP call. Answer: %s"%answer)
        test_acc_name = self.d.telephony.get_default_account()
        ref_acc_name = self.r_d.telephony.get_default_account()
        self.r_d.telephony.call_with_sip(test_acc_name, ref_acc_name)
        self.d.telephony.wait_sip_call()
        if not answer:
            return
        self.d.telephony.answer_sip_call(enter_phone = True)
        time.sleep(1)

    def press_dtmf(self, device, syms):
        self.logger.debug('Press DTMF on device %s'%device)
        device.uia_device.wakeup()
        for s in syms:
            self.d.telephony.dtmf(s)
            time.sleep(0.5)

    def check_dtmf_remote(self, syms = '0123456789*#'):
        self.logger.debug('Check DTMF tone on remote')
        self.r_d.telephony.enter_dial_pad()
        self.d.record_call()
        self.press_dtmf(self.r_d, syms)
        record = self.d.stop_record_call()
        self.assert_dtmf_exists(record, syms, similar=0.6)

    def check_dtmf_local(self, syms = '0123456789*#'):
        self.logger.debug('Check DTMF tone on local')
        self.d.telephony.enter_dial_pad()
        self.r_d.record_call()
        self.press_dtmf(self.d, syms)
        record = self.r_d.stop_record_call()
        self.assert_dtmf_exists(record, syms, similar=0.6)

class AudioStubTestBase(AudioTestBase):
    media_path = "/sdcard/otctest/"
    media_files = ["mp3_sample1.mp3", "mp3_sample2.mp3", "mp3_sample3.mp3"]

    def setUp(self):
        super(AudioStubTestBase, self).setUp()
        self.test_device = self.get_test_device()
        self.rpc = self.test_device.rpc
        # self.system = SystemImpl()
        # self.systemui = SystemUI()
        self.adb = self.test_device
        self.d = self.test_device.get_device()
        self.audio = self.test_device.gmusic

        # make sure screen unlocked, and in Home screen
        self.test_device.unlock_screen()
        self.d.press.home()
        time.sleep(1)

    def tearDown(self):
        self.test_device.clean_up()
        #self.logger.removeHanlders()
        super(AudioStubTestBase, self).tearDown()

    @property
    def failureException(self):
        self.logger._logobj.exception("------------------FAIL-----------------")
        try:
            self.test_device.screenshot(fname = "screenshot.png")
        except Exception as e:
            print e.message, e.args
            traceback.print_exc()

    def _refresh_storage(self):
        self.test_device._refresh_storage()

    def launch_play_music_n_play(self, *args, **kw):
        self.test_device.launch_play_music_n_play(*args, **kw)

    def deploy_play_music_content(self, *args):
        self.test_device.deploy_play_music_content(*args)

    def clean_play_music_content(self):
        self.test_device.clean_play_music_content()

    def get_media_file(self):
        fpath = resource.get_media_content("large_audio_file.mp3")
        if not os.path.isfile(fpath):
            raise Exception("media file not found")
        return fpath

    def launch_sound_settings(self):
        return self.test_device.launch_sound_settings()

    def enable_screen_lock(self):
        return self.test_device.enable_screen_lock()

    def check_screen_lock_sound(self):
        return self.test_device.check_screen_lock_sound()

    def check_volume_in_settings(self):
        return self.test_device.check_volume_in_settings()

    def set_option(self, *args, **kw):
        return self.test_device.set_option(*args, **kw)

    def scroll_n_click(self, *args, **kw):
        return self.test_device.scroll_n_click(*args, **kw)

    def get_android_version(self):
        return self.test_device.get_android_version()

    def screenshot(self):
        return self.test_device.screenshot()
