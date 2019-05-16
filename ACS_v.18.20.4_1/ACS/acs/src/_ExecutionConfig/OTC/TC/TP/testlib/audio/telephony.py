import re
import time
from testlib.util.common import g_common_obj
from testlib.audio.audio_log import AudioLogger


class Telephony(object):
    pkgname = 'com.android.dialer'
    phonename = 'com.android.phone'
    msgpkg = 'com.google.android.apps.messaging'
    prop = 'test.otc.telephony.number'

    def __init__(self, device=None, number=None):
        '''
        d is the uiautomator instance
        '''
        self.adb = device if device else g_common_obj.get_test_device()
        self.device = self.adb
        self.d = self.adb.get_device()
        if not number:
            number = self._get_number()
        self.number = number
        self.logger = AudioLogger.getLogger()

    def log_debug(self, msg):
        self.logger.debug("Telephony[%s]: %s" % (self.number, msg))

    def _call_with(self, acc, timeout = 10):
        '''
        Choose which account used to make call. SIM card or SIP account
        '''
        #if acc in self.number:
        #    acc_wgt = self.d(resourceId = 'com.android.dialer:id/number', textContains= self.number)
        #else:
        acc_wgt = self.d(resourceId = 'com.android.dialer:id/number', textContains = acc)
        opt_wgt = self.d(text = 'Call with', resourceId = 'android:id/alertTitle')
        start = time.time()
        while 1:
            time.sleep(.5)
            if self.is_in_call():
                break
            if time.time() - start > timeout:
                break
            if opt_wgt.exists:
                self.log_debug('Call with acc %s'%acc)
                acc_wgt.click.wait()
                break
        time.sleep(1)

    def _back_to_call(self):
        """Back to inprogess call"""
        self.launch_dialer()
        time.sleep(1)
        self.d(text = 'Return to call in progress').click.wait()
        time.sleep(.5)

    def _get_number(self):
        # get prop device prop first
        number = self.adb.adb_cmd_capture_msg('getprop ' + self.prop).strip()
        if re.match('[+0-9]+', number):
            return number
        # else, get from UI
        self.d.wakeup()
        self.adb.adb_cmd('input keyevent 82')  # unlock screen
        time.sleep(1)
        self.d.press.home()
        self.adb.adb_cmd("am start -W com.android.settings")
        time.sleep(1)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="SIM cards")
        self.d(text="SIM cards").click.wait()
        time.sleep(1)
        ui = self.d(textContains="SIM slot",
                    className="android.widget.TextView",
                    enabled=True)
        if ui.exists:
            text = ui.down(className="android.widget.TextView").text
            number = text.split('-')[-1].strip()
            self.d.press.back()
            self.d.press.back()
            # save to prop
            self.adb.root_on_device()  # must root first
            time.sleep(4)
            self.adb.adb_cmd('setprop %s %s' % (self.prop, number))
            return number
        else:
            raise Exception("No SIM card found")

    def get_number(self):
        '''
        get phone number of this DUT
        '''
        return self.number

    def launch_dialer(self):
        '''
        launch the dialer
        '''
        self.log_debug('launch dialer')
        self.adb.adb_cmd("am start -W " + self.pkgname)
        time.sleep(2)

    def enter_dial_pad(self):
        '''
        enter dial pad from dialer app
        '''
        self.log_debug('enter dial pad')
        try:
            self.d(description="dial pad").click.wait()
        except:
            self.d(resourceId='com.android.dialer:id/dialpadButton').click.wait()

    def enter_ongoing_call(self):
        '''
        '''
        self.log_debug("enter ongoing call page")
        # first check if in call-info page
        if not self.d(resourceId=self.pkgname+":id/callCardFragment").exists:
            self.d.press.home()
            self.d.open.notification()
            time.sleep(2)
            keyword = "Ongoing call"
            ui = self.d(text=keyword)
            if ui.exists:
                ui.click.wait()
            else:
                raise Exception("No ongoing call")

    def enter_call_log(self):
        '''Enter call log from main interface'''
        wgt = self.d(description = 'Recents')
        wgt.click.wait()
        assert wgt.selected, "Enter call log failed"

    def press(self, sym, long_click=False):
        '''
        press the the dial pad number
        '''
        self.log_debug("press button: %s" % sym)
        ui = self.d(descriptionContains="%s" % sym)
        if long_click:
            ui.long_click()
            time.sleep(1)
        else:
            ui.click.wait()

    def make_call_dialpad(self, numbers):
        '''
        make call in dial pad
        '''
        self.log_debug("make call from dial pad, number: %s" % numbers)
        for c in numbers:
            if c == '+':
                self.press(0, long_click=True)
            else:
                self.press(c)
        self.d(description='dial').click.wait()
        #time.sleep(1)
        self._call_with(self.number)

    def make_call_am(self, numbers):
        '''
        make call through 'am start' request
        '''
        self.log_debug("make call through am, number: %s" % numbers)
        cmd = "am start -a android.intent.action.CALL -d tel:" + numbers
        self.adb.adb_cmd(cmd)
        #time.sleep(1)
        self._call_with(self.number)

    def make_call_ui(self, numbers):
        '''
        make call from UI
        '''
        self.d.press.back()
        self.d.press.home()
        self.launch_dialer()
        self.enter_dial_pad()
        self.make_call_dialpad(numbers)
        #self._call_with(self.number)

    def make_call_from_log(self, numbers, usenumber = None):
        '''
        make call from call log items
        '''
        if not usenumber:
            usenumber = self.number
        wgt = self.d(text = numbers)
        assert wgt.wait.exists(), "Can't find call in call log: %s"%numbers
        wgt.right(resourceId = 'com.android.dialer:id/primary_action_button').click.wait()
        self._call_with(usenumber)

    def _select_btn(self, desc, enable):
        ui = self.d(descriptionContains=desc, selected=False if enable else True)
        if ui.exists:
            ui.click.wait()

    # these methods only applied to Ongoing call page
    def mute_call(self, enable):
        '''
        mute/unmute current call
        '''
        self.log_debug("mute call: %s" % enable)
        self._select_btn("Mute", enable)

    def hold_call(self, enable=True):
        '''
        hold/unhold current call
        '''
        self.log_debug("hold call: %s" % enable)
        hold_btn = self.d(resourceId='com.android.dialer:id/holdButton')
        if enable:
            if not self.is_onhold():
                hold_btn.click.wait()
                time.sleep(.5)
        else:
            if self.is_onhold():
                hold_btn.click.wait()
                time.sleep(.5)

    def speaker_on(self, enable):
        '''
        enable/disable speaker
        '''
        self.log_debug("set speaker on: %s" % enable)
        wgt = self.d(resourceId="com.android.dialer:id/audioButton")
        if (enable and not wgt.checked) or (not enable and wgt.checked):
            wgt.click.wait()
            time.sleep(.5)
        #self._select_btn("Speaker", enable)

    def dtmf(self, sym, long_click=False):
        '''
        enter [0-9*#]
        '''
        self._select_btn("Dialpad", True)
        self.press(sym, long_click = long_click)

    def hangup_call(self):
        '''
        hangup current call
        '''
        self.log_debug("hangup_call")
        self.d(description="End Call").click.wait()

    def answer_call(self, enter_phone = True):
        '''
        answer incoming call by keyevent
        '''
        self.log_debug("answer the call with am")
        # KEYCODE_CALL = 5
        self.adb.adb_cmd("input keyevent 5")
        time.sleep(1)
        if enter_phone:
            self._back_to_call()

    def answer_call_ui(self):
        '''
        answer incoming call by UI
        '''
        call_wgt = self.d(resourceId = "com.android.dialer:id/glow_pad_view")
        bounds = call_wgt.bounds
        x_middle = int(bounds['left']+ bounds['right']/2)
        x_right = int(bounds['right'])
        y_middle = int((bounds['top']+bounds['bottom'])/2)

        self.d.drag(x_middle,y_middle, x_right, y_middle, steps = 3)
        time.sleep(1)

    # helper function
    def wait_call(self, timeout=60, answer=True):
        '''
        wait for incoming call

        timeout: max wait timeout
        answer: if True, answer call if call incoming
        '''
        self.log_debug("wait call")
        start = time.time()
        while time.time() - start < timeout:
            if self.isRinging():
                if answer:
                    time.sleep(1)
                    self.answer_call()
                break
            time.sleep(1)
        else:
            raise Exception("wait for call timeout %ss" % timeout)

    def isRinging(self):
        """Check if ringing"""
        if self.d(resourceId = "com.android.dialer:id/glow_pad_view").exists:
            return True
        cmd = "dumpsys telecom | grep mIsRinging"
        msg = self.adb.adb_cmd_capture_msg(cmd)
        if not msg.strip():
            cmd = "dumpsys telecom | grep -A1 'Ringing calls' | tail -1"
            msgs = self.adb.adb_cmd_capture_msg(cmd)
            msg = 'Ringing calls: ' + msgs
            if not msg.strip():
                raise "Ring call not found."
        val = msg.split(":")[-1].strip()
        return True if (val != str(False).lower()) and (val != '') else False

    def send_sms_to(self, who, what):
        '''
        send sms to 'who' with the content 'what'
        '''
        self.log_debug("send sms to %s, content: %s" % (who, what))
        cmd = ' '.join(['am start -W -a android.intent.action.SENDTO',
                        '-d sms:%s' % who,
                        '--es sms_body "%s"' % what,
                        '--ez exit_on_sent true'])
        self.adb.adb_cmd(cmd)
        time.sleep(2)
        send_icon = "%s:id/self_send_icon" % self.msgpkg
        self.d(resourceId=send_icon).click.wait()
        self.d.press.back()
        self.d.press.back()

    def send_audio_mms_to(self, who):
        '''
        Send audio mms to remote
        '''
        self.log_debug("send audio mms to %s" % who)
        #Enable audio permission
        PRMSSN_AUDIO = "android.permission.RECORD_AUDIO"
        PRMSSN_CAMERA = "android.permission.CAMERA"
        cmd = "pm grant %s"%self.msgpkg
        self.adb.adb_cmd("%s %s"%(cmd, PRMSSN_AUDIO))
        self.adb.adb_cmd("%s %s"%(cmd, PRMSSN_CAMERA))

        cmd = ' '.join(['am start -W -a android.intent.action.SENDTO',
                        '-d sms:%s' % who])
        self.adb.adb_cmd(cmd)
        time.sleep(2)
        att_icon = self.d(resourceId = "com.google.android.apps.messaging:id/attach_media_button")
        att_icon.wait.exists()
        att_icon.click.wait()
        time.sleep(1)
        self.d(description = 'Record audio').click.wait()
        time.sleep(1)
        record_icon = self.d(resourceId = 'com.google.android.apps.messaging:id/record_button_visual')
        record_icon.long_click()
        send_btn = self.d(resourceId = 'com.google.android.apps.messaging:id/send_message_button')
        send_btn.click.wait()
        time.sleep(1)

    def wait_sms(self, content, timeout=60):
        '''
        wait for sms with content
        '''
        self.log_debug("wait sms with content: %s" % content)
        # start message app first
        self.adb.adb_cmd('am start -W ' + self.msgpkg)
        start = time.time()
        while time.time() - start < timeout:
            if self.d(text=content).exists:
                self.d.press.back()
                return True
            time.sleep(0.5)
        else:
            raise Exception("wait for sms timeout %ss" % timeout)

    def wait_audio_mms(self, timeout = 120):
        '''
        wait for audio mms and play
        '''
        self.log_debug("Wait audio mms")
        # start message app first
        self.adb.adb_cmd('am start -W ' + self.msgpkg)
        play_btn = self.d(resourceId = 'com.google.android.apps.messaging:id/play_button')
        start = time.time()
        while time.time() - start < timeout:
            if play_btn.exists:
                play_btn.click.wait()
                assert self.device.rpc.isStreamActive('music', 5000), 'Play recieved mms failed'
                self.d.press.back()
                return True
            time.sleep(0.5)
        raise Exception("wait for sms timeout %ss" % timeout)

    def dump_audio_state(self):
        # telephony audio state can get by 'adb shell dumpsys telecom'
        buf = self._dump_telecom()
        info = {}
        if 'mAudioState' in buf:
            judge = 'M'
            for l in buf.split('\n'):
                if 'mAudioState' in l:
                    line = l
                    self.log_debug(line)
                    break
        else:
            try:
                line = self.adb.adb_cmd_capture_msg('dumpsys telecom | grep AUDIO_ROUTE | tail -1')
                judge = "N"
            except:
                self.log_debug("Fail to detect AudioStatus")
                return {}
        if 'isMuted: true' in line:
            info['isMuted'] = True
        elif 'MUTE_ON' in line:
            info['isMuted'] = True
        else:
            info['isMuted'] = False
        # find the route type
        if judge == 'M':
            substr = filter(lambda s: s.startswith('route:'),
                            map(str.strip, line.split(',')))[0]
            info['route'] = substr.split()[-1]
        else:
            patt = re.compile(r"\((.*?)\)", re.I | re.X)
            substr = patt.findall(line)[0]
            info['route'] = substr
        return info

    def _dump_telecom(self):
        '''
        Dump telephony status by adb shell dumpsys telecom
        '''
        buf = self.adb.adb_cmd_capture_msg('dumpsys telecom')
        return buf

    def is_audio_mute(self):
        return self.dump_audio_state()['isMuted']

    def get_current_route(self):
        return self.dump_audio_state()['route']

    def is_in_call(self):
        '''
        Check in call or not by dump telephony status
        '''
        key, value = 'mAudioFocusStreamType',"STREAM_VOICE_CALL"
        buf = self._dump_telecom()
        for line in buf.split('\n'):
            if key in line:
                if value in line:
                    return True
                else:
                    return False

    def is_onhold(self):
        hold_btn = self.d(resourceId='com.android.dialer:id/holdButton')
        return hold_btn.selected

class TelephonySIP(Telephony):
    sip_prop = "test.otc.telephony.sip_account"

    class Account(dict):
        '''
        Account Instance
        a = Account(name, password, server)
        a.name
        a.password
        a.server
        a.enabled
        '''
        def __init__(self, name, psw, server):
            super(TelephonySIP.Account, self).__setitem__('name', name)
            super(TelephonySIP.Account, self).__setitem__('password', psw)
            super(TelephonySIP.Account, self).__setitem__('server', server)
            super(TelephonySIP.Account, self).__setitem__('enabled', True)
        def __str__(self):
            return self['name'] + '@' + self['server']
        def __getattr__(self, attr):
            if attr in self.keys():
                return self.get(attr)
        def __setattr__(self, attr, val):
            self[attr] = val

    def __init__(self, device, number = None):
        self.logger = AudioLogger.getLogger()
        self.device = device
        self.adb = self.device
        self.d = self.device.get_device()
        self.number = number

        self.__default_account = None
        self.__accounts = []

    def calling_accounts_settings(self):
        '''
        Launch calling accounts settings activity
        '''
        self.device.adb_cmd('am start -S -W \
            com.android.phone/com.android.phone.settings.PhoneAccountSettingsActivity')
        assert self.d(text = 'SIP settings').wait.exists(),\
            'Launch sip account settings failed'

    def get_accounts(self, enabled = True):
        '''
        Get account names that added.
        If enabled, return all account which enabled(a.enabled), else return all
        '''
        if enabled:
            accs = filter(lambda x: x.enabled == True, self.__accounts)
        else:
            accs = self.__accounts
        return [_.name for _ in accs]

    def get_default_account(self):
        '''
        Get default account
        '''
        return self.__default_account

    def clean_up(self):
        '''
        Clean up all accounts settings
        '''
        self.adb.adb_cmd("pm clear %s"%self.phonename)
        self.adb.adb_cmd("pm clear %s"%self.pkgname)
        self.__accounts = []
        self.device.back_home()

    def setup_default_account(self, name, password, server):
        '''
        Setup default account
        '''
        self.log_debug("Setup default account: %s"%name)
        self.calling_accounts_settings()
        self.add_account(name, password, server)
        self.__default_account = name
        time.sleep(1)
        self.d.press.back()
        time.sleep(.5)
        self.enable_make_calls_option()
        self.sip_call_only(enable = False)
        self.receive_incoming_sip_call(enable= True)
        self.device.back_home()

    def add_account(self, name, password, server):
        '''
        Add SIP account. name/password/server are required
        '''
        def set_text(wgt_txt, txt):
            self.d(text = wgt_txt).click.wait()
            time.sleep(1)
            self.d(className = 'android.widget.EditText').set_text(txt)
            time.sleep(1)
            self.d(text = 'OK').click.wait()
            time.sleep(1)
        acc = TelephonySIP.Account(name, password, server)
        self.log_debug('Add SIP account %s'%acc)

        self.d(text = 'SIP accounts').click.wait()
        time.sleep(.5)
        self.d(description = 'Add account').click.wait()
        time.sleep(.5)
        set_text('Username', acc.name)
        set_text('Password', acc.password)
        set_text('Server', acc.server)
        for _ in range(2):
            if self.d(text='Save').exists:
                self.d(text='Save').click.wait()
                break
            else:
                self.d(text='SAVE').click.wait()
                break
        self.__accounts.append(acc)

    def enable_account(self, name, enable = True):
        '''
        Enable SIP account.
        Account is enabled by default. But sometimes need to switch on/off
        '''
        self.log_debug("Enable Account %s: %s"%(name, enable))
        matched_accs = filter(lambda x: x.name == name, self.__accounts)
        if not matched_accs:
            raise Exception("Enable account %s failed: No such accout"%name)
        elif len(matched_accs) >1:
            raise Exception('Multi accout found. All of them named "%s"'%name)
        acc = matched_accs[0]
        self.d(text = 'All calling accounts').click.wait()
        time.sleep(.5)
        switch = self.d(text = name).right(resourceId = 'android:id/switchWidget')
        status = switch.text
        if (enable and status == 'OFF') or \
                (not enable and status == 'ON'):
            switch.click.wait()
            time.sleep(.5)
            assert switch.text != status, 'Failed to change status'
        else:
            self.log_debug("No need to change")
        acc.enabled = bool(status == "ON")
        #self.d.press.back()
        time.sleep(1)

    def delete_account(self, name):
        '''
        Delete an account in UI
        '''
        self.d(text = 'SIP accounts').click.wait()
        time.sleep(.5)
        wgt = self.d(textStartsWith = name + '@')
        for i in range(wgt.count):
            self.log_debug('Delete account %s'%wgt.text)
            wgt.click.wait()
            time.sleep(.5)
            self.d(description = 'More options').click.wait()
            time.sleep(.5)
            self.d(text = 'Remove account').click.wait()
            time.sleep(1)
        accs = filter(lambda x:x.name == name, self.__accounts)
        for a in accs:
            self.__accounts.remove(a)

    def sip_call_only(self, enable = True):
        '''
        Select sip call only or for all calls in 'Use SIP Calling'
        '''
        self.log_debug("Use SIP for SIP call only: %s"%enable)
        self.d(text = 'Use SIP calling').click.wait()
        time.sleep(.5)
        all_calls = self.d(text = 'For all calls')
        sip_calls_only = self.d(text = 'Only for SIP calls')
        if enable:
            sip_calls_only.click.wait()
        else:
            all_calls.click.wait()
        time.sleep(.5)

    def call_with_sip(self, dial_to, use_sip):
        '''
        Call with sip account
        :dial_to name or name@server
        :sip_account signed sip account name
        '''
        self.log_debug("Call %s with SIP %s"%(dial_to, use_sip))
        if use_sip not in [_.name for _ in self.__accounts]:
            raise Exception("SIP account %s not setuped"%use_sip)
        #super(TelephonySIP, self).make_call_am(dial_to)
        cmd = "am start -a android.intent.action.CALL -d sip:" + dial_to
        self.adb.adb_cmd(cmd)
        self._call_with(use_sip)

    def wait_sip_call(self, timeout = 60):
        '''
        Wait for sip call
        SIP call is different with Phone calls.
        '''
        self.wait_call(timeout = timeout, answer = False)

    def answer_sip_call(self, enter_phone = False):
        '''
        Answer SIP call
        '''
        self.log_debug("Answer incoming SIP call")
        super(TelephonySIP, self).answer_call()
        time.sleep(.5)
        assert self.device.rpc.isStreamActive('voice_call', 0), "Receive SIP call failed!"
        if enter_phone:
            self._back_to_call()

    def hangup_sip_call(self):
        '''
        End SIP call
        '''
        self.log_debug("End SIP call by keyevent")
        cmd = 'input keyevent 6'
        self.adb.adb_cmd(cmd)
        time.sleep(1)
        assert not self.device.rpc.isStreamActive('voice_call', 0), 'End SIP call failed'

    def receive_incoming_sip_call(self, enable = True):
        '''
        Enable this option in settings
        '''
        self.log_debug('Enable receive SIP calls: %s'%enable)
        check_box = self.d(text = 'Receive incoming calls').right(resourceId = 'android:id/checkbox')
        status = check_box.checked
        if enable != bool(status) :
            check_box.click.wait()
            time.sleep(.5)
        else:
            self.log_debug("No need to change")
        assert enable == bool(check_box.checked), "Failed to setup this option"

    def enable_make_calls_option(self):
        '''
        Set up 'Make calls with' as 'Ask first' in calling settings.
        '''
        self.log_debug("Setup 'Make calls with' as 'Ask first'")
        if not self.d(text = 'Make calls with').wait.exists():
            self.log_debug("No such option, skip")
            return False
        self.d(text = 'Make calls with').click.wait()
        time.sleep(.5)
        self.d(text = 'Ask first').click.wait()
        time.sleep(.5)
        assert self.d(text = 'Ask first', resourceId = 'android:id/summary').exists,\
                "Setup Make calls with' as 'Ask first' failed!"
"""
    def dtmf(self, sym):
        '''
        Press sym through SIP dialer pad.
        Overwrite parent's dtmf
        '''
        self.d(text = str(sym), resourceId = 'com.android.dialer:id/dialpad_key_number').click.wait()
        time.sleep(.5)
"""