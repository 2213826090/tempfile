from testlib.audio.audio_log import AudioLogger
from testlib.util.common import g_common_obj
import time

class AudioHangouts(object):
    """
    Hangouts test for Audio domain
    """
    PKG_NAME = "com.google.android.talk"
    ACTIVITY_NAME = "com.google.android.talk.SigningInActivity"
    LAUNCH_APP_DELAY = 3

    def __init__(self, account, password,  device = None):
        """
        @summary: Audio Hangouts obj
        @param account: Stromg. account email
        @param password: String, account password
        """
        self.logger = AudioLogger.getLogger()
        self.device = device  if device else g_common_obj.get_test_device()
        self.d = device.get_device()
        self.__account_email = account# if account else self.config.read(section = "google_account").get("user_name")
        self.__account_psw = password# if password else self.config.read(section = "google_account").get("password")

    def clean_up_data(self):
        """
        @summary: Clear hangouts app data by adb shell pm clear
        @return: void
        """
        self.device.adb_cmd("pm clear %s" % self.PKG_NAME)

    def grant_permissions(self):
        """
        @summary: Grant all permissions that may used by hangouts
        """
        PRMSSN_CAMERA = "android.permission.CAMERA"
        PRMSSN_AUDIO = "android.permission.RECORD_AUDIO"
        cmd = "pm grant %s"%self.PKG_NAME
        self.device.adb_cmd("%s %s"%(cmd, PRMSSN_CAMERA))
        self.device.adb_cmd("%s %s"%(cmd, PRMSSN_AUDIO))

    def get_account(self):
        """
        @summary: Get current account info
        @return: (String account, String password)
        """
        return self.__account_email, self.__account_psw

    def launch_by_am(self, timeout = 20):
        """
        @summary: Launch hangouts by adb shell am
        @param timeout: int, default is 20s
        """
        self.logger.info("Launch Hangouts by am")
        #g_common_obj.launch_app_am(self.PKG_NAME, self.ACTIVITY_NAME)
        self.device.launch_app_am(self.PKG_NAME, self.ACTIVITY_NAME)
        time.sleep(self.LAUNCH_APP_DELAY)
        self.first_launch_guide()
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(self.LAUNCH_APP_DELAY)
            if self._is_launched():
                self.logger.debug("Launch Hnagouts succ in %ds"%(time.time() - start))
                return
        self.logger.error("Launch Hangouts by AM timeouted %ds"%timeout)
        raise Exception("Launch Hangouts timeout")

    def stop_by_am(self):
        """
        @summary: Stop hangouts by adb shell am
        """
        self.device.stop_app_am(self.PKG_NAME)

    def launch_from_home(self, timeout = 20):
        """
        @summary: Launch hangouts from home screen
        @param timeout: int, default is 20s
        """
        self.logger.info("Launch Hangouts from Home screen")
        self.d.press.home()
        #g_common_obj.launch_app_from_home_sc("Hangouts")
        self.device.launch_app_from_home_sc("Hangouts")
        time.sleep(self.LAUNCH_APP_DELAY)
        self.first_launch_guide()
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(self.LAUNCH_APP_DELAY)
            if self._is_launched():
                self.logger.debug("Launch Hangouts succ in %ds"%(time.time() - start))
                return
        self.logger.error("Launch Hangouts by Home screen timeouted %ds"%timeout)
        raise Exception("Launch Hangouts timeout")

    def switch_user(self, account):
        """
        @summary: Switch to account
        @param account: string, account email
        """
        self.logger.info("Switch to the account: %s"% account)
        self.navigation_menu()
        exists_account_wgt = self.d(text = account,
                resourceId = "com.google.android.apps.hangouts:id/account_address")
        switch_to_account_wgt = self.d(description = "Switch to %s"%account)
        if switch_to_account_wgt.exists:
            switch_to_account_wgt.click()
            time.sleep(1)
        elif exists_account_wgt.exists:
            self.logger.debug("Account %s already signed in."%account)
            self.d.press.back()
            time.sleep(.5)
        else:
            self.logger.error("Account %s not signed in."%account)
        assert self._is_home_page(), "Switch to exists account failed"

    def add_account(self, account, password):
        """
        @summary: Add new account, switch to account if already exists
        @param account: String, account email
        @param password: String, account password
        """
        self.logger.info("Add accout %s"%account)
        self.navigation_menu()
        self.d(resourceId = "com.google.android.apps.hangouts:id/account_list_button").click()
        time.sleep(1)
        exists_account_wgt = self.d(text = account,
                resourceId = "com.google.android.apps.hangouts:id/account_address")
        if exists_account_wgt.exists:
            self.logger.debug("Already signed in, switch to this account")
            self.switch_user(account)
            return
        else:
            add_account_wgt = self.d(text = "Add account", \
                    resourceId = "com.google.android.apps.hangouts:id/add_account_text")
            add_account_wgt.click.wait()
            if self.d(text = "Google", packageName = "com.android.settings").exists:
                self.d(text = "Google").click.wait()
            self._sign_in(account, password)

    def remove_account(self, account):
        """
        @summary: Sign out hangouts
        """
        SETTINGS_PKG = "com.android.settings"
        self.logger.info("Remove account %s "%account)
        self.navigation_menu()
        self.d(resourceId = "com.google.android.apps.hangouts:id/account_list_button").click()
        time.sleep(1)
        exists_account_wgt = self.d(text = account,
                resourceId = "com.google.android.apps.hangouts:id/account_address")
        if not exists_account_wgt.exists:
            self.logger.warn("Account not exists, may not signed in. Skip!")
            return
        self.d(text = "Manage accounts",
               resourceId = "com.google.android.apps.hangouts:id/manage_accounts_text"
               ).click.wait()
        self.d(text = "Google", resourceId = "android:id/title").click.wait()
        time.sleep(.5)
        #if self.d(text = "Accounts", resourceId = "android:id/title").exists:
        self.d(text = account).click.wait()
        time.sleep(.5)
        self.d(text = "Sync").sibling(description = "More options").click()
        time.sleep(.5)
        self.d(text = "Remove account", resourceId = "android:id/title").click()
        time.sleep(.5)
        self.d(text = "Remove account", resourceId = "android:id/button1").click()
        time.sleep(5)
        assert not self.d(text = account).exists, "Remove account failed!"
        time.sleep(1)
        self.d.press.back()
        time.sleep(.5)
        if self.d.info.get("currentPackageName") == SETTINGS_PKG:
            self.d.press.back()
            time.sleep(.5)
        assert self._is_home_page(), "Back to home page after remove account failed."

    def navigation_menu(self):
        """
        @summary: Open navigation menu from main screen. Do nothing if already launched.
        """
        self.logger.debug("Open navigation menu")
        drawer_wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/navigation_drawer")
        if self._in_navigation_menu():
            self.logger.debug("Already in navigation menu, skipped!")
            return

        self.d(resourceId = "com.google.android.apps.hangouts:id/hangouts_toolbar")\
                .child(index = 0, className = "android.widget.ImageButton").click()
        time.sleep(.5)
        assert self._in_navigation_menu(), "Open navigation menu failed!"

    def search_contact(self, contact):
        """
        @summary: Search contact in contacts list or google+
        @param contact: string, google account
        """
        self.logger.info("Search contact %s"%contact)
        self.navigation_menu()
        self.d(resourceId = "com.google.android.apps.hangouts:id/text1", text = "Contacts").click.wait()
        time.sleep(1)
        edit_text_wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/search_area")
        edit_text_wgt.wait.exists(timeout = 5000)
        assert edit_text_wgt.exists, "Search area not found, Search contact failed!"
        assert edit_text_wgt.set_text(contact), "Input contact failed!"
        time.sleep(3)
        if self.d(text = "Search on Google+").exists:
            self.d(text = "Search on Google+").click()
            time.sleep(5)
        if self.d(scrollable = True).exists:
            self.d(scrollable = True).scroll.to(textStartsWith = contact.split("@")[0])
        self.d(resourceId = "android:id/list").child(textStartsWith = contact.split("@")[0]).click()
        time.sleep(1)
        self.d(text = "Hangouts message").click.wait()
        time.sleep(1)
        #assert self._is_in_conversation(), "Not message interface"
        sent_button = self.d(resourceId = "com.google.android.apps.hangouts:id/floating_send_button")
        sent_button.wait.exists()

    def first_launch_guide(self):
        """
        @summary: Init the guide when launch hangouts first time.
        """
        GMS_PKG = "com.google.android.gms"
        self.logger.info("OOBE if the first time launched.")
        if self.d.info.get("currentPackageName") == GMS_PKG:
            self._sign_in(self.__account_email, self.__account_psw)
        skip_wgt = self.d(text = "Skip", resourceId = "com.google.android.apps.hangouts:id/promo_button_no")
        skip_wgt_N = self.d(text="SKIP", resourceId="com.google.android.apps.hangouts:id/promo_button_no")
        cnfm_wgt = self.d(text = "Okay, got it", className = "android.widget.Button")
        retry = 1
        while not self._is_home_page():
            time.sleep(1)
            if retry > 10:
                self.logger.error("Can't finish OOBE in 10 loops")
                break
            if skip_wgt.exists:
                skip_wgt.click.wait()
                time.sleep(.5)
            if skip_wgt_N.exists:
                skip_wgt_N.click.wait()
                time.sleep(.5)
            if cnfm_wgt.exists:
                cnfm_wgt.click.wait()
                time.sleep(.5)
            if self._in_navigation_menu():
                self.d.press.back()
                time.sleep(.5)
            retry += 1
        assert self._is_home_page(), "Fist launch guide setup failed."

    def send_msg(self, text):
        """
        @summary: Send a text message to pointed contact.
        @param text: string, text msg
        """
        self.logger.info("Send message: %s"%text)
        assert self._is_in_conversation(), "Should in conversation page before this function"
        self.d(resourceId = "com.google.android.apps.hangouts:id/message_text").set_text(text)
        time.sleep(.5)
        self.d(resourceId = "com.google.android.apps.hangouts:id/floating_button_with_counter").click()
        time.sleep(.5)
        #assert self.d(resourceId = "android:id/list").child(text = text).exists, "Send mssage failed!"

    def make_voice_call(self):
        """
        @summary: Make a voice call
        """
        assert self._is_in_conversation(), "Should in conversation page before this function"
        self._make_call(type = "voice")

    def make_video_call(self):
        """
        @summary: Make a video call.
        """
        assert self._is_in_conversation(), "Should in conversation page befor this function"
        self._make_call(type = "video")

    def waitfor_msg(self, text = "test message", timeout = 60):
        """
        @summary: Wait for hangout message come.
        @param text: string, a message string that expected to receive
        @param timeout: int
        """
        self.logger.info("Waitfor msg, expected text: %s"%text)
        start = time.time()
        self.d.open.notification()
        time.sleep(1)
        received = False
        while time.time() - start < timeout:
            time.sleep(1)
            if self.d(textStartsWith = text[:10]).exists:
                self.logger.debug("Incomming msg in %ds"%(time.time() - start))
                received = True
                break
        assert received,"Waitfor message come timeouted: %ds"%timeout

        self.d(text = text).click.wait()
        self.d(resourceId = "android:id/list").wait.exists(timeout = 5000)

        assert self.d(resourceId = "android:id/list").child(text = text).exists,\
                "Check received msg failed!"

    def waitfor_call(self, timeout = 60):
        """
        @summary: Wait for hangout voice come.
        @param timeout: int
        """
        self.logger.info("Waiting for incoming call")
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(1)
            if self.device.rpc.isStreamActive("ring",0):
                self.logger.debug("Incomming call in %ds"%(time.time() - start))
                return True
        raise Exception("Wait for call timouted: %ds"%timeout)

    def accept_call(self):
        """
        @summary: Accept the voice call that comes.
        """
        #assert self._is_in_conversation(), "Should in conversation page befor this function"
        self._reply_incoming_call(accept = True)

    def audio_mute(self, enable = True):
        """
        @summary: Mute/unmute the voice/video call
        @param param: mute, boolean
        """
        self._mute_unmute(type = "audio", mute = enable)

    def video_mute(self, enable = True):
        """
        @summary: Video mute/unmute in video call/voice call
        @param param: enable, boolean
        """
        self._mute_unmute(type = "video", mute = enable)

    def decline_call(self):
        """
        @summary: Ignore the incoming video call
        """
        self._reply_incoming_call( accept = False)

    def end_call(self):
        """
        @summary: End voice call
        """
        self.logger.info("End call")
        assert self._is_in_call(), "Not in call"
        wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")
        self._call_up_self_menu()
        hang_up_btn = wgt.child_by_description("Hang up", resourceId = "com.google.android.apps.hangouts:id/action_icon")
        assert hang_up_btn.exists, "Hang up btn not found, could not end call"
        hang_up_btn.click()
        hang_up_btn.wait.gone(timeout = 5000)
        time.sleep(1)
        assert not hang_up_btn.exists, "Click hang up btn to end call failed."

    def clear_hangouts_list(self):
        """
        @summary: Clear all hangouts on hangouts list
        """
        pass

    def delete_hangout(self):
        """
        @summary: Delete current hangout
        """
        self.logger.info("Delete current hangout")
        assert self._is_in_conversation(), "Should in conversation page befor this function"
        toolbar = self.d(resourceId = "com.google.android.apps.hangouts:id/hangouts_toolbar")
        assert toolbar.exists, "Can't get toolbar."
        menu = toolbar.child_by_description("More options", className = "android.widget.ImageView")
        menu.click()
        time.sleep(.5)
        self.d(text = "Delete", resourceId = 'com.google.android.apps.hangouts:id/title').click()
        time.sleep(.5)
        self.d(text = "Delete", resourceId = "android:id/button1").click()
        assert self._is_home_page(), "Delete current hangouts failed."

    def set_audio_route_bt(self):
        """
        @summary: Set audio route to BT
        """
        return self._set_audio_route("BT")

    def set_audio_route_default(self):
        """
        @summary: Set audio route to handset
        """
        return self._set_audio_route("handset")

    def set_audio_route_headset(self):
        """
        @summary: Set audio route to handset
        """
        return self._set_audio_route("headset")

    def set_audio_route_speaker(self):
        """
        @summary: Set audio route to speaker
        """
        return self._set_audio_route("speaker")

    def _set_audio_route(self, route):
        """
        @summary: Set audio route to BT or Wired headphones
        @param route: String, "BT", "handset","speaker", "headset"
        """
        self.logger.debug("Set audio route to %s"%route)
        assert self._is_in_call(), "Not in call"

        BT, handset, speaker, headset = "BT", "handset","speaker", "headset"
        key_map = {
                   BT       : "Bluetooth",
                   handset  : "Handset",
                   speaker  : "Speaker",
                   headset  : "Wired headphones",
                   }
        assert route in key_map , 'route args should be "BT", "handset","speaker", "headset"'

        retry = 5

        wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/audio_device_menu_item")
        self._call_up_self_menu()

        wgt.click.wait()
        while not wgt.description.startswith(key_map.get(route)):
            if retry < 0:
                raise Exception("Change route to %s failed. Retry for %d times"%(route, 5))
            self._call_up_self_menu()
            wgt.click.wait()
            time.sleep(.5)
            wgt.wait.exists()
            retry -=1

    def _is_launched(self):
        """
        @summary: Return True if hangouts launched else False
        """
        result = self.d.info.get("currentPackageName") == self.PKG_NAME
        return result

    def _is_signed_in(self):
        """
        @summary: Return True if hangouts signed in else False
        """
        pass

    def _is_home_page(self):
        """
        @summary: Is hangouts default main page or not
        @return: True or False
        """
        is_home = self.d(resourceId = "android:id/tabhost", className = "android.widget.TabHost").exists
        in_menu = self.d(resourceId = "com.google.android.apps.hangouts:id/navigation_drawer").exists
        return is_home and not in_menu

    def _is_in_conversation(self):
        """
        @summary: In conversation page or not
        """
        sent_button = self.d(resourceId = "com.google.android.apps.hangouts:id/floating_send_button")
        toolbar_item = self.d(resourceId = "com.google.android.apps.hangouts:id/start_hangout_menu_item")
        return sent_button.exists or toolbar_item.exists

    def _sign_in(self, account, password):
        """
        @summary: Sign in hangouts if not
        @param account: string, google account, default is None
        """
        def wait_for(wgt, timeout):
            start = time.time()
            while time.time() - start < timeout:
                time.sleep(3)
                if wgt.exists: return True
            return False

        TIME_OUT = 30

        self.logger.info("Sign in with account %s"%account)
        account_wgt = self.d(resourceId = "identifierId", className ="android.widget.EditText")
        if not wait_for(account_wgt, TIME_OUT):
            raise Exception("Failed to add account")
        account_wgt.set_text(account)
        time.sleep(1)
        self.d.press.enter()
        time.sleep(1)

        self.logger.debug("Enter Password")
        psw_wgt = self.d(resourceId = "password", className = "android.widget.EditText")
        if not wait_for(psw_wgt, TIME_OUT):
            raise Exception("Failed to enter password")
        psw_wgt.set_text(password)
        time.sleep(1)
        #self.d(resourceId = 'identifierNext', className = "android.widget.Button").click()
        self.d.press.enter()
        time.sleep(1)

        self.logger.debug("Accept the policy")
        accept_wgt = self.d(description="ACCEPT", className = "android.widget.Button")
        if not wait_for(accept_wgt, TIME_OUT):
            raise Exception("Failed to sign in")
        accept_wgt.click.wait()
        time.sleep(1)

        self.logger.debug("Skip payment setup!")
        if wait_for(self.d(text = "Set up payment info"), TIME_OUT):
            self.d(text = "No thanks").click()
            time.sleep(.5)
            self.d(text = "Continue", resourceId = "com.android.vending:id/positive_button").click()

        self.logger.debug("Disable agree backup")
        agree_backup_wgt = self.d(resourceId = "com.google.android.gms:id/agree_backup",
                                  className = "android.widget.CheckBox")
        if wait_for(agree_backup_wgt, TIME_OUT):
            agree_backup_wgt.click()
            time.sleep(.5)
            self.d(text = "Next", className = 'android.widget.Button').click.wait()

        self.logger.debug("Wait for hangouts main page")
        time.sleep(5)
        assert self._is_launched(), "Failed to back to hangouts after add account"

    def _mute_unmute(self, type = "video", mute = True):
        """
        @summary: mute/unmute
        @param mute: True or False, default is True
        @param type: string, should be 'video' or 'audio'
        """
        self.logger.info("Type: %s, mute: %s"%(type, str(mute)))
        def is_muted(btn):
            content_description = btn.info.get("contentDescription")
            self.logger.debug("status %s"%content_description)
            assert content_description, "Get video mute/unmute btn failed!"
            if content_description.startswith("Mute"):
                return False
            elif content_description.startswith("Unmute"):
                return True
            else:
                raise Exception("Unknown video btn status, not Mute or Unmute")

        muted = None
        wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")

        if type == "video":
            switch_btn = wgt.child(index = 2, className = "android.widget.FrameLayout").child(index  = 0)
        elif type == "audio":
            switch_btn = wgt.child(index = 0, className = "android.widget.FrameLayout").child(index  = 0)
        else:
            raise Exception("unknown type of mute/unmute btn")
        assert switch_btn.exists, "%s btn not exists"%type
        self._call_up_self_menu()
        muted = is_muted(switch_btn)
        if mute and not muted:
            self._call_up_self_menu()
            switch_btn.click()
            time.sleep(.5)
            assert is_muted(switch_btn), "Turn video on failed."
        elif not mute and muted:
            self._call_up_self_menu()
            switch_btn.click()
            time.sleep(.5)
            #self._permission_allow()
            time.sleep(.5)
            self._call_up_self_menu()
            assert not is_muted(switch_btn), "Turn video off failed."

    def _make_call(self, type = "voice"):
        """
        @summary: Make voice/video call in message interface
        @param type: string, 'voice' or 'video'
        """
        self.logger.info("make %s call"%type)
        if type == "video":
            wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/start_hangout_menu_item")
        elif type == "voice":
            wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/realtimechat_conversation_call_menu_item")
        else:
            self.logger.error("Unkown type of hangouts: %s"%type)
            raise Exception("Unkown type of hangouts, should be video or voice")
        wgt.click.wait()
        wgt.wait.gone(timeout = 10000)
        #self._permission_allow()
        time.sleep(.5)
        assert self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")\
                .wait.exists(timeout = 30000), "Make %s call failed"%type

    def _reply_incoming_call(self, accept = True):
        """
        @summary: accept/ignore incoming voice/video call
        @param accept: True or False, accept if True, else ignore
        """
        self.logger.debug("Reply incoming call: accept = %s"%str(accept))
        #incoming_call_wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/ongoing_hangout_bar")
        #assert incoming_call_wgt.exists, "Incoming call disappear, may timeouted."
        self.d.open.notification()
        time.sleep(1)
        retry = 10
        if accept:
            if self.d(text = "Answer", className = "android.widget.Button").exists:
                reply_wgt = self.d(text = "Answer", className = "android.widget.Button")
            if self.d(text = "ANSWER", className = "android.widget.Button").exists:
                reply_wgt = self.d(text="ANSWER", className="android.widget.Button")
        else:
            if self.d(text = "Decline", className = "android.widget.Button").exists:
                reply_wgt = self.d(text = "Decline", className = "android.widget.Button")
            if self.d(text = "DECLINE", className = "android.widget.Button").exists:
                reply_wgt = self.d(text="DECLINE", className="android.widget.Button")
        for i in range(retry):
            if self.d(textStartsWith = "You are invited to").exists:
                self.d(textStartsWith = "You are invited to").click.wait()
                time.sleep(1)
            if reply_wgt.exists:
                reply_wgt.click.wait()
                time.sleep(1)
                #self._permission_allow()
                break
            time.sleep(1)
        assert self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")\
                .wait.exists(timeout = 10000), "%s call failed"%("accept" if accept else "Decline")

    def _is_hangouts_list_empty(self):
        """
        @summary: Return True if all hangouts cleared else False
        @return: True or False
        """
        result = self.d(resourceId = "android:id/empty").exists
        return result

    def _in_navigation_menu(self):
        """
        @summary: Is in navigation menu or not
        @return: boolean
        """
        drawer_wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/navigation_drawer")
        return drawer_wgt.exists

    def _permission_allow(self):
        """
        @summary: Press 'Allow' if meet permission request.
        """
        allow_wgt = self.d(text = "Allow", className = "android.widget.Button")
        timeout = 6
        start = time.time()
        while time.time() - start < timeout:
            if allow_wgt.exists:
                allow_wgt.click.wait()
                time.sleep(.5)
            time.sleep(.5)

    def _is_in_call(self):
        """
        @summary:  Is in Video/Audio call.
        @return: boolean
        """
        self_tray = self.d(resourceId = "com.google.android.apps.hangouts:id/self_tray_container")
        self_menu = self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")
        return self_tray.exists or self_menu.exists

    def _call_up_self_menu(self):
        wgt = self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_self_menu")
        middle_view = self.d(resourceId = "com.google.android.apps.hangouts:id/hangout_focused_participant_view")
        retry = 5
        while 1:
            if wgt.exists:
                return
            if retry == 0:
                raise Exception("Failed to call up self tray container view")
            middle_view.click.wait()
            retry -= 1
