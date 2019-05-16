#Copyright (C) 2014  Chen mei <meix.chen@intel.com@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

import os
import time
from testlib.util.common import g_common_obj
from testlib.util.device import TestDevice


class APStatus(object):
    """
    constant class for AP status
    """

    Connected = 'Connected'
    NotInRange = "Not in range"
    Saved = "Saved"
    NONE = ""
    WPSAvailable = "WPS available"
    Connecting = "Connecting"
    Authenticating = "Authenticating"
    AProblem = "Authentication problem"
    Disable = "Disabled"
    Disconnected = "Disconnected"

    NotConnected = [
        Saved, NONE, WPSAvailable,
        AProblem, Disable, Disconnected]


class WiFiSecurity(object):
    """
    constant class for wifi security status
    """

    SEC_None = "None"
    SEC_WEP = "WEP"
    SEC_WPA_WPA2 = "WPA/WPA2 PSK"
    SEC_802_1x = "802.1x EAP"

    secs_list = [SEC_None, SEC_WEP, SEC_WPA_WPA2, SEC_802_1x]


class WifiAP:
    """
    WifiAP definition
    """

    def __init__(self, cfg):
        """Get value from config"""

        self.name = cfg.get("name")
        self.passwd = cfg.get("passwd")
        self.ssid = cfg.get("ssid")
        self.security = cfg.get("security")
        self.identity = cfg.get("identity")
        self.eap = cfg.get("eap")
        self.phase = cfg.get("phase")
        self.ca = cfg.get("ca")
        self.usr = cfg.get("usr")
        self.conf = cfg.get("conf")
        self.dconf = cfg.get("dconf")
        self.type = cfg.get("type")
        self.setname = cfg.get("setname")
        self.pin = cfg.get("pin")
        self.aconf = cfg.get("aconf")
        self.apchangetime = cfg.get("apchangetime")
        self.enable = cfg.get("enable")
        self.times = cfg.get("times")
        self.number = cfg.get("num")


class WifiSettingImpl:
    """
    Implements WiFi Setting UI actions.
    """

    setting_pkg_name = "com.android.settings"
    setting_activity_name = ".Settings"

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            """Init environment"""
            self.d = device

        @property
        def wifi_settings(self):
            """ UI 'Wi-Fi' display on Settings, \
            click it enter Wi-Fi settings """
            return self.d(textMatches="Wi.*Fi")

        @property
        def settings_title(self):
            """ UI setting_title"""
            return self.d(
                resourceId="android:id/action_bar_title", text="Settings")

        @property
        def onoff_switch(self):
            """ UI switch turn on/off wifi """
            return self.d(resourceId="com.android.settings:id/switch_widget")

        @property
        def ap_list_view(self):
            """ UI AP list view, which list out all added APs """
            return self.d(resourceId="android:id/list")

        @property
        def ap_view(self):
            """ UI AP view"""
            return self.d(resourceId="android:id/title")

        @property
        def label_connected(self):
            """UI 'Connected' appear when AP connected success"""
            try:
                if self.ap_list_view.child_by_text(
                        "Connected", allow_scroll_search=True).exists:
                    return self.d(
                        text="Connected", resourceId="android:id/summary")
            except:
                return None
            return None

        def ap_title(self, ssid):
            """ UI label which display AP ssid """
            try:
                if self.ap_list_view.child_by_text(
                    ssid, allow_scroll_search=True,
                        resourceId="android:id/title").exists:
                    return self.d(text=ssid, resourceId="android:id/title")
            except:
                return None
            return None

        def ap_summary(self, ssid):
            """UI label which display AP \
            summary(Saved, Connected, Secured with ...)"""
            try:
                if self.ap_list_view.child_by_text(
                    ssid, allow_scroll_search=True,
                        resourceId="android:id/title").exists:
                    if self.d(
                        text=ssid, resourceId="android:id/title").sibling(
                            resourceId="android:id/summary").exists:
                        return self.d(
                            text=ssid, resourceId="android:id/title").sibling(
                                resourceId="android:id/summary")
            except:
                return None
            return None

        def ap_connected(self, ssid=None):
            """UI connected AP"""
            if self.label_connected is None:
                return None
            if not ssid:
                return self.label_connected.up(resourceId="android:id/title")
            else:
                if self.label_connected.up(
                        text=ssid, resourceId="android:id/title") is None:
                    return None
                if self.label_connected.up(
                        text=ssid, resourceId="android:id/title").exists:
                    return self.label_connected.up(
                        text=ssid, resourceId="android:id/title")
            return None

        @property
        def btn_forget(self):
            """ UI button Forget when forget ap """
            return self.d(text="Forget")

        @property
        def btn_connect(self):
            """ UI button Connect """
            return self.d(text="Connect")

        @property
        def btn_continue(self):
            """ UI button Continue"""
            return self.d(text="Continue")

        @property
        def btn_cancel(self):
            """ UI button Cancel """
            return self.d(text="Cancel")

        @property
        def btn_save(self):
            """ UI button Save when add ap """
            return self.d(text="Save")

        @property
        def btn_connected(self):
            """ UI button Connected """
            return self.d(text="Connected")

        @property
        def btn_more_options(self):
            """ UI button more options """
            return self.d(description="More options")

        @property
        def btn_scan_switch(self):
            """ UI button advanced """
            return self.d(
                textContains="Scanning").right(
                    resourceId="android:id/switchWidget")

        @property
        def btn_advanced(self):
            """ UI button advanced """
            return self.d(textMatches="Advanced")

        @property
        def btn_refresh(self):
            """ UI button refresh """
            return self.d(text="Refresh")

        @property
        def btn_saved(self):
            """ UI button saved """
            return self.d(textMatches="Saved networks")

        @property
        def btn_add_ap(self):
            """ UI button add ap """
            return self.d(text="Add network")

        @property
        def btn_none(self):
            """ UI button None """
            return self.d(text="None")

        @property
        def screen_lock(self):
            """ UI screen lock """
            return self.d(text="Screen lock")

        @property
        def pin_entery(self):
            """ UI PIN entery """
            return self.d(resourceId="com.android.settings:id/password_entry")

        @property
        def pin(self):
            """ UI PIN """
            return self.d(text="PIN")

        @property
        def install_cer(self):
            """ UI Install certificates """
            return self.d(text="Install certificates")

        @property
        def recent(self):
            """ UI recent """
            return self.d(text="Recent")

        @property
        def confirm_pin(self):
            """ UI confirm PIN """
            return self.d(text="Confirm your PIN")

        @property
        def open_from(self):
            """ UI Open from """
            return self.d(text="Open from")

        @property
        def clear_credenty(self):
            """ UI Clear credentials"""
            return self.d(text="Clear credentials")

        @property
        def ssid(self):
            """ UI input field for 'Network SSID' """
            return self.d(resourceId="com.android.settings:id/ssid")

        @property
        def network_list(self):
            """ UI network list """
            return self.d(resourceId="android:id/icon")

        @property
        def security(self):
            """ UI dropdown for 'Security' """
            return self.d(resourceId="com.android.settings:id/security")

        def security_option(self, sec):
            """ UI option of 'Security' dropdown """
            return self.d(textMatches=sec)

        @property
        def eap_method(self):
            """ UI dropdown for 'EAP method' """
            return self.d(resourceId="com.android.settings:id/method")

        def eap_option(self, eap):
            """ UI option of 'EAP method' dropdown """
            return self.d(textMatches=eap)

        @property
        def phase_method(self):
            """ UI dropdown for 'Phase 2' """
            return self.d(resourceId="com.android.settings:id/phase2")

        def phase_option(self, phase):
            """ UI option of 'Phase 2' dropdown """
            return self.d(textMatches=phase)

        @property
        def ca_method(self):
            """ UI dropdown for 'CA certificate' """
            return self.d(resourceId="com.android.settings:id/ca_cert")

        def ca_option(self, cacert):
            """ UI option of 'CA certificate' dropdown """
            return self.d(
                textMatches=cacert, className="android.widget.CheckedTextView")

        @property
        def usr_method(self):
            """ UI dropdown for 'User certificate' """
            return self.d(resourceId="com.android.settings:id/user_cert")

        def usr_option(self, usrcert):
            """UI option of 'User certificate' dropdown"""
            return self.d(
                textMatches=usrcert,
                className="android.widget.CheckedTextView")

        @property
        def identity(self):
            """ UI dropdown for 'identity' """
            return self.d(resourceId="com.android.settings:id/identity")

        @property
        def password(self):
            """ UI input field for 'Password' """
            return self.d(resourceId="com.android.settings:id/password")

        @property
        def btn_extract_certificate_dialog(self):
            """ UI Extract certificate dialog title """
            return self.d(
                resourceId="android:id/alertTitle", text="Extract certificate")

        @property
        def extract_certificate_password(self):
            """ UI input field for certificate password """
            return self.d(
                resourceId="com.android.certinstaller:id/credential_password")

        @property
        def certificate_name(self):
            """ UI input field for certificate name """
            return self.d(
                resourceId="com.android.certinstaller:id/credential_name")

        @property
        def credential_usage(self):
            """ UI credential usage """
            return self.d(
                resourceId="com.android.certinstaller:id/credential_usage")

        @property
        def btn_credential_use(self):
            """ UI credential use """
            return self.d(
                resourceId="android:id/text1", textMatches="Wi.*Fi")

        @property
        def btn_ok(self):
            """ UI button OK """
            return self.d(text="OK")

        @property
        def btn_done(self):
            """ UI button Done """
            return self.d(text="Done")

        @property
        def btn_certification_title(self):
            """ UI button certification title """
            return self.d(resourceId="android:id/action_bar_title")

        @property
        def btn_certification_openfrom(self):
            """ UI button Open from """
            return self.d(
                text="Open from", resourceId="android:id/action_bar_title")

        @property
        def btn_certification_storage(self):
            """ UI button storage """
            return self.d(
                text="Internal storage", resourceId="android:id/title")

        @property
        def btn_certification_intall(self):
            """ UI button download folder base dir """
            return self.d(resourceId="android:id/up")

        @property
        def btn_download_folder(self):
            """ UI button download folder """
            return self.d(
                resourceId="com.android.documentsui:id/list").child_by_text(
                    "Download", allow_scroll_search=True)

        @property
        def btn_download(self):
            """ UI button download folder """
            return self.d(text="Download")

        @property
        def btn_attention(self):
            """ UI button download folder """
            return self.d(
                text="Attention", resourceId="android:id/title")

        @property
        def switch_text(self):
            """ Text for switch """
            return self.d(resourceId="com.android.settings:id/switch_text")
    #------------------------ end of class Locator ----------------------

    def __init__(self, cfg):
        """Init environment"""
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = WifiSettingImpl.Locator(self.d)

    def set_orientation_n(self):
        """Set orientation as n"""
        self.d.orientation = "n"

    def setup_connection(self):
        """Set up the connection"""
        self.d = g_common_obj.get_device()
        value = False
        for i in range(5):
            if self.d.server.alive:
                value = True
                break
            else:
                self.d.server.stop()
                self.d.server.start()
        if value:
            print "[INFO]: --- Success to set up connection"
        else:
            print "[INFO]: --- Fail to set up connection"
        assert value, True

    def launch_from_am(self):
        """Launch Wi-Fi Setttings from am"""
        print "[INFO] Launch Wi-Fi Setttings from am"
        cmd = 'am start -a android.settings.WIFI_SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "[FAILURE] Launch Wi-Fi Setttings fail"
        if g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip() == '[O]':
            ret = int(res_list[-1][-1].strip())
        else:
            ret = int(res_list[-1].strip())
        assert ret == 0, msg_fail
        self._locator.onoff_switch.wait.exists(timeout=3*1000)
        if self._locator.onoff_switch.exists is False:
            self._locator.onoff_switch.click()
            time.sleep(1)
        assert self._locator.onoff_switch.exists, msg_fail
        print "[INFO] Launch Wi-Fi Setttings success"

    @staticmethod
    def stop_from_am():
        """Stop Wi-Fi Setttings from am"""
        print "[INFO] Stop Wi-Fi Setttings from am"
        g_common_obj.stop_app_am(WifiSettingImpl.setting_pkg_name)

    def launch_from_settings(self):
        """Launch Wi-Fi from Settings"""
        print "[INFO] Launch Wi-Fi from Settings"
        msg_fail = "[FAILURE] Launch Wi-Fi Setttings fail"
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc("Settings")
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(textMatches="Wi.*Fi")
            time.sleep(1)
            self._locator.wifi_settings.click()
        assert self._locator.btn_more_options.exists, msg_fail

    def tls_exist(self, wifiap):
        """TLS judgement"""
        if wifiap.eap == "TLS":
            return True
        else:
            return False

    def turn_on_wifi(self, waitconnect=False):
        """Turn on WiFi"""
        return self.on(waitconnect)

    def on(self, waitconnect=False):
        """Turn on WiFi"""
        print "[INFO] Turn on Wi-Fi."
        if self._locator.onoff_switch.info['text'] == "ON":
            print "[INFO] Wi-Fi is already ON"
            return
        else:
            self._locator.onoff_switch.click()
            self.__wait_wifi_onoff_comp(True)
            assert self._locator.onoff_switch.info[
                'text'] == "ON", "Turn on Wi-Fi fail."
        if waitconnect is True:
            # wait any one AP connected
            ret, _ = self.__wait_ap_con_comp(2)
            if ret is False:
                pass
            else:
                assert self._locator.label_connected is not None, "\
                No AP connected."
        print "[INFO] Turn on Wi-Fi success."

    def turn_off_wifi(self):
        """Turn off wifi"""
        return self.off()

    def off(self):
        """Turn off wifi"""
        print "[INFO] Turn off Wi-Fi."
        if self._locator.onoff_switch.info['text'] == "OFF":
            return
        else:
            self._locator.onoff_switch.click()
            self.__wait_wifi_onoff_comp(False)
            assert self._locator.onoff_switch.info[
                'text'] == "OFF", "Turn off Wi-Fi fail."
        print "[INFO] Turn off Wi-Fi success."

    def connect_to_ap(self):
        """Connect to default ap"""
        self.connect_ap(WifiAP(self.cfg), False, False)

    def connect_ap(self, wifiap, forget=True, ignore_exist=True):
        """Connect to ap"""
        print "[INFO] Connect to ap: %s" % wifiap.ssid
        # forget conneted ap, if there is
        if forget:
            self.forget_all_ap()
        ap_status = self.get_ap_status(wifiap)
        if APStatus.AProblem in ap_status or \
            APStatus.Disable in ap_status or \
                APStatus.Disconnected in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_forget.click()
            print "[INFO] Sleep 15s to refresh wifi list"
            time.sleep(15)
        # add ap directly
        if ignore_exist:
            self.__add_ap(wifiap)
            self.check_connected(wifiap.ssid)
        else:
            # if ap has exists
            timeout = 30
            while self._locator.ap_title(wifiap.ssid) is None:
                self.scan()
                time.sleep(3)
                timeout -= 3
                print "[INFO] refresh"
                if self._locator.ap_title(wifiap.ssid) is not None:
                    print "[INFO] AP applied completely"
                    break
                if timeout <= 0:
                    print "[INFO] fresh time is out"
                    break
            if self._locator.ap_title(wifiap.ssid) is not None:
                self.__con_ap(wifiap)
                time.sleep(3)
                if self._locator.ap_summary(wifiap.ssid) is None:
                    self.__con_ap(wifiap)
                self.check_connected(wifiap.ssid)
            else:
                print "Error: can't find the ssid of connect ap"

    def ap_enable(self, wifiap):
        """Whether ap need enable"""
        if wifiap.enable:
            return True
        else:
            return False

    def connect_ap_with_wrongpw(self, wifiap, forget=True, ignore_exist=True):
        """Connect to ap  with wrong password"""
        print "[INFO] Connect to ap %s with wrong password" % wifiap.ssid
        if forget:
            self.forget_all_ap()
        # add ap directly
        if ignore_exist:
            self.__add_ap(wifiap)
            self.check_connected(wifiap.ssid, False)
        else:
            # if ap has exists
            timeout = 30
            while self._locator.ap_title(wifiap.ssid) is None:
                self.scan()
                time.sleep(3)
                timeout -= 3
                if self._locator.ap_title(wifiap.ssid) is not None:
                    print "[INFO] AP applied completely"
                    break
                if timeout <= 0:
                    print "[INFO] fresh time is out"
                    break
            if self._locator.ap_title(wifiap.ssid) is not None:
                self.__con_ap(wifiap)
                time.sleep(3)
                if self._locator.ap_summary(wifiap.ssid) is None:
                    self.__con_ap(wifiap)
                self.check_connected(wifiap.ssid, False)
            else:
                print "Error: can't find the ssid of connect ap"

    def connect_ap_without_check(self, wifiap, forget=True):
        """Connect to ap without checking whether the ap is connected"""
        print "[INFO] Connect to ap: %s" % wifiap.ssid
        # forget conneted ap, if there is
        if forget:
            self.forget_all_ap()
        ap_status = self.get_ap_status(wifiap)
        if APStatus.AProblem in ap_status \
            or APStatus.Disable in ap_status \
                or APStatus.Disconnected in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_forget.click()
            print "[INFO] Sleep 15s to refresh wifi list"
            time.sleep(15)
        # if ap has exists
        if self._locator.ap_title(wifiap.ssid) is not None:
            self.__con_ap(wifiap)
        else:
            self.__add_ap(wifiap)

    def get_ap_status(self, wifiap):
        return self.get_status(wifiap)

    def get_status(self, wifiap):
        """Get ap connect status"""
        ap_status = APStatus.NONE
        if self._locator.ap_summary(wifiap.ssid) is not None:
            if self._locator.ap_summary(wifiap.ssid):
                ap_status = self._locator.ap_summary(wifiap.ssid).info['text']
            timeout = 180
            while APStatus.Connecting in ap_status or \
                APStatus.Authenticating in ap_status or \
                    APStatus.Saved in ap_status:
                if APStatus.Saved in ap_status:
                    self._locator.ap_title(wifiap.ssid).click()
                    self._locator.btn_connect.click()
                print "[INFO] ap connecting, wait 5s!"
                time.sleep(5)
                timeout -= 5
                if timeout <= 0:
                    print "Error: the ap is always connecting!"
                    break
                ap_status = self._locator.ap_summary(wifiap.ssid).info['text']
        return ap_status

    def __con_ap(self, wifiap):
        """Connect to AP according to AP Status"""
        time.sleep(5)
        ap_status = self.get_status(wifiap)
        print "[INFO] The %s status: %s" % (wifiap.ssid, ap_status)
        if APStatus.Connected in ap_status:
            print "[INFO] AP is already connected"
            return
        elif APStatus.Saved in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_connect.click()
        elif APStatus.NONE in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            if self._locator.eap_method.exists and wifiap.eap:
                print "[INFO] Select eap"
                self._locator.eap_method.click()
                self._locator.eap_option(wifiap.eap).click()
            if wifiap.phase:
                print "[INFO] Select Phase 2 %s" % wifiap.phase
                self._locator.phase_method.click.wait()
                self._locator.phase_option(wifiap.phase).click()
            if wifiap.identity:
                assert self._locator.identity.exists, \
                    "Please check the ap and ap_config, \
                    There is no identity widget"
                self._locator.identity.set_text(wifiap.identity)
            if wifiap.ca:
                assert self._locator.ca_method.exists, \
                    "Please check the ap and ap_config, \
                    There is no ca certificate widget"
                print "[INFO] Select CA certificate %s" % wifiap.ca
                self._locator.ca_method.click()
                self._locator.ca_option(wifiap.ca).click()
            if wifiap.usr:
                assert self._locator.usr_method.exists, \
                    "Please check the ap and ap_config, \
                    There is no user certificate widget"
                self._locator.usr_method.click()
                self._locator.usr_option(wifiap.usr).click()
            if wifiap.passwd:
                assert self._locator.password.exists, \
                    "Please check the ap and ap_config, \
                    There is no passwd widget"
                print "DEBUG PASSWD %s" % wifiap.passwd
                self._locator.password.set_text(wifiap.passwd)
            if self._locator.btn_connect.exists:
                self._locator.btn_connect.click()
            else:
                print "[INFO] AP is unsecured"
            time.sleep(1)
        else:
            raise Exception("[ERROR] not supported AP status: %s" % ap_status)
        print "[INFO] Connect AP success"

    def check_status(self, status):
        """Check WiFi is ON or OFF"""
        print "[INFO]: Check airplane mode is %s" % status
        value_dict = {"True": "ON", "False": "OFF"}
        if str(status) in value_dict:
            newstatus = value_dict[str(status)]
        else:
            newstatus = str(status)
        assert self._locator.switch_text.text, newstatus

    def check_avi_network_status(self, status=True):
        """Check available network status"""
        print "[INFO]: Check the available network status is %s(\
            True-Have available network, False-No available network)" % status
        assert str(self._locator.network_list.exists), str(status)

    def __add_ap(self, wifiap):
        """Add a new wifi AP"""
        print "[INFO] Start to add AP: %s" % wifiap.ssid
        self._locator.btn_more_options.click()
        self._locator.btn_add_ap.click()
        print "[INFO] Input ssid: %s" % wifiap.ssid
        self._locator.ssid.set_text(wifiap.ssid)
        if cmp(wifiap.security, WiFiSecurity.SEC_WEP) == 0 or cmp(
                wifiap.security, WiFiSecurity.SEC_WPA_WPA2) == 0:
            self._locator.security.click()
            time.sleep(2)
            print "[INFO] Select security: %s" % wifiap.security
            self._locator.security_option(wifiap.security).click()
            time.sleep(2)
            print "[INFO] Input password: %s" % wifiap.passwd
            self._locator.password.set_text(wifiap.passwd)
            time.sleep(2)
        if cmp(wifiap.security, WiFiSecurity.SEC_802_1x) == 0:
            self._locator.security.click()
            time.sleep(2)
            print "[INFO] Select security: %s" % wifiap.security
            self._locator.security_option(wifiap.security).click()
            time.sleep(2)
            if wifiap.eap:
                print "[INFO] Select EAP method %s" % wifiap.eap
                self._locator.eap_method.click()
                self._locator.eap_option(wifiap.eap).click()
            if wifiap.phase:
                print "[INFO] Select Phase 2 %s" % wifiap.phase
                self._locator.phase_method.click()
                self._locator.phase_option(wifiap.phase).click()
            if wifiap.ca:
                print "[INFO] Select CA certificate %s" % wifiap.ca
                self._locator.ca_method.click()
                self._locator.ca_option(wifiap.ca).click()
            if wifiap.usr:
                self._locator.usr_method.click()
                self._locator.usr_option(wifiap.usr).click()
            if wifiap.identity:
                print "[INFO] Input Identity %s" % wifiap.identity
                self._locator.identity.set_text(wifiap.identity)
            if wifiap.passwd:
                print "[INFO] Input password: %s" % wifiap.passwd
                print "DEBUG PASSWD %s" % wifiap.passwd
                if self.d(scrollable=True).exists:
                    self.d(scrollable=True).scroll.to(
                        resourceId="com.android.settings:id/password")
                self._locator.password.set_text(wifiap.passwd)
            time.sleep(2)
        self._locator.btn_save.click()
        print "[INFO] Click Save button"
        print "[INFO] Add AP success"

    def forget_spec_ap(self, ssid):
        """ Forget specified AP """
        print "[INFO] Forget AP: %s" % ssid
        if self._locator.ap_title(ssid) is None:
            print "[ERROR] Can't find AP: %s" % ssid
            return
        ap_status = self._locator.ap_summary(ssid).info['text']
        self.__forget_ap_old(ssid, ap_status)
        print "[INFO] Forget success"

    def forget_ap(self, wifiap):
        """Forget specified AP"""
        print "[INFO] Forget AP: %s" % wifiap.ssid
        self._locator.btn_more_options.click()
        if self._locator.btn_saved.exists:
            self._locator.btn_saved.click()
            self.__forget_ap(wifiap.ssid)
            self.d.press.back()
        else:
            self.d.press.back()
            print "There is no connect AP"

    def forget_con_ap(self):
        """ Forget connected AP """
        print "[INFO] Forget connected AP"
        con_ap = self._locator.ap_connected()
        if con_ap:
            self.forget_spec_ap(con_ap.info['text'])
        else:
            print "[INFO] No AP connected"

    def forget_all_ap(self):
        """Forget connect AP"""
        self._locator.btn_more_options.click()
        if self._locator.btn_saved.exists:
            self._locator.btn_saved.click()
            number = self._locator.ap_view.count
            while (number != 0):
                self._locator.ap_view.click.wait()
                self._locator.btn_forget.click()
                number = self._locator.ap_view.count
            print "[INFO] Forget AP successfully"
            self.d.press.back()
            time.sleep(3)
        else:
            print "No connect ap"
            time.sleep(3)
            self.d.press.back()

    def check_connected(self, ssid, status=True):
        """Check AP connected status"""
        if status:
            print "[INFO] Check AP is connect"
            timeout = 20
            while timeout > 0:
                if self._locator.ap_connected(ssid) is None:
                    timeout -= 3
                    time.sleep(3)
                else:
                    break
            assert self._locator.ap_connected(ssid) is not None, \
                "[ERROR] %s connect fail" % ssid
        else:
            print "[INFO] Check AP is disconnect"
            timeout = 30
            while timeout > 0:
                ap_status = self._locator.ap_summary(ssid).info['text']
                if APStatus.AProblem in ap_status:
                    break
                else:
                    print "[INFO] ap connecting wait for 5s"
                    timeout -= 5
                    time.sleep(5)
            assert self._locator.ap_connected(ssid) is None

    def check_ap_save(self, wifiap):
        """Check ap status is saved"""
        ap_status = self._locator.ap_summary(wifiap.ssid).info['text']
        assert ap_status, APStatus.Saved

    def check_disconnect(self, wifiap):
        """Check if wifi is not in connected status"""
        print "[INFO]: Check if the Wi-Fi ap %s is disconnect" % wifiap.ssid
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.toBeginning()
        for i in range(7):
            if not self.d(text=wifiap.ssid).exists:
                break
            time.sleep(10)
        if not self.d(text=wifiap.ssid).exists:
            self._locator.btn_more_options.click()
            self._locator.btn_saved.click()
            time.sleep(2)
            value = str(self.d(text=wifiap.ssid).exists)
            self.d.press.back()
        assert value, True

    def __forget_ap_old(self, ssid, ap_status):
        """ Forget AP according to AP's Status """
        if APStatus.NotInRange in ap_status \
            or APStatus.Connected in ap_status \
                or APStatus.AProblem in ap_status \
                or APStatus.Saved in ap_status:
            self._locator.ap_title(ssid).click()
            self._locator.btn_forget.click()
            if APStatus.NotInRange in ap_status:
                assert self._locator.ap_title(
                    ssid) is None, "[ERROR] Forget AP fail"
            else:
                if not self._locator.ap_summary(
                        ssid) is None:
                    ap_status = self._locator.ap_summary(ssid).info['text']
                    assert APStatus.SecuredWith in ap_status or \
                        ap_status.strip() == ''
            # wait ap disappear from ap lists, if forgeted AP is a hidden AP
            time.sleep(10)
        else:
            print "AP current status: %s not support forget" % ap_status

    def __forget_ap(self, ssid):
        """Forget AP according to AP's Status"""
        if self._locator.ap_title(ssid).exists:
            self._locator.ap_title(ssid).click.wait()
            self._locator.btn_forget.click()
            assert self._locator.ap_title(ssid) is None, \
                "[INFO] Forget AP successfully"
        else:
            print "[ERROR] Can't find AP: %s" % ssid

    @staticmethod
    def __wait_wifi_onoff_comp(onoff):
        """Wait Wi-Fi on/off complete"""
        cmd = 'dumpsys wifi | grep Wi-Fi; echo $?'
        loop = 40
        while(loop > 0):
            res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')

            if g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip() == '[O]':
                ret = int(res_list[-1][-1].strip())
            else:
                ret = int(res_list[-1].strip())
            if ret != 0:
                continue
            if onoff is True:
                if "enabled" in ''.join(res_list):
                    print "[INFO] Wifi is enabled."
                    break
                else:
                    print "[INFO] Wifi is enabling."
                    loop -= 1
                    time.sleep(3)
                    continue
            else:
                if "disabled" in ''.join(res_list):
                    print "[INFO] Wifi is disabled."
                    break
                else:
                    loop -= 1
                    time.sleep(3)
                    continue

    def install_usr(self, user):
        """Install P12 certificates from wifi advanced"""
        print "[INFO]: --- Install certificates %s from wifi\
        " % user.name
        self._locator.install_cer.click()
        time.sleep(2)
        if self._locator.open_from.exists:
            self._locator.btn_certification_storage.click.wait()
        time.sleep(1)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=user.name)
        self.d(text=user.name).click.wait()
        self._locator.extract_certificate_password.set_text(user.passwd)
        self._locator.btn_ok.click()
        time.sleep(2)
        self._locator.certificate_name.clear_text()
        self._locator.certificate_name.set_text(user.setname)
        self._locator.credential_usage.click()
        self.d(text=user.type).click()
        time.sleep(1)
        self._locator.btn_ok.click()
        if self._locator.confirm_pin.exists:
            self._locator.pin_entery.set_text(user.pin)
            self._locator.btn_continue.click()
        print "[INFO]: --- Finish to install certificates %s from wifi\
        " % user.name

    def install_ca(self, ca):
        """Install ca certificates from wifi advanced"""
        print "[INFO]: --- Install certificates %s from wifi\
        " % ca.name
        if self._locator.install_cer.exists:
            self._locator.install_cer.click()
        time.sleep(2)
        if self._locator.open_from.exists:
            self._locator.btn_certification_storage.click.wait()
        time.sleep(1)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=ca.name)
        self.d(text=ca.name).click.wait()
        time.sleep(1)
        self._locator.certificate_name.set_text(ca.setname)
        self._locator.credential_usage.click()
        self.d(text=ca.type).click()
        time.sleep(1)
        self._locator.btn_ok.click()
        if self._locator.confirm_pin.exists:
            self._locator.pin_entery.set_text(ca.pin)
            self._locator.btn_continue.click()
        print "[INFO]: --- Finish to install certificates %s from wifi\
        " % ca.name

    def clear_credentials(self):
        """Remove all certificates"""
        print "[INFO]: --- Clear certificates"
        time.sleep(3)
        cerdential = self.d(text="Clear credentials", enabled="false")
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="Clear credentials")
            if self.d(text="Clear credentials", enabled="true").exists:
                self._locator.clear_credenty.click()
                self._locator.btn_ok.click()
                assert cerdential.exists is True

    def launch_security_from_am(self):
        """Launch security Setttings from am"""
        print "[INFO] Launch security Setttings from am"
        cmd = 'am start -a android.settings.SECURITY_SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "[FAILURE] Launch security Setttings fail"
        ret = int(res_list[-1].strip())
        assert ret == 0, msg_fail
        self._locator.screen_lock.wait.exists(timeout=3*1000)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="Screen lock")
        assert self._locator.screen_lock.exists, msg_fail
        print "[INFO] Launch security Setttings success"

    def __advanced(self):
        """
        Go to wifi advanced UI
       """
        self._locator.btn_more_options.click()
        self._locator.btn_advanced.click()

    def set_PIN(self, pin="1234"):
        """Set PIN screen lock"""
        print "[INFO]: --- Set PIN %s" % pin
        time.sleep(3)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="Screen lock")
        if self._locator.pin.exists:
            print "[INFO]: --- PIN has already been set"
        else:
            self._locator.screen_lock.click()
            self._locator.pin.click()
            self._locator.pin_entery.set_text(pin)
            self._locator.btn_continue.click()
            self._locator.pin_entery.set_text(pin)
            self._locator.btn_ok.click()
            self._locator.btn_done.click()
        time.sleep(1)
        assert self._locator.screen_lock.down(text="PIN").exists is True

    def remove_PIN(self, pin="1234"):
        """Remove PIN screen lock"""
        print "[INFO]: --- Remove PIN %s" % pin
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="Screen lock")
        if self._locator.screen_lock.down(text="PIN").exists:
            self._locator.screen_lock.click()
            self._locator.pin_entery.set_text(pin)
            self._locator.btn_continue.click()
            self._locator.btn_none.click()
        else:
            print "[INFO]: --- Screen security is not PIN"
        assert self._locator.screen_lock.down(text="None").exists is True

    def tls_env_setting(self, user, ca):
        """Set the environment for connect AP with tls"""
        self.stop_from_am()
        self.launch_security_from_am()
        self.set_PIN()
        self.clear_credentials()
        self.launch_from_settings()
        self.__advanced()
        self.install_usr(user)
        self.install_ca(ca)

    def tls_env_clear(self):
        """Clear the environment for connect AP with tls"""
        self.launch_security_from_am()
        self.clear_credentials()
        self.remove_PIN()

    @staticmethod
    def __wait_ap_con_comp(timeout):
        """Wait AP connect complete within timeout"""
        cmd = 'dumpsys wifi | grep "mNetworkInfo NetworkInfo"; echo $?'
        loop = timeout * 10
        sleep_interval = 6
        while(loop > 0):
            res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
            ret = int(res_list[-1].strip())
            if ret != 0:
                continue
            return_str = ''.join(res_list)
            if "state: CONNECTED" in return_str:
                _parts = [
                    part for part in return_str.split(',')if "extra:" in part]
                connected_ap = _parts[0].split(':')[1]
                print "[INFO] AP %s is connected." % connected_ap.strip()
                return True, connected_ap
            else:
                print "[INFO] AP is connecting."
                loop -= 1
                time.sleep(sleep_interval)
                continue
        print "[INFO] No AP connected within %d mins." % timeout
        return False, None

    def init_tc_status(self):
        """Init test status, turn off wifi"""
        if (self._locator.onoff_switch.info['text'] == "ON"):
            print "Initialization, current WiFi status is ON, \
            try to turn it off ..."
            self._locator.onoff_switch.click()
            assert self._locator.onoff_switch.info['text'] == "OFF"
            print "Initialization, \
            turn WiFi status from ON to OFF successfully."

    def wifi_restore(self):
        self.restore()

    def restore(self):
        """restore wifi status to ON and connected to the pre-configure AP"""
        self.launch_from_am()
        if self._locator.onoff_switch.info['text'] == "OFF":
            print "Initialization, current WiFi status is OFF, \
            try to turn it ON ..."
            self._locator.onoff_switch.click()
            assert self._locator.onoff_switch.info['text'] == "ON"
            time.sleep(20)
        self.connect_to_ap()

    def __get_ap_available_in_range(self):
        """get available from config"""
        access_points = self.cfg.get("access_points")
        if access_points:
            return map(lambda x: x.strip(), access_points.split(','))
        return None

    def get_wifi_ap_on_device(self, timeout=60):
        return self.get_ap_on_device(timeout)

    def get_ap_on_device(self, timeout=60):
        """Get scan ap"""
        while not self._locator.ap_view.exists:
            print "There is no ap here, wait 5s!"
            time.sleep(5)
            timeout -= 5
            assert timeout > 0, "Scan no ap in 60s"
        if self._locator.ap_list_view.exists:
            import xml.etree.ElementTree as ET
            import tempfile
            lauout_file_path = os.path.join(
                tempfile.gettempdir(), "wifi_layout.xml")
            self.d.dump(lauout_file_path)
            xmltree = ET.parse(lauout_file_path)
            root = xmltree.getroot()
            childs = root.findall('.//node[@resource-id="android:id/title"]')
            if childs:
                wifi_names = map(lambda ele: ele.attrib[
                    'text'].strip(), childs)
                return wifi_names
            return None

    def scan(self):
        """Scan wifi"""
        print "[INFO]: Click scan button to scan Wi-Fi"
        self._locator.btn_more_options.click()
        self._locator.btn_refresh.click()
        time.sleep(2)
        assert self._locator.network_list.count > 1, True

    def wifi_scan_access_point(self):
        """Wifi scan  AP"""
        print "Scan wifi AP"
        self._locator.btn_more_options.click()
        self._locator.btn_advanced.click()
        value = self._locator.btn_scan_switch.info['text']
        if "ON" in value:
            self._locator.btn_scan_switch.click()
            time.sleep(2)
        self.d.press.back()
        #switch on off
        self._locator.onoff_switch.click()
        time.sleep(2)
        self._locator.onoff_switch.click()
        time.sleep(15)
        conf_ponits = self.__get_ap_available_in_range()
        device_ponits = self.get_ap_on_device()
        print "Wifi AP in conf: %s" % conf_ponits
        print "Wifi AP on device: %s" % device_ponits
        assert conf_ponits is not None, "\
        Not find any wifi AP in wifi_setting.conf"
        assert device_ponits is not None, "\
        Not find any wifi AP on device"
        conf_set = set(conf_ponits)
        device_set = set(device_ponits)
        all_find = conf_set.issubset(device_set)
        notfind_points = list(conf_set-device_set)
        assert all_find, "Not find AP on device: %s" % notfind_points
        print "Find wifi AP on device: %s " % conf_ponits

    def auto_scan_on(self):
        """Set auto scan on"""
        self.launch_from_am()
        self._locator.btn_more_options.click()
        self._locator.btn_advanced.click()
        value = self._locator.btn_scan_switch.info['text']
        if "OFF" in value:
            self._locator.btn_scan_switch.click()
            time.sleep(2)
        assert "ON" in self._locator.btn_scan_switch.info['text'], \
            "Open auto scan function fail!"
        self.d.press.back()

    def quit_app(self):
        """Quit app"""
        for _ in range(3):
            self.d.press.back()

    def reconnectAP(self, ap_name, pwd):
        """Reconnect AP"""
        print "wifi name= " + ap_name
        assert self.d(textMatches=ap_name).exists, "ERROR:text not found!"
        self.d(textMatches=ap_name).click.wait()
        if self._locator.btn_connected.exists and \
                self._locator.btn_forget.exists:
            self._locator.btn_forget.click.wait()
            time.sleep(3)
            self.d(textMatches=ap_name).click.wait()
        if self._locator.password.exists:
            self._locator.password.set_text(pwd)
            time.sleep(2)
        if self._locator.btn_connect.exists:
            self._locator.btn_connect.click.wait()
        time.sleep(20)
        assert self._locator.btn_connected.exists, "ERROR:text not found!"

    def wifi_connect(self):
        self.connect()

    def connect(self):
        """A note function for wifi connect"""
        self.launch_from_am()
        time.sleep(2)
        self._locator.wifi_settings.click.wait()
        self.reconnectAP(self.cfg.get("wifi_ap1_name"), "")
        time.sleep(2)
        self.reconnectAP(self.cfg.get(
            "wifi_ap2_name"), self.cfg.get("wifi_ap2_pwd"))
        self.quit_app()

    def unlock(self, raiseError=False):
        self.d.wakeup()
        lockview = self.d(
            resourceId="com.android.keyguard:id/keyguard_selector_view_frame")
        if lockview.exists:
            lockview.swipe.right()
        if self.d(resourceId="com.android.systemui:id/lock_icon").exists:
            w = self.d.info[u'displayWidth']
            h = self.d.info[u'displayHeight']
            self.d.swipe(w/2, h, w/2, 0)


class AirPlaneModeImpl:
    """
    Implements Airplane mode Setting UI actions.

    """

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            """Init environment"""
            self.d = device

        @property
        def setting_more(self):
            """ Settings UI more """
            return self.d(textStartsWith="More")

        @property
        def btn_airplane(self):
            """ Airplane button"""
            return self.d(text="Airplane mode")

        @property
        def onoff_switch(self):
            """ UI switch turn on/off airplane """
            return self.d(text="Airplane mode").right(
                className="android.widget.Switch")
    #------------------------ end of class Locator ----------------------

    def __init__(self, cfg):
        """Init environment"""
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = AirPlaneModeImpl.Locator(self.d)

    def launch_from_settings(self):
        """Launch Settings and enter into Airplane mode Setting UI"""
        print "[INFO]: Launch airplane mode from Settings"
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc("Settings")
        self._locator.setting_more.click.wait()
        assert self._locator.btn_airplane.exists, True

    def init_status(self, status):
        """Turn on/off Airplane mode according to the expected status"""
        value_dict = {"True": "ON", "False": "OFF"}
        print "[INFO]: Initialize airplane mode to %s" % status
        if str(status) in value_dict:
            newstatus = value_dict[str(status)]
        else:
            newstatus = str(status)
        if str(self._locator.onoff_switch.text) != newstatus:
            self._locator.onoff_switch.click()
        assert str(self._locator.onoff_switch.text), newstatus

    def on(self):
        """Turn on Airplane mode"""
        print "[INFO]: Turn on airplane mode"
        self._locator.onoff_switch.click()
        time.sleep(2)
        assert self._locator.onoff_switch.text, "ON"

    def off(self):
        """Turn off Airplane mode"""
        print "[INFO]: Turn off airplane mode"
        self._locator.onoff_switch.click()
        time.sleep(2)
        assert self._locator.onoff_switch.text, "OFF"

    def check_status(self, status):
        """Check Airplane mode is ON or OFF"""
        print "[INFO]: Check airplane mode is %s" % status
        value_dict = {"True": "ON", "False": "OFF"}
        if str(status) in value_dict:
            newstatus = value_dict[str(status)]
        else:
            newstatus = str(status)
        assert self._locator.onoff_switch.text, newstatus
