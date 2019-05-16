#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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
import re


class APStatus(object):
    """
    constant class for AP status
    """

    Connected = 'Connected'
    SecuredWith = "Secured with"
    NotInRange = "Not in range"
    Saved = "Saved"
    NONE = ""
    WPSAvailable = "WPS available"
    Connecting = "Connecting"
    Authenticating = "Authenticating"
    AProblem = "Authentication problem"
    Disable = "Disabled"
    Disconnected = "Disconnected"

    NotConnected = [SecuredWith, Saved, NONE, WPSAvailable, \
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
        self.name = cfg.get("name")
        self.passwd = cfg.get("passwd")
        self.ssid = cfg.get("ssid")
        self.security = cfg.get("security")
        self.identity = cfg.get("identity")
        self.eap = cfg.get("eap")
        self.phase = cfg.get("phase")
        self.usr = cfg.get("usr")

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
            self.d = device

        @property
        def wifi_settings(self):
            """ UI 'Wi-Fi' display on Settings, \
            click it enter Wi-Fi settings """
            return self.d(textMatches="Wi.*Fi")

        @property
        def settings_title(self):
            """ UI setting_title"""
            return self.d(resourceId="android:id/action_bar_title", \
                text="Settings")

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
            """ UI 'Connected' appear when AP connected success """
            try:
                if self.ap_list_view.child_by_text(\
                    "Connected", allow_scroll_search=True).exists:
                    return self.d(\
                        text="Connected", resourceId="android:id/summary")
            except:
                return None
            return None

        def ap_title(self, ssid):
            """ UI label which display AP ssid """
            try:
                if self.ap_list_view.child_by_text(\
                    ssid, allow_scroll_search=True, \
                    resourceId="android:id/title").exists:
                    return self.d(text=ssid, resourceId="android:id/title")
            except:
                return None
            return None

        def ap_summary(self, ssid):
            """ UI label which display AP \
            summary(Saved, Connected, Secured with ...) """
            try:
                if self.ap_list_view.child_by_text(\
                    ssid, allow_scroll_search=True, \
                    resourceId="android:id/title").exists:
                    if self.d(text=ssid, \
                        resourceId="android:id/title").\
                    sibling(resourceId="android:id/summary").exists:
                        return self.d(text=ssid, \
                            resourceId="android:id/title").\
                        sibling(resourceId="android:id/summary")
            except:
                return None
            return None

        def ap_connected(self, ssid=None):
            """ UI connected AP """
            if self.label_connected is None:
                return None
            if not ssid:
                return self.label_connected.up(resourceId="android:id/title")
            else:
                if self.label_connected.up(\
                    text=ssid, resourceId="android:id/title") is None:
                    return None
                if self.label_connected.up(\
                    text=ssid, resourceId="android:id/title").exists:
                    return self.label_connected.up(\
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
            return self.d(textContains="Scanning").right(\
            resourceId="android:id/switchWidget")

        @property
        def btn_advanced(self):
            """ UI button advanced """
            return self.d(textMatches="Advanced")

        @property
        def btn_add_ap(self):
            """ UI button add ap """
            return self.d(text="Add network")

        @property
        def ssid(self):
            """ UI input field for 'Network SSID' """
            return self.d(resourceId="com.android.settings:id/ssid")

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
            return self.d(textMatches=cacert, \
                className="android.widget.CheckedTextView")

        @property
        def usr_method(self):
            """ UI dropdown for 'User certificate' """
            return self.d(resourceId="com.android.settings:id/user_cert")

        def usr_option(self, usrcert):
            """ UI option of 'User certificate' dropdown """
            return self.d(textMatches=usrcert, \
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
            return self.d(resourceId="android:id/alertTitle", \
                text="Extract certificate")

        @property
        def extract_certificate_password(self):
            """ UI input field for certificate password """
            return self.d(\
                resourceId="com.android.certinstaller:id/credential_password")

        @property
        def certificate_name(self):
            """ UI input field for certificate name """
            return self.d(\
                resourceId="com.android.certinstaller:id/credential_name")

        @property
        def credential_usage(self):
            """ UI credential usage """
            return self.d(\
                resourceId="com.android.certinstaller:id/credential_usage")

        @property
        def btn_credential_use(self):
            """ UI credential use """
            return self.d(resourceId="android:id/text1", \
                textMatches="Wi.*Fi")

        @property
        def btn_ok(self):
            """ UI button OK """
            return self.d(text="OK")

        @property
        def btn_certification_title(self):
            """ UI button certification title """
            return self.d( resourceId="android:id/action_bar_title")

        @property
        def btn_certification_openfrom(self):
            """ UI button Open from """
            return self.d(text="Open from", \
                resourceId="android:id/action_bar_title")

        @property
        def btn_certification_storage(self):
            """ UI button storage """
            return self.d(text="Internal storage", \
                resourceId="android:id/title")

        @property
        def btn_certification_intall(self):
            """ UI button download folder base dir """
            return self.d(resourceId="android:id/up")

        @property
        def btn_download_folder(self):
            """ UI button download folder """
            return self.d(resourceId="com.android.documentsui:id/list").\
                child_by_text("Download", allow_scroll_search=True)

        @property
        def btn_download(self):
            """ UI button download folder """
            return self.d(text="Download")

        @property
        def btn_attention(self):
            """ UI button download folder """
            return self.d(text="Attention", \
                resourceId="android:id/title")
    #------------------------ end of class Locator ----------------------

    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = WifiSettingImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_from_am(self):
        """ Launch Wi-Fi Setttings from am """
        print "[INFO] Launch Wi-Fi Setttings from am"
        cmd = 'am start -a android.settings.WIFI_SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        msg_fail = "[FAILURE] Launch Wi-Fi Setttings fail"
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
        """
            Stop Wi-Fi Setttings from am
        """
        print "[INFO] Stop Wi-Fi Setttings from am"
        g_common_obj.stop_app_am(WifiSettingImpl.setting_pkg_name)

    def turn_on_wifi(self, waitconnect=False):
        """
            Turn on WiFi
        """
        print "[INFO] Turn on Wi-Fi."
        if self._locator.onoff_switch.info['text'] == "ON":
            print "[INFO] Wi-Fi is already ON"
            return True
        else:
            self._locator.onoff_switch.click()
            self.__wait_wifi_onoff_comp(True)
            assert self._locator.onoff_switch.info['text'] == "ON", \
            "Turn on Wi-Fi fail."
        if waitconnect is True:
            # wait any one AP connected
            ret, _ = self.__wait_ap_con_comp(2)
            if ret is False:
                pass
            else:
                assert self._locator.label_connected is not None, \
                "No AP connected."
        print "[INFO] Turn on Wi-Fi success."
        return True

    def turn_off_wifi(self):
        """
            Turn off wifi
        """
        print "[INFO] Turn off Wi-Fi."
        if self._locator.onoff_switch.info['text'] == "OFF":
            return
        else:
            self._locator.onoff_switch.click()
            self.__wait_wifi_onoff_comp(False)
            assert self._locator.onoff_switch.info['text'] == "OFF", \
            "Turn off Wi-Fi fail."
        print "[INFO] Turn off Wi-Fi success."

    def connect_to_ap(self):
        """
        Connect to default ap
        """
        #self.connect_ap(WifiAP(self.cfg), False, False)
        self.connect_ap(WifiAP(self.cfg), False)

    def connect_ap(self, wifiap, forget=True, ignore_exist=True):
        """
        @summary: Connect to ap
        @param wifiap:
        @param forget: True = forget connected AP firstly
        @param ignore_exist: True = always add AP, no matter the ap exists or not.
        support Security: [ None, WEP, WPA/WPA2 PSK ]
        """
        print "[INFO] Connect to ap: %s" % wifiap.ssid
        ap_status = self.get_ap_status(wifiap)
        if APStatus.AProblem in ap_status or \
        APStatus.Disable in ap_status or \
        APStatus.Disconnected in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_forget.click()
            print "[INFO] Sleep 15s to refresh wifi list"
            time.sleep(15)

        # forget conneted ap, if there is
        if forget:
            self.forget_con_ap()
        # add ap directly
        #if wifiap.ca:
            #self.__tls_certificate()
        if ignore_exist:
            self.__add_ap(wifiap)
            self.check_connected(wifiap.ssid)
        else:
            # if ap has exists
            if self._locator.ap_title(wifiap.ssid) is not None:
                self.__con_ap(wifiap)
                self.check_connected(wifiap.ssid)
            else:
                self.__add_ap(wifiap)
                self.check_connected(wifiap.ssid)

    def connect_ap_without_check(self, wifiap, forget=True):
        """ Connect to ap without checking whether the ap is connected
        1. forget = True, forget connected AP firstly
        2. if the ap want to connected not in ap list, add the ap,
           if has exists skip add it, just connect it
        3. support Security: [ None, WEP, WPA/WPA2 PSK ]
        """
        print "[INFO] Connect to ap: %s" % wifiap.ssid
        ap_status = self.get_ap_status(wifiap)
        if APStatus.AProblem in ap_status or \
        APStatus.Disable in ap_status or \
        APStatus.Disconnected in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_forget.click()
            print "[INFO] Sleep 15s to refresh wifi list"
            time.sleep(15)

        # forget conneted ap, if there is
        if forget:
            self.forget_con_ap()
        # if ap has exists
        #if wifiap.ca:
            #self.__tls_certificate()
        if self._locator.ap_title(wifiap.ssid) is not None:
            self.__con_ap(wifiap)
        else:
            self.__add_ap(wifiap)

    def get_ap_status(self, wifiap):
        """
        @summary: get ap connect status
        """
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
                    if self._locator.btn_connect.exists:
                        self._locator.btn_connect.click()
                    else:
                        self.d.press.back()
                print "[INFO] ap connecting, wait 5s!"
                time.sleep(5)
                timeout -= 5
                assert timeout > 0, \
                "Error: the ap is always connecting!"
                ap_status = self._locator.ap_summary(wifiap.ssid).info['text']
        return ap_status

    def __con_ap(self, wifiap):
        """ Connect to AP according to AP Status """
        ap_status = self.get_ap_status(wifiap)
        print "[INFO] The %s status: %s" % (wifiap.ssid, ap_status)
        if APStatus.Connected in ap_status:
            print "[INFO] AP is already connected"
            return
        elif APStatus.Saved in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            self._locator.btn_connect.click()
        elif APStatus.SecuredWith in ap_status:
            self._locator.ap_title(wifiap.ssid).click()
            if wifiap.passwd:
                assert self._locator.password.exists, \
                "Please check the ap and ap_config, There is no passwd widget"
                print "DEBUG PASSWD %s" % wifiap.passwd
                self._locator.password.set_text(wifiap.passwd)
            if self._locator.eap_method.exists and wifiap.eap:
                print "[INFO] Select eap"
                self._locator.eap_method.click()
                if self._locator.eap_option(wifiap.eap).exists:
                    self._locator.eap_option(wifiap.eap).click()
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
            time.sleep(1)
            self._locator.btn_connect.click()
        elif APStatus.NotInRange in ap_status:
            print "[INFO] AP is %s" % ap_status
            self.__forget_ap(wifiap.ssid, ap_status)
            self.__add_ap(wifiap)
        elif APStatus.NONE == ap_status.strip() or \
        APStatus.WPSAvailable == ap_status.strip():
            time.sleep(5)
            g_common_obj.restart_server()
            self._locator.ap_title(wifiap.ssid).click()
            if self._locator.password.exists:
                self._locator.password.set_text(wifiap.passwd)
                self._locator.btn_connect.click()
            else:
                print "[INFO] AP is unsecured"

        else:
            raise Exception("[ERROR] not supported AP status: %s" % ap_status)
        print "[INFO] Connect AP success"

    def __add_ap(self, wifiap):
        """ Add a new wifi AP """
        print "[INFO] Start to add AP: %s" % wifiap.ssid
        self._locator.btn_more_options.click()
        self._locator.btn_add_ap.click()
        print "[INFO] Input ssid: %s" % wifiap.ssid
        self._locator.ssid.set_text(wifiap.ssid)
        if cmp(wifiap.security, WiFiSecurity.SEC_WEP) == 0 or \
        cmp(wifiap.security, WiFiSecurity.SEC_WPA_WPA2) == 0:
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
        self.__forget_ap(ssid, ap_status)
        print "[INFO] Forget success"

    def forget_con_ap(self):
        """ Forget connected AP """
        print "[INFO] Forget connected AP"
        con_ap = self._locator.ap_connected()
        if con_ap:
            self.forget_spec_ap(con_ap.info['text'])
        else:
            print "[INFO] No AP connected"

    def check_connected(self, ssid):
        """ Check AP connected """
        timeout = 60
        while timeout > 0:
            if self._locator.ap_connected(ssid) is None:
                timeout -= 3
                time.sleep(3)
            else:
                break
        assert self._locator.ap_connected(ssid) is not None, \
        "[ERROR] %s connect fail" % ssid

    def __forget_ap(self, ssid, ap_status):
        """ Forget AP according to AP's Status """
        if APStatus.NotInRange in ap_status \
        or APStatus.Connected in ap_status \
        or APStatus.AProblem in ap_status:
            self._locator.ap_title(ssid).click()
            self._locator.btn_forget.click()
            if APStatus.NotInRange in ap_status:
                assert self._locator.ap_title(ssid) is None, \
                "[ERROR] Forget AP fail"
            else:
                if not self._locator.ap_summary(ssid) == None:
                    ap_status = self._locator.ap_summary(ssid).info['text']
                    assert APStatus.SecuredWith in ap_status or \
                    ap_status.strip() == ''
            # wait ap disappear from ap lists, if forgeted AP is a hidden AP
            time.sleep(10)
        else:
            print "AP current status: %s not support forget" % ap_status

    @staticmethod
    def __wait_wifi_onoff_comp(onoff):
        """ Wait Wi-Fi on/off complete
        @param onoff: (boolean)
            True = expect Wi-Fi on, False = expect Wi-Fi off
        """
        cmd = 'dumpsys wifi | grep Wi-Fi; echo $?'
        loop = 40
        while(loop > 0):
            # res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
            r_enter = re.compile("\r|\n")
            res_list = r_enter.split(g_common_obj.adb_cmd_capture_msg(cmd))
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

    @staticmethod
    def __wait_ap_con_comp(timeout):
        """ Wait AP connect complete within timeout
        @param timeout: (int) minutes to wait any AP connected
        @return: (True, connected ap ssid) or (False, None)
        """
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
                _parts = [part for part in return_str.split(',') \
                if "extra:" in part]
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
        """
            Init test status, turn off wifi
        """
        if (self._locator.onoff_switch.info['text'] == "ON"):
            print "Initialization, current WiFi status is ON, \
            try to turn it off ..."
            self._locator.onoff_switch.click()
            assert self._locator.onoff_switch.info['text'] == "OFF"
            print "Initialization, \
            turn WiFi status from ON to OFF successfully."

    def wifi_restore(self):
        """ restore wifi status to ON and connected to the pre-configure AP """
        self.launch_from_am()
        if self._locator.onoff_switch.info['text'] == "OFF":
            print "Initialization, current WiFi status is OFF, \
            try to turn it ON ..."
            self._locator.onoff_switch.click()
            assert self._locator.onoff_switch.info['text'] == "ON"
            time.sleep(20)
        self.connect_to_ap()

    def __get_wifi_ap_available_in_range(self):
        """
            get available from config
        """
        access_points = self.cfg.get("access_points")
        if access_points:
            return map(lambda x:x.strip(), access_points.split(','))
        return None

    def get_wifi_ap_on_device(self, timeout=60):
        """
            get scan ap
        """
        while not self._locator.ap_view.exists:
            print "There is no ap here, wait 5s!"
            time.sleep(5)
            timeout -= 5
            assert timeout > 0, "Scan no ap in 60s"
        if self._locator.ap_list_view.exists:
            import xml.etree.ElementTree as ET
            import tempfile
            lauout_file_path = os.path.join(tempfile.gettempdir(), \
                "wifi_layout.xml")
            self.d.dump(lauout_file_path)
            xmltree = ET.parse(lauout_file_path)
            root = xmltree.getroot()
            childs = root.findall('.//node[@resource-id="android:id/title"]')
            if childs:
                wifi_names = map(lambda ele: ele.attrib['text'].strip(), childs)
                return wifi_names
            return None

    def wifi_scan_access_point(self):
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

        conf_ponits = self.__get_wifi_ap_available_in_range()
        device_ponits = self.get_wifi_ap_on_device()

        print "Wifi AP in conf: %s" % conf_ponits
        print "Wifi AP on device: %s" % device_ponits
        assert conf_ponits != None, "Not find any wifi AP in wifi_setting.conf"
        assert device_ponits != None, "Not find any wifi AP on device"

        conf_set = set(conf_ponits)
        device_set = set(device_ponits)
        all_find = conf_set.issubset(device_set)
        notfind_points = list(conf_set-device_set)
        assert all_find, "Not find AP on device: %s" % notfind_points
        print "Find wifi AP on device: %s " % conf_ponits

    def auto_scan_on(self):
        """ Set auto scan on """
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
        for _ in range(3):
            self.d.press.back()

    def reconnectAP(self, ap_name,pwd):
        print "wifi name= " + ap_name
        assert self.d(textMatches = ap_name).exists,"ERROR:text not found!"
        self.d(textMatches = ap_name).click.wait()
        if self._locator.btn_connected.exists and self._locator.btn_forget.exists:
            self._locator.btn_forget.click.wait()
            time.sleep(3)
            self.d(textMatches = ap_name).click.wait()
        if self._locator.password.exists:
            self._locator.password.set_text(pwd)
            time.sleep(2)
        if self._locator.btn_connect.exists:
            self._locator.btn_connect.click.wait()
        time.sleep(20)
        assert self._locator.btn_connected.exists,"ERROR:text not found!"

    def wifi_connect(self):
        """
        This test used to test create a note function.
        The test case spec is following:
        1. Launch the "setting" and connect a wifi.
        2. verify that connect a ap success.
        """
        self.launch_from_am()
        time.sleep(2)
        self.d(textMatches="Wi.*Fi").click.wait()
        self.reconnectAP(self.cfg.get("wifi_ap1_name"), "")
        time.sleep(2)
        self.reconnectAP(self.cfg.get("wifi_ap2_name"), \
                       self.cfg.get("wifi_ap2_pwd"))
        self.quit_app()
