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

"""
@summary: Class for VPN settings
@since: 07/10/2014
@author: yongzoux (yongx.zou@intel.com)
"""

import time
from testlib.util.common import g_common_obj
from testlib.screen.notifypanel_impl import NotifyPanelImpl, VPNInNofifyPanel

class VPNStatus(object):
    ''' Constant class for VPN status '''


    Connected = "Connected"
    Disconnected = "Disconnected"
    PPTP_VPN = "PPTP VPN"
    L2TP_IPSec_VPN = "L2TP/IPSec VPN"

class VPNType(object):
    ''' Constant class for VPN type '''


    PPTP = "PPTP"
    L2TP_PSK = "L2TP_PSK"
    L2TP_PSK_PRE = "L2TP_PSK_PRE"
    L2TP_RSA = "L2TP_RSA"
    IPSec_X_PSK = "IPSec_X_PSK"
    IPSec_X_RSA = "IPSec_X_RSA"
    IPSec_H_RSA = "IPSec_H_RSA"

    __vpntypes_dic = {
        PPTP : "PPTP",
        L2TP_PSK : "L2TP/IPSec PSK",
        L2TP_PSK_PRE : "L2TP/IPSec VPN with pre-shared keys",
        L2TP_RSA : "L2TP/IPSec RSA",
        IPSec_X_PSK : "IPSec Xauth PSK",
        IPSec_X_RSA : "IPSec Xauth RSA",
        IPSec_H_RSA : "IPSec Hybrid RSA"
    }

    @staticmethod
    def get_vpntype(name):
        """
        get vpn type
        """
        if VPNType.__vpntypes_dic.has_key(name):
            return VPNType.__vpntypes_dic[name]
        else:
            raise Exception("No such VPN type: %s" % name)

class VPNProfile(object):
    """
    Data Model for vpn profile
    include fields: name, type, server address, username, password and so on
    different VPN type with different fields
    """

    def __init__(self, dic):
        self._dic = dic

    def __getattr__(self, name):
        if name == "vpntype":
            return VPNType.get_vpntype(self._dic[name])
        return self._dic[name].strip()

class EditVPNProfile(object):
    ''' Class for manipulate the pop-up 'Add/Edit VPN Profile' UI
    '''


    def __init__(self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    @property
    def _name(self):
        """ UI textfield VPN name """
        return self.d(resourceId="com.android.settings:id/name")

    @property
    def _type(self):
        """ UI textfield VPN type """
        return self.d(resourceId="com.android.settings:id/type")

    def _type_option(self, option):
        """ UI type option in dropdown list """
        return self.d(className="android.widget.CheckedTextView", text=option)

    @property
    def _server_address(self):
        """ UI textfield Server address """
        return self.d(resourceId="com.android.settings:id/server")

    @property
    def _mppe(self):
        """ UI checkbox PPP encryption (MPPE)"""
        return self.d(resourceId="com.android.settings:id/mppe")

    @property
    def _l2tp_secret(self):
        """ UI textfield L2TP secret """
        return self.d(resourceId="com.android.settings:id/l2tp_secret")

    @property
    def _ipsec_identifier(self):
        """ UI textfield IPsec identifier """
        return self.d(resourceId="com.android.settings:id/ipsec_identifier")

    @property
    def _ipsec_preshared_key(self):
        """ UI textfield IPsec pre-shared key """
        return self.d(resourceId="com.android.settings:id/ipsec_secret")

    @property
    def _show_adv_opt(self):
        """ UI checkbox Show advanced options """
        return self.d(resourceId="com.android.settings:id/show_options")

    @property
    def _btn_cancel(self):
        """ UI button Cancel """
        return self.d(text="Cancel")

    @property
    def _btn_save(self):
        """ UI button Save """
        return self.d(text="Save")

    def add_vpn(self, vpnprofile):
        """ Add a VPN """
        self._name.set_text(vpnprofile.name)
        self._type.click()
        self._type_option(vpnprofile.vpntype).click()
        self._server_address.set_text(vpnprofile.server)
        time.sleep(2)
        self.__get_vpn_add_func(vpnprofile.vpntype)(vpnprofile)
        self._btn_save.click()

    def __get_vpn_add_func(self, vpntype):
        """
        get vpn and add function
        """
        addvpn_func = {
            VPNType.get_vpntype(VPNType.PPTP) : self.__add_pptp_vpn,
            VPNType.get_vpntype(VPNType.L2TP_PSK): self.__add_l2tp_ipsec_psk_vpn
        }
        return addvpn_func.get(vpntype)

    def __add_pptp_vpn(self, vpnprofile):
        """ Add PPTP VPN """
        pass

    def __add_l2tp_ipsec_psk_vpn(self, vpnprofile):
        """ Add L2TP/IPsec PSK VPN """
        self._l2tp_secret.set_text(vpnprofile.l2tp_secret)
        time.sleep(2)
        self._ipsec_identifier.set_text(vpnprofile.ipsec_identifier)
        time.sleep(2)
        self._ipsec_preshared_key.set_text(vpnprofile.ipsec_preshared_key)
        time.sleep(2)

class ConnectToVPN(object):
    ''' Class for manipulate the pop-up 'Connect To VPN' UI
    '''

    def __init__(self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    @property
    def _username(self):
        """ UI textfield Username """
        return self.d(resourceId="com.android.settings:id/username")

    @property
    def _password(self):
        """ UI textfield Password """
        return self.d(resourceId="com.android.settings:id/password")

    @property
    def _save_account_infor(self):
        """ UI checkbox Save account information """
        return self.d(resourceId="com.android.settings:id/save_login")

    @property
    def _btn_cancel(self):
        """ UI button Cancel """
        return self.d(text="Cancel")

    @property
    def _btn_connect(self):
        """ UI button Connect """
        return self.d(text="Connect")

    def connect_to(self, vpnprofile):
        """
        @summary: Fill VPN account information, then click Connect
        @param vpnprofile: instance of VPNProfile
        @param saveinfo: True = check 'Save Account information', False = not check
        """
        self._username.set_text(vpnprofile.user)
        time.sleep(2)
        self._password.set_text(vpnprofile.passwd)
        self._btn_connect.click()

class ScreenLockType(object):
    ''' class for constant value of screen lock type '''
    NONE = "None"
    Swipe = "Swipe"
    FaceUnlock = "Face Unlock"
    Pattern = "Pattern"
    PIN = "PIN"
    Password = "Password"

class ScreenLockImpl(object):
    ''' class implement to manipulate 'Settings -> Display -> Screen lock' '''


    def __init__(self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    def __locktype(self, locktype):
        """ UI lock type 
        @param locktype: one constant value in ScreenLockType
        """
        return self.d(text=locktype)

    @property
    def __pin_password(self):
        """ UI text filed for PIN password """
        return self.d(resourceId="com.android.settings:id/password_entry")

    @property
    def __no_thanks(self):
        """ UI button no require further pin encrypt """
        return self.d(text="No thanks")

    @property
    def __btn_continue(self):
        """ UI button Continue """
        return self.d(text="Continue")

    @property
    def __btn_ok(self):
        """ UI button OK """
        return self.d(text="OK")

    @property
    def __btn_done(self):
        """ UI button Done """
        return self.d(resourceId="com.android.settings:id/next_button")

    @property
    def __lbl_choose_pin(self):
        """ UI label 'Confirm your PIN' """
        return self.d(text="Choose your PIN")

    @property
    def __lbl_confirm_pin(self):
        """ UI label 'Confirm your PIN' """
        return self.d(text="Confirm your PIN")

    @property
    def __security_settings_list(self):
        """ UI list of Security Settings """
        return self.d(resourceId="android:id/list")

    @property
    def __screen_lock_entry(self):
        """ UI entry for enter Screen lock under Settings -> Security """
        try:
            if self.__security_settings_list.child_by_text("Screen lock", \
                allow_scroll_search=True).exists:
                return self.d(text="Screen lock")
        # if can't find the UI object
        except:
            return None

    @property
    def __current_lock_type(self):
        """ UI label display current Screen lock type """
        return self.__screen_lock_entry.sibling(resourceId="android:id/summary")

    @property
    def __clear_credentials(self):
        """ UI item of Security 'Clear credentials' """
        try:
            if self.__security_settings_list.\
            child_by_text("Clear credentials", \
                allow_scroll_search=True).exists:
                return self.d(text="Clear credentials")
        # if can't find the UI object
        except:
            return None

    @staticmethod
    def launch_from_am():
        ''' Launch Screen lock from adb am '''
        print "[INFO] Launch Screen lock from adb am"
        cmd = 'am start -S -n \
        com.android.settings/com.android.settings.ChooseLockGeneric$ChooseLockGenericFragment \
        -a android.intent.action.MAIN; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1])
        assert ret == 0, "[ERROR] Launch Screen lock fail"
        print "[INFO] Launch success"

    @staticmethod
    def launch_securitysettings_from_am():
        ''' Launch Security Settings from adb am '''
        print "[INFO] Launch Security Settings from adb am"
        cmd = 'am start -S -n com.android.settings/.SecuritySettings; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1])
        assert ret == 0, "[ERROR] Launch Security Settings fail"
        print "[INFO] Launch success"

    def __enter_screen_lock(self):
        """ Enter Screen lock settings from Security
        please enter Settings-> Security UI before call it
        """
        self.__screen_lock_entry.click()

    def clear_credentials(self):
        """ Clear credentials """
        if self.__clear_credentials is not None:
            if self.__clear_credentials.click() is True:
                self.__btn_ok.click()

    def cancel_pin(self, pinpassword):
        """ Cancel PIN
        @param pinpassword: (string) the password of PIN
        """
        print "[INFO] Cancel Screen lock from PIN to None"
        self.launch_securitysettings_from_am()
        pintype = self.__current_lock_type.info['text']
        if cmp(pintype, ScreenLockType.PIN) != 0:
            print "[INFO] Current Screen lock type is not PIN"
            return
        # clear credentials
        self.clear_credentials()
        self.__enter_screen_lock()
        time.sleep(1)
        assert self.__lbl_confirm_pin.exists
        # input pin password
        self.__pin_password.set_text(pinpassword)
        # click Continue
        time.sleep(1)
        self.__btn_continue.click()
        time.sleep(1)
        assert self.__locktype(ScreenLockType.NONE).exists
        # click lock type 'None'
        self.__locktype(ScreenLockType.NONE).click()
        time.sleep(1)
        pintype = self.__current_lock_type.info['text']
        assert cmp(pintype, ScreenLockType.NONE) == 0
        print "[INFO] Cancel success"

    def setup_pin(self, pinpassword):
        """ Setup Screen lock to PIN
        @param pinpassword: (string) the password of PIN
        """
        print "[INFO] Setup Screen lock to PIN"
        self.launch_securitysettings_from_am()
        pintype = self.__current_lock_type.info['text']
        if cmp(pintype, ScreenLockType.PIN) == 0:
            print "[INFO] Screen lock already is PIN"
            return
        self.__enter_screen_lock()
        self.__locktype(ScreenLockType.PIN).click()
        time.sleep(5)
        if self.__no_thanks.exists:
            self.__no_thanks.click()
            if not self.__no_thanks.checked:
                self.__no_thanks.click()
            assert self.__no_thanks.checked
            self.__btn_continue.click()
            time.sleep(2)
        assert self.__lbl_choose_pin.exists
        # input pin password first time
        self.__pin_password.set_text(pinpassword)
        time.sleep(1)
        self.__btn_continue.click()
        time.sleep(1)
        assert self.__lbl_confirm_pin.exists
        # input pin password second time
        self.__pin_password.set_text(pinpassword)
        time.sleep(1)
        self.__btn_ok.click()
        time.sleep(1)
        self.__btn_done.click()
        time.sleep(1)
        pintype = self.__current_lock_type.info['text']
        assert cmp(pintype, ScreenLockType.PIN) == 0
        print "[INFO] Setup success"

    def setup_swipe(self):
        """
        Setup Screen lock to swipe
        """
        print "[INFO] Setup Screen lock to swipe"
        self.launch_securitysettings_from_am()
        locktype = self.__current_lock_type.info['text']
        if cmp(locktype, ScreenLockType.Swipe) == 0:
            print "[INFO] Screen lock already is swipe"
            return
        # clear credentials
        self.clear_credentials()
        self.__enter_screen_lock()
        time.sleep(1)
        self.__locktype(ScreenLockType.Swipe).click()
        time.sleep(5)
        locktype = self.__current_lock_type.info['text']
        assert cmp(locktype, ScreenLockType.Swipe) == 0
        print "[INFO] Setup success"

    def set_screenlock_none(self):
        """
        Setup Screen lock to None
        """
        print "[INFO] Setup Screen lock to None"
        self.launch_securitysettings_from_am()
        locktype = self.__current_lock_type.info['text']
        if cmp(locktype, ScreenLockType.NONE) == 0:
            print "[INFO] Screen lock already is None"
            return
        # clear credentials
        self.clear_credentials()
        self.__enter_screen_lock()
        time.sleep(1)
        self.__locktype(ScreenLockType.NONE).click()
        time.sleep(5)
        locktype = self.__current_lock_type.info['text']
        assert cmp(locktype, ScreenLockType.NONE) == 0
        print "[INFO] Setup success"

class VPNSettingsImpl(object):
    ''' Class for manipulate on 'VPN Settings' UI
    '''

    def __init__(self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._editprofile = EditVPNProfile(self.d)
        self._contovpn = ConnectToVPN(self.d)
        self._vpnprofile = None

    @property
    def _vpn_list(self):
        """
            vpn list
        """
        return self.d(resourceId="android:id/list")

    def _vpn_item(self, vpn_name):
        """
            vpn item
        """
        try:
            if self._vpn_list.child_by_text(vpn_name, \
                allow_scroll_search=True, \
                resourceId="android:id/title").exists:
                return self.d(text=vpn_name)
        # if can't find the UI object
        except:
            return None

    def _vpn_status(self, vpn_name):
        """ UI vpn status displayed under vpn name, eg: Connected/Disconnect
        Please call _vpn_item() firstly to find the vpn whose name is vpn_name
        """
        return self.d(text=vpn_name).down(resourceId="android:id/summary")

    @property
    def _label_connected(self):
        """
            label Connected
        """
        return self.d(text="Connected")

    def _connected_vpn(self, vpn_name):
        """
            connected vpn
        """
        if self._label_connected.exists:
            return self._label_connected.up(text=vpn_name)
        else:
            return None

    @property
    def _always_on_vpn(self):
        """
            always on vpn
        """
        return self.d(text="Always-on VPN")

    @property
    def _more_options(self):
        """
            btn more options
        """
        return self.d(description="More options")

    @property
    def _btn_add_profile(self):
        """
            btn add profile
        """
        return self.d(description="Add VPN profile")

    @property
    def _btn_edit_profile(self):
        """
            btn edit profile
        """
        return self.d(text="Edit profile")

    @property
    def _btn_del_profile(self):
        """
            btn del profile
        """
        return self.d(text="Delete profile")

    @staticmethod
    def launch_from_am():
        """ Launch VPN Setting from am
        """
        cmd = 'am start -a android.net.vpn.SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1].strip())
        assert ret == 0, "[ERROR] Launch VPN Setting from am fail"
        print "[INFO] Launch VPN Setting from am success"

    @staticmethod
    def stop_from_am():
        """ Stop VPN Setting from am
        """
        cmd = 'am force-stop com.android.settings; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd)
        ret = int(res_list[-1].strip())
        assert ret == 0, "[ERROR] Stop VPN Setting from am fail"
        print "[INFO] Stop VPN Setting from am success"

    def add_vpn(self, vpnprofile):
        """ Add VPN profile
        please call load_vpn_profile before this function
        """
        print "[INFO] Add VPN profile"
        if not vpnprofile:
            raise Exception("[ERROR] Parameter error: vpnprofile is none")
        self._vpnprofile = vpnprofile
        # if vpn had exist
        if self._vpn_item(vpnprofile.name) is not None:
            print "[INFO] VPN %s already exist, skip add" % vpnprofile.name
            return
        self._btn_add_profile.click()
        self._editprofile.add_vpn(vpnprofile)
        self._vpn_item(vpnprofile.name).wait.exists(timeout=10*1000)
        assert self._vpn_item(vpnprofile.name) is not None, "[ERROR] Add fail"
        print "[INFO] Add success"

    def edit_vpn(self, vpnname, newprofile):
        """ Edit VPN profile
        """ 
        print "[INFO] Edit VPN profile: %s" % vpnname
        if not newprofile:
            raise Exception("[ERROR] Parameter error: newprofile is none")
        if not vpnname:
            raise Exception("[ERROR] Parameter error: vpnname is none")
        self.d(text=vpnname).drag.to(text=vpnname)
        self._btn_edit_profile.click()
        self._editprofile.add_vpn(newprofile)
        self._vpn_item(newprofile.name).wait.exists(timeout=10*1000)
        assert self._vpn_item(newprofile.name) is not None, "[ERROR] Edit fail"
        print "[INFO] Edit success"

    def del_vpn(self, vpnname):
        """ Delete VPN profile
        """
        print "[INFO] Delete VPN profile"
        if not vpnname:
            raise Exception("[ERROR] Parameter error: vpnname is none")
        if self._vpn_item(vpnname) is None:
            raise Exception("[ERROR] Not found VPN: %s" % vpnname)
        self.d(text=vpnname).drag.to(text=vpnname)
        self._btn_del_profile.click()
        self._vpn_item(vpnname).wait.gone(timeout=2*1000)
        assert self._vpn_item(vpnname) is None, "[ERROR] Delete fail"
        print "[INFO] Delete success"

    def connect_to_vpn(self, vpnprofile):
        """ Connect to VPN
        please call add_vpn before this function.
        parameter: vpnprofile is same as passed in for function: add_vpn
        """
        if not self._connected_vpn(vpnprofile.name) is None:
            print "[INFO] The vpn [%s] has been connected,Skip this" % vpnprofile.name
            return True
        if vpnprofile is None:
            raise Exception("[ERROR] Parameter error: vpnprofile is none")
        self._vpnprofile = vpnprofile
        print "[INFO] Connect to VPN: %s" % vpnprofile.name
        self._vpn_item(vpnprofile.name).click()
        self._contovpn.connect_to(vpnprofile)
        self._label_connected.wait.exists(timeout=30*1000)
        message = "[ERROR] VPN Not Connected within 30 seconds"
        assert not self._connected_vpn(vpnprofile.name) is None, message
        print "[INFO] Connect success"

    def discon_vpn_from_notify_panel(self, vpnstatus):
        """ Disconnect VPN from notification panel
        @param vpnstatus: constant value of VPNStatus
        """
        print "[INFO] Disconnect VPN from notification panel"
        notifyPanelImpl = NotifyPanelImpl()
        vpnInNofifyPanel = VPNInNofifyPanel()
        notifyPanelImpl.open_notification_panel()
        time.sleep(2)
        if vpnInNofifyPanel.discon_vpn():
            time.sleep(2)
            self.launch_from_am()
            self.check_vpn_status(self._vpnprofile.name, vpnstatus)
            print "[INFO] Disconnect success"
        else:
            print "[INFO] VPN not active"

    @staticmethod
    def load_vpn_profile(vpnconfig):
        """ Load VPN profile from config file
        @param vpnconfig: vpn config value(instance of WifiConf)
        currently acceptable values: VPNType.PPTP, VPNType.L2TP_PSK
        @return an instance of VPNProfile
        """
        print "[INFO] Load VPN profile from config file"
        return VPNProfile(vpnconfig)

    def get_vpn_status(self, vpnname):
        """ Get VPN status: Connect/Disconnect
        @param vpnname: (string) the name of vpn display on vpn list
        @return (string) the text displayed under vpn name
        """
        print "[INFO] Get VPN status"
        if self._vpn_item(vpnname) is not None:
            return self._vpn_status(vpnname).info['text'].strip()
        return None

    def check_vpn_status(self, vpnname, exp_status):
        """ Check VPN status as expected status
        @param exp_status: constant value of VPNStatus
        """
        print "[INFO] Check VPN Status as expect"
        act_status = self.get_vpn_status(vpnname)
        print "[INFO] Actual status: %s" % act_status
        print VPNType.get_vpntype("L2TP_PSK"), exp_status in VPNType.get_vpntype("L2TP_PSK")
        if exp_status in VPNType.get_vpntype(VPNType.L2TP_PSK):
            exp_status_x = VPNType.get_vpntype(VPNType.L2TP_PSK_PRE)
            print "[INFO] Expect status: %s or %s" % (exp_status, exp_status_x)
            assert exp_status in act_status or exp_status_x in act_status
        else:
            print "[INFO] Expect status: %s" % exp_status
            assert exp_status in act_status
