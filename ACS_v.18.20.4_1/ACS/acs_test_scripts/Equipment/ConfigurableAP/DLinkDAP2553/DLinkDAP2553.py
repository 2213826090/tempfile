"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: implementation of DAP2553 configurable AP
:since:23/08/2011
:author: szhen11
"""

import time
import re
import telnetlib

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiAuthenticationTypes

from ErrorHandling.TestEquipmentException import TestEquipmentException

import UtilitiesFWK.Utilities as Util
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies


class DLinkDAP2553(EquipmentBase, IConfigurableAP):

    """
    Implementation of DAP2553 configurable AP
    """

    # Define specific list for the current equipment
    SUPPORTED_WIFI_STANDARD = ['a', 'an', 'bg', 'gb', 'bgn', 'ngb', 'n',
                               'n2.4G', 'n5G']
    SUPPORTED_WIFI_AUTHENTICATION = ['OPEN', 'WEP64', 'WEP128',
                                     'WEP64-OPEN', 'WEP128-OPEN',
                                     'WPA-PSK-TKIP','WPA-PSK-TKIP-AES',
                                     'WPA2-PSK-AES','WPA-PSK-AES',
                                     'WPA2-PSK-TKIP',
                                     'WPA2-PSK-TKIP-AES',
                                     'WPA-WPA2-PSK-TKIP-AES',
                                     'EAP-WPA', 'EAP-WPA2']

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None

        self.ap_band = None
        self.wlmode = None
        self.cipher = None
        self.authentication = None
        self.keylength1 = None
        self.key1 = None
        self.channel = None
        self.dtim = None
        self._configuration_timer = 60
        self._radio_status = "undef"

        self._console_timeout = str(self.__bench_params.get_param_value("ConsoleTimeout", "0"))
        if self._console_timeout.isdigit():
            self._console_timeout = int(self._console_timeout)

        try:
            self.init()
            self._reboot()
        except TestEquipmentException:
            self.get_logger().warning("Unable to reboot the AccessPoint")
        finally:
            self.release()

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def _set_handle(self, handle):
        """
        Sets the connection handle of the equipment
        """
        self.__handle = handle

    def _is_supported_config(self, standard_type, authentication_type):
        """
        Check if standard and authentication type combination
        is supported by Equipment

        :type standard_type: str
        :param standard_type: wifi standard type

        :type authentication_type: str
        :param authentication_type: wifi authentication type

        :rtype: boolean
        :return: if supported or not
        """
        if standard_type in ('n', 'n2.4G', 'n5G') and authentication_type in \
            ('WEP64', 'WEP128', 'WEP64-OPEN', 'WEP128-OPEN',
             'WPA-PSK-TKIP', 'EAP-WPA'):
            return False
        return True

    def _check_wifi_config(self, configindex=0):
        """
        check Equipment wifi config, include standard and authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        try:
            self.__write("get ap_band\n")
            self.__read_until(self.ap_band, "ap_band")
            self.__write("get wlmode\n")
            self.__read_until(self.wlmode, "wlmode")
        except:
            msg = "check ap standard via telnet failed"
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        try:
            if configindex == 0:
                self.__write("get authentication\n")
                self.__read_until(self.authentication, "authentication")

                if self.authentication in ["WPA-PSK", "WPA2-PSK", "WPA-EAP", "WPA2-EAP"]:
                    self.__write("get cipher\n")
                    self.__read_until(self.cipher, "cipher")
                elif self.authentication in ["Open-System", "Shared-Key"]:
                    self.__write("get cipher\n")
                    self.__read_until(self.cipher, "cipher")
                    if self.cipher == "WEP":
                        self.__write("get keylength 1\n")
                        self.__read_until(self.keylength1, "keylength1")
                        self.__write("get key 1\n")
                        self.__read_until(self.key1, "key1")
            else:
                configindex_str = str(configindex)
                self.__write("get multi-auth open-system %s\n" % configindex_str)
                self.__read_until(self.authentication, "authentication")

                if self.authentication in ["WPA-PSK", "WPA2-PSK", "WPA-EAP", "WPA2-EAP"]:
                    self.__write("get multi-cipher no %s\n" % configindex_str)
                    self.__read_until(self.cipher, "cipher")
                elif self.authentication in ["Open-System", "Shared-Key"]:
                    self.__write("get multi-cipher no %s\n" % configindex_str)
                    self.__read_until(self.cipher, "cipher")
                    if self.cipher == "WEP":
                        if self.authentication in ["WEP64", "WEP64-OPEN"]:
                            self.__write("get multi-d-wepkeylen 64Bit 1 %s\n" % configindex_str)
                        elif self.authentication in ["WEP128", "WEP128-OPEN"]:
                            self.__write("get multi-d-wepkeylen 128Bit 1 %s\n" % configindex_str)
                        self.__read_until(self.keylength1, "keylength1")
                        self.__write("get key %s\n" % configindex_str)
                        self.__read_until(self.key1, "key1")
        except:
            msg = "check ap authentication via telnet failed"
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

    def __connect_via_telnet(self, host, user, password):
        """
        connect DAP2553 via telnet thus config the equipment

        :param host: host ip
        :type host: str

        :param user: Username credential
        :type user: str

        :param password: Password credential
        :type password: str

        :raise: TestEquipmentException
        """
        self.get_logger().debug("Open telnet connection to equipment.")

        # Initialize telnet session
        telnet_session = telnetlib.Telnet()
        # debug level: 0->disable / 1->enable
        telnet_session.set_debuglevel(0)

        try:
            telnet_session.open(host)
            out = telnet_session.read_until("login:", 5)
            if "login" not in out:
                msg = "Telnet connection is out of order. "
                msg += "Please reboot the AP and restart the campaign"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
            telnet_session.write(str(user) + "\n")
            telnet_session.read_until("Password:", 5)
            telnet_session.write(str(password) + "\n")
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            msg = "connect to equipment via telnet failed"
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        # Update handle value
        self._set_handle(telnet_session)

    def __disconnect_via_telnet(self):
        """
        disconnect DAP2553 via telnet thus config the equipment
        """
        self.get_logger().debug("Close telnet connection from the equipment.")
        if self.get_handle() is not None:
            self.get_handle().close()
            self._set_handle(None)

    def __set_wifi_authentication_OPEN(self, configindex=0):
        """
        Set AP Encryption type: Open

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to OPEN")

        if configindex == 0:
            self.__write("set cipher no\n")
            self.__write("set authentication open-system\n")
        else:
            configindex_str = str(configindex)
            self.__write("set multi-cipher no %s\n" % configindex_str)
            self.__write("set multi-auth open-system %s\n" % configindex_str)

        self.__write("set apply\n")

        self.cipher = "NO"
        self.authentication = "Open-System"

    def __set_wifi_authentication_WEP64(self, authentication, passphrase, configindex=0):
        """
        Set AP Encryption type to WEP64

        :type authentication: String
        :param authentication: WEP64 or WEP64-OPEN

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WEP 64 bits")

        # WEP64 uses a 40-bit key = 5 bytes
        if len(passphrase) == 5:
            # Then we should transform ascii chars into hexadecimal values
            passphrase = Util.char2hexa(passphrase)
        elif not re.match("^[0-9a-fA-F]{10}$", passphrase):
            msg = "WEP64 passphrase should be 5 chars or 10 hexadecimal " + \
                "chars. Currently [%s]" % passphrase
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if configindex == 0:
            self.__write("set cipher wep\n")
            if authentication == "WEP64":
                self.__write("set authentication shared-key\n")
            else:
                self.__write("set authentication open\n")
            self.__write("set keyentrymethod hexadecimal 1\n")
            self.__write("set keylength 64Bit 1\n")
            self.__write("set key 1 %s\n" % str(passphrase))
            self.__write("set defkeyindex 1\n")
        else:
            configindex_str = str(configindex)
            self.__write("set multi-cipher wep %s\n" % configindex_str)
            if authentication == "WEP64":
                self.__write("set multi-auth shared-key %s\n" % configindex_str)
            else:
                self.__write("set multi-auth open %s\n" % configindex_str)

            self.__write("set keyentrymethod hexadecimal 1\n")
            self.__write("set multi-d-wepkeylen 64Bit 1 %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))
            self.__write("set multi-defkeyindex 1 %s\n" % configindex_str)

        self.__write("set apply\n")

        self.cipher = "WEP"
        if authentication == "WEP64":
            self.authentication = "Shared-Key"
        else:  # WEP64 - OPEN
            self.authentication = "Open-System"
        self.keylength1 = "64"
        self.key1 = passphrase

    def __set_wifi_authentication_WEP128(self, authentication, passphrase, configindex=0):
        """
        Set AP Encryption type to WEP128

        :type authentication: String
        :param authentication: WEP128 or WEP128-OPEN

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WEP 128 bits")

        # WEP128 uses a 104-bit key = 13 bytes
        if len(passphrase) == 13:
            # Then we should transform ascii chars into hexadecimal values
            passphrase = Util.char2hexa(passphrase)
        elif not re.match("^[0-9a-fA-F]{26}$", passphrase):
            msg = "WEP128 passphrase should be 13 chars or 26 hexadecimal " + \
                "chars. Currently [%s]" % passphrase
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if configindex == 0:
            self.__write("set cipher wep\n")
            if authentication == "WEP128":
                self.__write("set authentication shared-key\n")
            else:
                self.__write("set authentication open\n")
            self.__write("set keyentrymethod hexadecimal 1\n")
            self.__write("set keylength 128Bit 1\n")
            self.__write("set key 1 %s\n" % str(passphrase))
            self.__write("set defkeyindex 1\n")
        else:
            configindex_str = str(configindex)
            self.__write("set multi-cipher wep %s\n" % configindex_str)
            if authentication == "WEP128":
                self.__write("set multi-auth shared-key %s\n" % configindex_str)
            else:
                self.__write("set multi-auth open %s\n" % configindex_str)
            self.__write("set keyentrymethod hexadecimal 1\n")
            self.__write("set multi-d-wepkeylen 128Bit 1 %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))
            self.__write("set multi-defkeyindex 1 %s\n" % configindex_str)

        self.__write("set apply\n")

        self.cipher = "WEP"
        if authentication == "WEP128":
            self.authentication = "Shared-Key"
        else:  # WEP128 - OPEN
            self.authentication = "Open-System"
        self.keylength1 = "128"
        self.key1 = passphrase

    def __set_wifi_authentication_WPA_PSK_TKIP(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP")

        if configindex == 0:
            self.__write("set authentication wpa-psk\n")
            self.__write("set cipher TKIP\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa-psk %s\n" % configindex_str)
            self.__write("set multi-cipher TKIP %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "TKIP"
        self.authentication = "WPA-PSK"

    def __set_wifi_authentication_WPA2_PSK_AES(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA2 PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK AES")

        if configindex == 0:
            self.__write("set authentication wpa2-psk\n")
            self.__write("set cipher AES\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa2-psk %s\n" % configindex_str)
            self.__write("set multi-cipher AES %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "AES"
        self.authentication = "WPA2-PSK"

    def __set_wifi_authentication_WPA_PSK_AES(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK AES")

        if configindex == 0:
            self.__write("set authentication wpa-psk\n")
            self.__write("set cipher AES\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa-psk %s\n" % configindex_str)
            self.__write("set multi-cipher AES %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "AES"
        self.authentication = "WPA-PSK"

    def __set_wifi_authentication_WPA_PSK_TKIP_AES(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP AES")

        if configindex == 0:
            self.__write("set authentication wpa-psk\n")
            self.__write("set cipher auto\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa-psk %s\n" % configindex_str)
            self.__write("set multi-cipher auto %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "auto"
        self.authentication = "WPA-PSK"

    def __set_wifi_authentication_WPA2_PSK_TKIP(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA2 PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
       """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP")

        if configindex == 0:
            self.__write("set authentication wpa2-psk\n")
            self.__write("set cipher TKIP\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa2-psk %s\n" % configindex_str)
            self.__write("set multi-cipher TKIP %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "TKIP"
        self.authentication = "WPA2-PSK"

    def __set_wifi_authentication_WPA2_PSK_TKIP_AES(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP AES")

        if configindex == 0:
            self.__write("set authentication wpa2-psk\n")
            self.__write("set cipher auto\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa2-psk %s\n" % configindex_str)
            self.__write("set multi-cipher auto %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "auto"
        self.authentication = "WPA2-PSK"

    def __set_wifi_authentication_EAP_WPA(self,
                                          radiusip, radiusport, radiussecret, configindex=0):
        """
        set AP Encryption type: EAP-WPA

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA EAP")

        if radiusip is None or radiusport is None or radiussecret is None or \
                radiusip == "" or radiusport == "" or radiussecret == "":
            msg = "Radius configuration is missing in BenchConfig"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if configindex == 0:
            self.__write("set authentication wpa-eap\n")
            self.__write("set cipher auto\n")
            self.__write("set radiusip %s \n" % str(radiusip))
            self.__write("set radiusport %s \n" % str(radiusport))
            self.__write("set radiussecret %s \n" % str(radiussecret))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa-eap %s\n" % configindex_str)
            self.__write("set multi-cipher auto\n")
            self.__write("set multi-radiusip %s %s\n" % (str(radiusip), configindex_str))
            self.__write("set multi-radiusport %s %s\n" % (str(radiusport), configindex_str))
            self.__write("set multi-radiussecret %s %s\n" % (str(radiussecret), configindex_str))

        self.__write("set apply\n")

        self.cipher = "auto"
        self.authentication = "WPA-EAP"

    def __set_wifi_authentication_EAP_WPA2(self,
                                           radiusip, radiusport, radiussecret, configindex=0):
        """
        set AP Encryption type: EAP-WPA2

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA2 EAP")

        if radiusip is None or radiusport is None or radiussecret is None or \
                radiusip == "" or radiusport == "" or radiussecret == "":
            msg = "Radius configuration is missing in BenchConfig"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if configindex == 0:
            self.__write("set authentication wpa2-eap\n")
            self.__write("set cipher auto\n")
            self.__write("set radiusip %s \n" % str(radiusip))
            self.__write("set radiusport %s \n" % str(radiusport))
            self.__write("set radiussecret %s \n" % str(radiussecret))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa2-eap %s\n" % configindex_str)
            self.__write("set multi-cipher auto\n")
            self.__write("set multi-radiusip %s %s\n" % (str(radiusip), configindex_str))
            self.__write("set multi-radiusport %s %s\n" % (str(radiusport), configindex_str))
            self.__write("set multi-radiussecret %s %s\n" % (str(radiussecret), configindex_str))

        self.__write("set apply\n")

        self.cipher = "auto"
        self.authentication = "WPA2-EAP"

    # pylint: disable=W0221
    def create_ssid(self, ssid, hidden=False, configindex=0):
        """
        create ssid on the equipement

        :type ssid: str
        :param ssid: SSID to create

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """

        self.get_logger().info("Create ssid '%s'" % str(ssid))

        if configindex == 0:
            if hidden:
                self.__write("set ssidhidden enable\n")
            else:
                self.__write("set ssidhidden disable\n")
            self.__write("set ssid %s\n" % str(ssid))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-state enable %s\n" % configindex_str)
            if hidden:
                self.__write("set multi-ssidhidden enable %s\n" % configindex_str)
            else:
                self.__write("set multi-ssidhidden disable %s\n" % configindex_str)
            self.__write("set multi-ssid %s %s\n" % (str(ssid), configindex_str))
            self.__write("set multi-ind-state enable %s\n" % configindex_str)

        self.__write("set apply\n")
    # pylint: enable=W0221

    def __set_wifi_authentication_WPA_WPA2_PSK_TKIP_AES(self, passphrase, configindex=0):
        """
        Set AP Encryption type to WPA WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().debug("Set wifi authentication to WPA WPA2 PSK TKIP AES")

        if configindex == 0:
            self.__write("set authentication wpa2-auto-psk\n")
            self.__write("set cipher auto\n")
            self.__write("set passphrase %s\n" % str(passphrase))
        else:
            configindex_str = str(configindex)
            self.__write("set multi-auth wpa2-auto-psk %s\n" % configindex_str)
            self.__write("set multi-cipher auto %s\n" % configindex_str)
            self.__write("set multi-passphrase %s %s\n" % (str(passphrase), configindex_str))

        self.__write("set apply\n")

        self.cipher = "auto"
        self.authentication = "WPA/WPA2-PSK"

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        # Extract time to wait for configuration of the equipment
        self._configuration_timer = float(self.__bench_params.get_param_value("ConfigurationWaitingTime"))

        if self.get_handle() is not None:
            return

        if not str(self._console_timeout).isdigit():
            msg = "Wrong value for ConsoleTime in BenchConfig: " + str(self._console_timeout)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Retrieve parameters from BenchConfig for connection
        host = str(self.__bench_params.get_param_value("IP"))
        username = str(self.__bench_params.get_param_value("username"))
        password = str(self.__bench_params.get_param_value("password"))

        # Open telnet session
        connection_attempts = 5
        while connection_attempts > 0:
            try:
                self.__connect_via_telnet(host, username, password)
                break
            except TestEquipmentException as e:
                self._logger.info(e)

                # In case of Telnet connection failure, wait for the Telnet timeout to expire,
                # in order to be able to reconnect during next attempt
                msg = "Waiting for the Telnet connection to reset from the AP "
                msg += "(%d min)" % self._console_timeout
                self._logger.warning(msg)
                time.sleep(self._console_timeout * 60 + 10)

                connection_attempts -= 1
                if connection_attempts <= 0:
                    raise e
                self.release()

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")

        self.__disconnect_via_telnet()

    def set_wifi_config(self,
                        ssid,
                        hidden,
                        standard_type,
                        authentication_type,
                        passphrase,
                        channel=None,
                        dtim=None,
                        beacon=None,
                        wmm=None,
                        bandwidth=None,
                        mimo=False,
                        radiusip=None,
                        radiusport=None,
                        radiussecret=None,
                        configindex=0):
        """
        set wifi config, include standard and authentication

        :type ssid: str
        :param ssid: access point SSID

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled

        :type standard_type: str
        :param standard_type: wifi standard type

        :type authentication_type: str
        :param authentication_type: wifi authentication type

        :type passphrase: str
        :param passphrase: wifi passphrase

        :type channel: integer
        :param channel: wifi channel number (optional)

        :type dtim: integer
        :param dtim: wifi DTIM interval (optional)

        :type beacon: integer
        :param beacon: beacon interval in ms (optional)

        :type wmm: integer
        :param wmm: Enable/Disable wifi wmm (optional)

        :type bandwidth: str
        :param bandwidth: bandwidth to use (optional)

        :type mimo: Boolean
        :param mimo: enable/disable Multiple Input Multiple Output feature on the AP (Optional)

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP and Radius server (optional)

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().info("Set wifi multi configuration: index %s" % str(configindex))

        self.create_ssid(ssid, hidden, configindex)

        if standard_type not in DLinkDAP2553.SUPPORTED_WIFI_STANDARD:
            msg = "wifi standard type: %s not correct" % standard_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if authentication_type not in DLinkDAP2553.SUPPORTED_WIFI_AUTHENTICATION:
            msg = "wifi authentication type: %s not correct" % authentication_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if not self._is_supported_config(standard_type, authentication_type):
            msg = "wifi standard type: %s with authentication type: %s not supported" % (standard_type, authentication_type)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.set_wifi_standard(standard_type, mimo)

        self.set_wifi_authentication(authentication_type, passphrase,
                                     radiusip, radiusport, radiussecret, standard_type, configindex)

        if channel is not None:
            # Set the wifi channel if exists
            self.set_wifi_channel(standard_type, channel)
        if dtim is not None:
            # Set the wifi dtim if exists
            self.set_wifi_dtim(dtim)
        if beacon is not None:
            # Set the wifi beacon interval if exists
            self.set_wifi_beacon(beacon)
        if wmm is not None:
            # Enable/disable wmm if exists
            self._set_wifi_wmm(wmm, configindex)
        if bandwidth is not None:
            # Set the bandwidth if exists
            self.set_wifi_bandwidth(bandwidth, standard_type)

        # Waiting for configuration to be applied
        time.sleep(self._configuration_timer)

        # Check the configuration
        self._check_wifi_config(configindex)

    # pylint: disable=W0221
    def set_wifi_authentication(self, authentication_type, passphrase="",
                                radiusip=None, radiusport=None,
                                radiussecret=None,
                                standard_type=None,
                                configindex=0):
        """
        Set the authentication on the equipment

        :type authentication_type: String
        :param authentication_type: Authentication supported by the equipment

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP and Radius server (optional)

        :type standard_type: str
        :param standard_type: The wifi standard used to determine if the \
                            channel has to be set for 2.4GHz or 5GHz

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().info("Set wifi authentication to '%s'" % str(authentication_type))

        if authentication_type == WifiAuthenticationTypes.OPEN:
            self.__set_wifi_authentication_OPEN(configindex)

        elif authentication_type == WifiAuthenticationTypes.WEP_64 or \
                authentication_type == WifiAuthenticationTypes.WEP_64_OPEN:
            self.__set_wifi_authentication_WEP64(authentication_type, passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WEP_128 or \
                authentication_type == WifiAuthenticationTypes.WEP_128_OPEN:
            self.__set_wifi_authentication_WEP128(authentication_type, passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA_PSK_TKIP:
            self.__set_wifi_authentication_WPA_PSK_TKIP(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_AES:
            self.__set_wifi_authentication_WPA2_PSK_AES(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.EAP_WPA:
            self.__set_wifi_authentication_EAP_WPA(radiusip, radiusport,
                                                   radiussecret, configindex)

        elif authentication_type == WifiAuthenticationTypes.EAP_WPA2:
            self.__set_wifi_authentication_EAP_WPA2(radiusip, radiusport,
                                                    radiussecret, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA_PSK_AES:
            self.__set_wifi_authentication_WPA_PSK_AES(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA_PSK_TKIP_AES:
            self.__set_wifi_authentication_WPA_PSK_TKIP_AES(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP:
            self.__set_wifi_authentication_WPA2_PSK_TKIP(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP_AES:
            self.__set_wifi_authentication_WPA2_PSK_TKIP_AES(passphrase, configindex)

        elif authentication_type == WifiAuthenticationTypes.WPA_WPA2_PSK_TKIP_AES:
            self.__set_wifi_authentication_WPA_WPA2_PSK_TKIP_AES(passphrase, configindex)

        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi authentication '%s'" % str(authentication_type))
    # pylint: enable=W0221

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        Set wifi standard

        :type standard: String
        :param standard: Wifi standard to set
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        self.get_logger().info("Set wifi standard to '%s'" % str(standard))

        if enable_mimo:
            self._logger.warning("MIMO is not available on this Access Point")

        try:
            if standard == "a":
                self.ap_band = "5G"
                self.wlmode = "80211a"
                self.__write("set ap_band 5G\n")
                self.__write("set wlmode only-a\n")
            elif standard == "an":
                self.ap_band = "5G"
                self.wlmode = "80211a 80211n"
                self.__write("set ap_band 5G\n")
                self.__write("set wlmode an\n")
            elif standard == "n5G":
                self.ap_band = "5G"
                self.wlmode = "80211n (5GHz)"
                self.__write("set ap_band 5G\n")
                self.__write("set wlmode only-n5G\n")
            elif standard == "gb" or standard == "bg":
                self.ap_band = "2.4G"
                self.wlmode = "80211b 80211g"
                self.__write("set ap_band 2.4G\n")
                self.__write("set wlmode gb\n")
            elif standard == "ngb" or standard == "bgn":
                self.ap_band = "2.4G"
                self.wlmode = "80211n 80211g 80211b"
                self.__write("set ap_band 2.4G\n")
                self.__write("set wlmode ngb\n")
            elif standard == "n2.4G" or standard == "n":
                self.ap_band = "2.4G"
                self.wlmode = "80211n"
                self.__write("set ap_band 2.4G\n")
                self.__write("set wlmode only-n2.4G\n")
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Unsupported wifi standard '%s'." % str(standard))

            self.__write("set apply\n")

        except:
            msg = "set wifi standard failed."
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def set_wifi_channel(self, standard, channel):
        """
        Set wifi channel

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz
        :type channel: integer/str
        :param channel: The wifi channel to set
        """
        channel = str(channel)
        if channel == "0":
            self.get_logger().info("Set wifi channel to auto")
            self.__write("set autoChannel enable\n")

        else:
            self.__write("set autoChannel disable\n")

            if standard not in AcsWifiFrequencies.WIFI_STANDARD_5G:
                if int(channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                        "channel is out of range: " + channel)
            else:
                if int(channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                        "channel is out of range: " + channel)
            self.get_logger().info("Set wifi channel to " + channel)
            self.__write("set channel %s\n" % channel)

        self.__write("set apply\n")
        self.channel = channel

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim:
        """
        self.get_logger().info("Set wifi DTIM to '%s'" % str(dtim))

        self.__write("set dtim %s\n" % str(dtim))
        self.__write("set apply\n")
        self.dtim = dtim

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        self.get_logger().info("Set wifi beacon to '%s'" % str(beacon))

        self.__write("set beaconinterval %s\n" % str(beacon))
        self.__write("set apply\n")

    def _set_wifi_wmm(self, mode, configindex=0):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 - 0 is the default SSID data (optional)
        """
        if configindex == 0:
            if mode in ("on", "1", 1):
                self.get_logger().info("Set wifi wmm to on")
                self.__write("set wmm enable\n")
            elif mode in ("off", "0", 0):
                self.get_logger().info("Set wifi wmm to off")
                self.__write("set wmm disable\n")
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter mode is not valid !")
        else:
            configindex_str = str(configindex)
            if mode in ("on", "1", 1):
                self.get_logger().info("Set wifi wmm to on")
                self.__write("set multi-wmm enable %s\n" % configindex_str)
            elif mode in ("off", "0", 0):
                self.get_logger().info("Set wifi wmm to off")
                self.__write("set multi-wmm disable %s\n" % configindex_str)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter mode is not valid !")

        self.__write("set apply\n")

    def set_wifi_bandwidth(self, bandwidth, standard):
        """
        Set wifi bandwidth

        :type bandwidth: str
        :param bandwidth: Wifi bandwidth to set

        :type standard: str
        :param standard: The wifi standard used to control the validity of
                         the bandwidth value or to select the good antenna.

        :raise: TestEquipmentException
        """

        if (standard not in ['n', 'n2.4G', 'n5G']) and (int(bandwidth) == 40):
            msg = "Trying to set 40Mhz bandwitdh with a non N standard."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_logger().info("Set wifi bandwidth to %s" % bandwidth)

        if "20" in bandwidth:
            bandwidth = "20Mhz"
        elif "40" in bandwidth:
            bandwidth = "auto"
        else:
            msg = "bandwidth parameter invalid: " + str(bandwidth)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.__write("set cwm %s\n" % bandwidth)
        self.__write("set apply\n")

    def enable_wireless(self, standard=None):
        """
        enable wireless network

        :type standard: str
        :param standard: type of frequency to enable

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        ret = True
        retries = 5

        # Check if the wireless is not already enabled
        self.__write("get wireless \n")
        status = self.get_handle().read_until("enable", 5)

        # Correct the radio status flag if necessary
        if "enable" in status:
            self._radio_status = "ON"
        elif "disable" in status:
            self._radio_status = "OFF"
        else:
            self._radio_status = "undef"

        if self._radio_status != "ON":

            while "enable" not in status and retries > 0:
                self.get_logger().info("Enable wireless")

                self.__write("set wireless enable \n")
                self.__write("set apply\n")

                # Waiting for configuration to be applied
                time.sleep(self._configuration_timer)

                # control that the wireless has well been enabled
                self.__write("get wireless \n")
                status = self.get_handle().read_until("enable", 5)
                retries -= 1

            if "enable" not in status:
                self._logger.error("Enable wireless fails")
                ret = False

            self._radio_status = "ON"
        else:
            self._logger.info("Wireless already enabled")

        return ret

    def disable_wireless(self):
        """
        disable wireless

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        ret = True
        retries = 5

        # Check if the wireless is not already disabled
        self.__write("get wireless \n")
        status = self.get_handle().read_until("disable", 5)

        # Correct the radio status flag if necessary
        if "disable" in status:
            self._radio_status = "OFF"
        elif "enable" in status:
            self._radio_status = "ON"
        else:
            self._radio_status = "undef"

        if not self.is_radio_off():

            while "disable" not in status and retries > 0:
                self.get_logger().info("Disable wireless")

                self.__write("set wireless disable \n")
                self.__write("set apply\n")

                # Waiting for configuration to be applied
                time.sleep(self._configuration_timer)

                # control that the wireless has well been disabled
                self.__write("get wireless \n")
                status = self.get_handle().read_until("disable", 5)
                retries -= 1

            if "disable" not in status:
                self._logger.error("Disable wireless fails")
                ret = False
            self._radio_status = "OFF"
        else:
            self._logger.info("Wireless already disabled")

        return ret

    def set_acl_mode(self, mode):
        """
        Enable/Disable wifi MAC address filtering

        :type mode: str
        :param mode: can be ("disable", "enable"(reject), "accept")
        """
        if mode == "enable":
            mode = "reject"

        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Check if acl mode is already set
        self.__write("get acl \n")
        status = self.get_handle().read_until("%s" % mode, 5)

        if mode in status:
            self._logger.info("acl mode already set")
            return

        if mode in ("disable", "accept", "reject"):
            self.get_logger().info("Set acl mode to %s" % mode)
            self.__write("set acl %s\n" % mode)
        else:
            self._logger.error("Parameter mode is not valid: %s" % mode)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Parameter mode is not valid: %s" % mode)
        self.__write("set apply\n")
        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Waiting for configuration to be applied
        time.sleep(self._configuration_timer)

        # control that the acl mode is correctly set
        self.__write("get acl \n")
        status = self.get_handle().read_until(mode, 5)

        if mode not in status:
            self._logger.error("Unable to set acl status")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Unable to set acl status")

    def add_mac_address_to_acl(self, mac_addr):
        """
        Add mac address to AP access control list

        :type mac_addr: str
        :param mac_addr: can be "08:00:28:xx:xx:xx" where x is between [0-F]
        """
        # Check the format of the mac_addr parameter
        mac_addr = mac_addr.strip().upper()
        expr = "^(([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2})$"
        match = re.compile(expr).search(mac_addr)
        if not match:
            self._logger.error("Parameter mac_addr is not valid: %s" % mac_addr)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Parameter mac_addr is not valid: %s" % mac_addr)

        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Check if mac address is already in the list
        self.__write("get macaddrlist \n")
        status = self.get_handle().read_until("%s" % mac_addr, 5)
        if mac_addr in status:
            self._logger.info("mac address already in mac address acl list")
            return

        self.get_logger().info("add %s to acl" % mac_addr)
        self.__write("set macaddradd %s\n" % mac_addr)
        self.__write("set apply\n")
        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Waiting for configuration to be applied
        time.sleep(self._configuration_timer)

        # control that the mac address is in the acl list
        self.__write("get macaddrlist \n")
        status = self.get_handle().read_until("%s" % mac_addr, 5)
        if mac_addr not in status:
            msg = "Unable to add mac address to acl list: %s" % mac_addr
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def del_mac_address_from_acl(self, mac_addr):
        """
        Remove mac address from AP access control list

        :type mac_addr: str
        :param mac_addr: can be "08:00:28:xx:xx:xx" where x is between [0-F]
        """
        # Check the format of the mac_addr parameter
        mac_addr = mac_addr.strip().upper()
        expr = "^(([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2})$"
        match = re.compile(expr).search(mac_addr)
        if not match:
            self._logger.error("Parameter mac_addr is not valid: %s" % mac_addr)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Parameter mac_addr is not valid: %s" % mac_addr)

        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Check if mac address is already in the list
        self.__write("get macaddrlist \n")
        status = self.get_handle().read_until("%s" % mac_addr, 5)
        if mac_addr not in status:
            self._logger.info("mac address is not present in acl list")
            return

        self.get_logger().info("removing %s from acl" % mac_addr)
        self.__write("set macaddrdel %s\n" % mac_addr)
        self.__write("set apply\n")

        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Waiting for configuration to be applied
        time.sleep(self._configuration_timer)

        # control that the mac address is in the acl list
        self.__write("get macaddrlist \n")
        status = self.get_handle().read_until(">", 5)
        if mac_addr in status:
            msg = "Unable to remove mac address from acl list: %s" % mac_addr
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def _reboot(self):
        """
        Reboots the Access Point
        """
        self.get_logger().info("Reboot AP")
        self.__write("reboot \n")
        time.sleep(self._configuration_timer * 2)

    def is_regulatorydomain_configurable(self):
        """
        Function used to know if it is possible to configure the Regulatory
        Domain for this Configuable Access Point
        If True, the function set_regulatorydomain() must be implemented
        If False, the function get_regulatorydomain() should be implemented

        :rtype: boolean
        :return: True if the Regulatory Domain is configurable
        """
        return False

    def is_regulatorydomain_sent(self):
        """
        Function used to know if the Access Point automatically sends the
        Regulatory Domain to their clients.

        :rtype: boolean
        :return: True if the AP automatically sends the regulatory domain
        """
        return False

    def get_regulatorydomain(self):
        """
        Return current regulatory domain for the Access Point

        :rtype: String
        :return: The current regulatory domain set
        """
        self.get_logger().info("Get the regulatory domain")

        # Flush the reader buffer
        self.get_handle().read_very_eager()

        # Run the command
        self.__write("get country\n")
        out = self.get_handle().read_until(">", 2)
        if "Country:" not in out:
            self._logger.debug("Fail to find country code once: " + out)
            out = self.get_handle().read_until(">", 4)

        # Parse the result
        match = re.search(r"Country:([A-Z]*)", out)
        if match is None or match.group(1) is None or match.group(1) == "":
            self._logger.debug("Fail to find country code twice: " + out)
            msg = "Unable to parse regulatory domain"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

        return match.group(1)

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF"

    def get_client_macaddr_list(self):
        """
        Get the Client MAC addresses list
        The MAC Addresses returned are in lower case.
        The format of MAC addresses is: xx:xx:xx:xx:xx:xx where x is 0-9 or a-f.

        :rtype: list
        :return: list of MAC addresses of the clients connected to the Access Point
        """
        self.get_logger().info("Get Client MAC addresses list")
        local_connection = False

        if self.get_handle() is None:
            # Connect to the AP if no connection ongoing
            local_connection = True
            self.init()

        try:
            # Flush the reader buffer
            self.get_handle().read_very_eager()

            # Run the command
            self.__write("get clientinfo\n")
            out = self.get_handle().read_until(">", 2)
            self._logger.debug("output=" + out)

            # Find all MAC addresses, preceeded by the word 'mac:', from 'out' str
            client_list = re.findall(r'(?:mac:)((?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})', out)

            # Set lower case all MAC addresses found
            for index in range(len(client_list)):
                client_list[index] = client_list[index].lower()
            self._logger.debug("client list=" + str(client_list))

        finally:
            if local_connection:
                # Disconnection from AP if connection was handled at the beggining of the function
                self.release()

        return client_list

    def get_selected_channel(self):
        """
        Get the selected WiFi channel

        :rtype: int
        :return: the selected channel number
        """
        self.get_logger().info("Get selected Wifi channel")
        local_connection = False
        channel = ""

        if self.get_handle() is None:
            # Connect to the AP if no connection ongoing
            local_connection = True
            self.init()

        try:
            # Flush the reader buffer
            self.get_handle().read_very_eager()

            # Run the command
            self.__write("get channel\n")
            out = self.get_handle().read_until(">", 2)
            self._logger.debug("output=" + out)

            # Find the channel number
            channel = re.findall(r'channel:([0-9]+)', out)
            if len(channel) > 0:
                channel = channel[0]
            else:
                channel = ""

        finally:
            if local_connection:
                # Disconnection from AP if connection was handled at the beggining of the function
                self.release()

        if not channel.isdigit():
            msg = "Unable to retrieve Channel used by AccessPoint"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        channel = int(channel)

        self._logger.debug("Selected channel is : %d" % channel)
        return channel

    def __write(self, cmd):
        self.get_handle().write(cmd)
        time.sleep(0.3)

    def __read_until(self, until, label, timeout=5):
        """
        Reads telnet ouput until meet the str "until".
        The function stops after timeout expiration and raise an exception if
        the str has not been read.

        :type until: str
        :param until: reads until this str
        :type label: str
        :param label: error message includes that str
        :type timeout: int
        :param timeout: time to wait for the output to contain "until" str
        """
        out = self.get_handle().read_until("%s" % (str(until)), timeout)
        if until not in out:
            msg = "%s not correctly set. Read: %s" % (label, out)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
