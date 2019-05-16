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
:summary: implementation of WRVS4400N configurable AP
:since:24/10/2011
:author: jpstierlin
"""

import urllib2

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiAuthenticationTypes, build_wep_key_64, build_wep_key_128
from acs_test_scripts.Equipment.ConfigurableAP.CiscoWRVS4400N.WifiConfigurationParser import WifiConfigurationParser

from ErrorHandling.TestEquipmentException import TestEquipmentException


class CiscoWRVS4400N(EquipmentBase, IConfigurableAP):

    """
    Implementation of WRVS4400N configurable AP
    """

    # Define specific list for the current equipment
    SUPPORTED_WIFI_STANDARD = ['b', 'g', 'n', 'bg', 'gb', 'gn', 'ng', 'bgn', 'ngb', 'off']
    SUPPORTED_WIFI_AUTHENTICATION = ['OPEN', 'WEP64', 'WEP128', 'WPA-PSK-TKIP', 'WPA-PSK-AES',
                                     'WPA2-PSK-AES', 'WPA2-PSK-TKIP', 'WPA2-PSK-TKIP-AES']

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None
        self._standard = None
        self._radio_status = "undef"

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
        Check supported combination of wifi standard and authentication

        :type standard: String
        :param standard: Wifi standard to set

        :type authentication_type: String
        :param authentication_type: Authentication supported by the equipment
        """
        if standard_type == 'n' and authentication_type in ['WEP64', 'WEP128']:
            return False
        return True

    def __connect_via_http(self, host, user, password):
        """
        connect to equipment via http to configure the access point

        :type host: str
        :param host: access point IP address

        :type user: str
        :param user: access point admin login

        :type password: str
        :param password: access point admin password

        :raise: TestEquipmentException
        """
        self.get_logger().debug("Open http connection to equipment.")
        try:
            password_mgr = urllib2.HTTPPasswordMgrWithDefaultRealm()
            password_mgr.add_password('Router', 'http://' + host, user, password)
            handler = urllib2.HTTPDigestAuthHandler(password_mgr)
            opener = urllib2.build_opener(handler)
            urllib2.install_opener(opener)
            handle = WifiConfigurationParser(self.get_logger(), host)
            handle.download_configuration()

            self._set_handle(handle)

        except:
            msg = "Http connection failed."
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    def __set_wifi_authentication_OPEN(self):
        """
        Set AP Encryption type: Open
        """
        self.get_logger().debug("Set wifi authentication to OPEN")
        self.get_handle().set_value('wl_security', 'disabled')

    def __set_wifi_authentication_WEP64(self, passphrase):
        """
        Set AP Encryption type to WEP64

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 64 bits")

        self.get_handle().set_value('wl_security', 'wep')
        self.get_handle().set_value('wep_auth_type', 4)  # 3:Open System 4:Shared Key
        self.get_handle().set_value('wl_keyindex', 1)
        keys = build_wep_key_64(passphrase)
        for index in range(4):
            self.get_handle().set_value('wl_key' + str(index + 1), keys[index])

    def __set_wifi_authentication_WEP128(self, passphrase):
        """
        Set AP Encryption type to WEP128

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 128 bits")

        self.get_handle().set_value('wl_security', 'wep')
        self.get_handle().set_value('wep_auth_type', 4)  # 3:Open System 4:Shared Key
        self.get_handle().set_value('wl_keyindex', 1)
        keys = build_wep_key_128(passphrase)
        for index in range(4):
            self.get_handle().set_value('wl_key' + str(index + 1), keys[0])

    def __set_wifi_authentication_WPA_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP")

        self.get_handle().set_value('wl_security', 'psk')
        self.get_handle().set_value('wl_psk', passphrase)
        self.get_handle().set_value('wl_algo', 'tkip')

    def __set_wifi_authentication_WPA_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK AES")

        self.get_handle().set_value('wl_security', 'psk')
        self.get_handle().set_value('wl_psk', passphrase)
        self.get_handle().set_value('wl_algo', 'aes')

    def __set_wifi_authentication_WPA2_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK AES")

        self.get_handle().set_value('wl_security', 'psk2')
        self.get_handle().set_value('wl_psk2', passphrase)
        self.get_handle().set_value('wl_algo', 'aes')

    def __set_wifi_authentication_WPA2_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP")

        self.get_handle().set_value('wl_security', 'psk2')
        self.get_handle().set_value('wl_psk2', passphrase)
        self.get_handle().set_value('wl_algo', 'tkip')

    def __set_wifi_authentication_WPA2_PSK_TKIP_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP AES")

        self.get_handle().set_value('wl_security', 'psk2_mixed')
        self.get_handle().set_value('wl_psk3', passphrase)
        self.get_handle().set_value('wl_algo', 'aes')

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        # Retrieve parameters from BenchConfig for connection
        host = str(self.__bench_params.get_param_value("IP"))
        username = str(self.__bench_params.get_param_value("username"))
        password = str(self.__bench_params.get_param_value("password"))

        # Open telnet session
        self.__connect_via_http(host, username, password)

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")
        self._set_handle(None)

    def create_ssid(self, ssid, hidden=False):
        """
        Create ssid on the equipment

        :type ssid: str
        :param ssid: SSID to create

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled
        """
        self.get_handle().set_value('wl_ssid', ssid)
        if hidden:
            self.get_handle().set_value('wl_ssid_bc', 'disable')
        else:
            self.get_handle().set_value('wl_ssid_bc', 'enable')

    def set_wifi_authentication(self, authentication_type, passphrase="",
                                radiusip=None, radiusport=None,
                                radiussecret=None,
                                standard_type=None):
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
        """
        self.get_logger().info("Set wifi authentication to '%s'" % str(authentication_type))

        if authentication_type == WifiAuthenticationTypes.OPEN:
            self.__set_wifi_authentication_OPEN()

        elif authentication_type == WifiAuthenticationTypes.WEP_64:
            self.__set_wifi_authentication_WEP64(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WEP_128:
            self.__set_wifi_authentication_WEP128(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WPA_PSK_TKIP:
            self.__set_wifi_authentication_WPA_PSK_TKIP(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WPA_PSK_AES:
            self.__set_wifi_authentication_WPA_PSK_AES(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_AES:
            self.__set_wifi_authentication_WPA2_PSK_AES(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP:
            self.__set_wifi_authentication_WPA2_PSK_TKIP(passphrase)

        elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP_AES:
            self.__set_wifi_authentication_WPA2_PSK_TKIP_AES(passphrase)

        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi authentication '%s'" % str(authentication_type))

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        Set wifi standard

        :type standard: String
        :param standard: Wifi standard to set
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        self.get_logger().info("Set wifi standard to '%s'" % standard)

        if enable_mimo:
            self._logger.warning("MIMO is not available on this Access Point")

        if standard == 'gb':
            standard = 'bg'
        elif standard == 'ng':
            standard = 'gn'
        elif standard == 'ngb':
            standard = 'bgn'
        self.get_handle().set_value('wl_mode', standard)
        self._standard = standard

    def set_wifi_channel(self, standard, channel):
        """
        Set wifi channel

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz
        :type channel: integer or Digit-String
        :param channel: The wifi channel to set
        """
        self.get_logger().info("Set wifi channel to '%s'" % str(channel))
        self.get_handle().set_value('wl_channel', channel)

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim:
        """
        self.get_logger().info("Set wifi DTIM to '%s'" % str(dtim))
        self.get_handle().set_value('wl_dtim', dtim)

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        self.get_logger().info("Set wifi beacon to '%s'" % str(beacon))
        self.get_handle().set_value('wl_beacon', beacon)

    def set_wifi_wmm(self, mode):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        if mode in ("on", "1", 1):
            self.get_logger().info("Set wifi wmm to on")
            self.get_handle().set_value('wl_wmm', "enable")

        elif mode in ("off", "0", 0):
            self.get_logger().info("Set wifi wmm to off")
            self.get_handle().set_value('wl_wmm', "disable")

        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Parameter mode is not valid !")

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

        # Convert to str
        bandwidth = str(bandwidth)
        self.get_logger().info("Set wifi bandwidth to %s" % bandwidth)

        if bandwidth == "20":
            value = '0'
        elif bandwidth == "40":
            value = '1'
        else:
            msg = "Unsupported bandwidth '%s'" % bandwidth
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if (standard not in ['n', 'n2.4G', 'n5G']) and (int(bandwidth) == 40):
            msg = "Trying to set 40Mhz bandwitdh with a non N standard."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_handle().set_value('radio_band', value)

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
        :param configindex: Configuration index - can be 0 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().info("Set wifi configuration")

        if configindex != 0:
            msg = "Configuration index can be only zero : %s not correct" % configindex
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # Extract time to wait for configuration of the equipment
        configuration_timer = float(self.__bench_params.get_param_value("ConfigurationWaitingTime"))

        if standard_type not in CiscoWRVS4400N.SUPPORTED_WIFI_STANDARD:
            msg = "wifi standard type: %s not correct" % standard_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if authentication_type not in CiscoWRVS4400N.SUPPORTED_WIFI_AUTHENTICATION:
            msg = "wifi authentication type: %s not correct" % authentication_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if not self._is_supported_config(standard_type, authentication_type):
            msg = "wifi standard type: %s with authentication type: %s not supported" % (standard_type, authentication_type)
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # disable multiple ssids
        self.get_handle().set_value('wl_mbssid', 'disable')
        # disable MAC address filtering
        self.get_handle().set_value('wl_macfilter', 'disable')
        self.create_ssid(ssid, hidden)
        self.set_wifi_standard(standard_type, mimo)
        self.set_wifi_authentication(authentication_type, passphrase)

        if channel is not None:
            self.set_wifi_channel(standard_type, channel)
        if dtim is not None:
            self.set_wifi_dtim(dtim)
        if beacon is not None:
            self.set_wifi_beacon(beacon)
        if wmm is not None:
            self.set_wifi_wmm(wmm)
        if bandwidth is not None:
            self.set_wifi_bandwidth(bandwidth, standard_type)

        self.get_handle().upload_configuration(configuration_timer)

    def enable_wireless(self):
        """
        enable wireless

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Enable wireless")
        status = True

        if self._radio_status != "ON":
            if self._standard is None:
                msg = "Cannot call enable_wireless() before calling " \
                      + "set_wifi_standard() method"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            self.get_handle().set_value('wl_mode', self._standard)
            configuration_timer = float(self.__bench_params.get_param_value("ConfigurationWaitingTime"))
            self.get_handle().upload_configuration(configuration_timer)
            self._radio_status = "ON"
        else:
            self._logger.info("Wireless already enabled")

        return status

    def disable_wireless(self):
        """
        disable wireless

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Disable wireless")
        status = True

        if not self.is_radio_off():
            standard = self.get_handle().get_value('wl_mode')
            if standard != "off":
                self._standard = standard
            self.get_handle().set_value('wl_mode', 'off')
            configuration_timer = float(self.__bench_params.get_param_value("ConfigurationWaitingTime"))
            self.get_handle().upload_configuration(configuration_timer)
            self._radio_status = "OFF"
        else:
            self._logger.info("Wireless already disabled")

        return status

    def is_regulatorydomain_configurable(self):
        """
        Function used to know if it is possible to configure the Regulatory
        Domain for this Configuable Access Point

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
        self.get_logger().warning("set_regulatorydomain() is not " +
                                  "implemented in AP Equipment. Skip Regulatory Domain management.")
        return None

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF"
