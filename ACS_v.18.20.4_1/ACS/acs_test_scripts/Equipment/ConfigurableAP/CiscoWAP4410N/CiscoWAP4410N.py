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
:summary: implementation of WAP4410N configurable AP
:since:26/10/2011
:author: jpstierlin
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiAuthenticationTypes, build_wep_key_64, build_wep_key_128
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from acs_test_scripts.Equipment.ConfigurableAP.CiscoWAP4410N.WifiConfigurationParser import WifiConfigurationParser

from ErrorHandling.TestEquipmentException import TestEquipmentException

import urllib
import base64


class CiscoWAP4410N(EquipmentBase, IConfigurableAP):

    """
    Implementation of WAP4410N configurable AP
    """

    # Define specific list for the current equipment
    SUPPORTED_WIFI_STANDARD = {'b': '1', 'g': '2', 'n': '3', 'bg': '4', 'gb': '4', 'bgn': '7', 'ngb': '7', 'off': '0'}
    SUPPORTED_WIFI_AUTHENTICATION = ['OPEN', 'WEP64', 'WEP128', 'WPA-PSK-TKIP', 'WPA-PSK-AES', 'WPA2-PSK-AES', 'WPA2-PSK-TKIP', 'WPA2-PSK-TKIP-AES']
    SUPPORTED_MAC_FILTERS = {'enabled': '1', 'radius': '2', 'disabled': '0'}

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None
        self._set_handle(None)
        self._standard = None
        self._profile = '1'
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

    def _delete_ssids(self):
        """
        Delete all ssids from the Equipment
        """
        for profile in range(1, 5):
            self.get_handle().set_wb_value('SSID' + str(profile), '')

    def _set_macfilter(self, macfilter):
        """
        Set mac filter

        :type macfilter : String
        :param macfilter: Enable/disable or radius
        """
        if not macfilter in CiscoWAP4410N.SUPPORTED_MAC_FILTERS.keys():
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Invalid macfilter value: {0}".format(macfilter))
        self.get_handle().set_value('security_mode', CiscoWAP4410N.SUPPORTED_MAC_FILTERS[macfilter])

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
        connect WAP4410N via http to configure the access point

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
            login = urllib.quote(base64.urlsafe_b64encode(user))
            passwd = urllib.quote(base64.urlsafe_b64encode(password))
            # In this case the handle represents the config file to update
            handle = WifiConfigurationParser(self.get_logger(), host, login, passwd)
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
        self.get_handle().set_ws_value('security_mode', '0')

    def __set_wifi_authentication_WEP64(self, passphrase):
        """
        Set AP Encryption type to WEP64

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 64 bits")

        self.get_handle().set_ws_value('security_mode', '1')
        self.get_handle().set_ws_value('encryption', '64')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('authentication_type', '0')  # 0:Open System 1:Shared Key
        self.get_handle().set_ws_value('Default_Tx_key', '1')  # 1..4
        keys = build_wep_key_64(passphrase)
        for index in range(4):
            self.get_handle().set_ws_value('key' + str(index + 1), keys[index])

    def __set_wifi_authentication_WEP128(self, passphrase):
        """
        Set AP Encryption type to WEP128

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 128 bits")

        self.get_handle().set_ws_value('security_mode', '1')
        self.get_handle().set_ws_value('encryption', '128')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('authentication_type', '0')  # 0:Open System 1:Shared Key
        self.get_handle().set_ws_value('Default_Tx_key', '1')  # 1..4
        keys = build_wep_key_128(passphrase)
        for index in range(4):
            self.get_handle().set_ws_value('key' + str(index + 1), keys[0])

    def __set_wifi_authentication_WPA_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP")

        self.get_handle().set_ws_value('security_mode', '2')
        self.get_handle().set_ws_value('encryption', '1')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('PSK_key', passphrase)

    def __set_wifi_authentication_WPA_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK AES")

        self.get_handle().set_ws_value('security_mode', '2')
        self.get_handle().set_ws_value('encryption', '2')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('PSK_key', passphrase)

    def __set_wifi_authentication_WPA2_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK AES")

        self.get_handle().set_ws_value('security_mode', '3')
        self.get_handle().set_ws_value('encryption', '2')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('PSK_key', passphrase)

    def __set_wifi_authentication_WPA2_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP")

        self.get_handle().set_ws_value('security_mode', '3')
        self.get_handle().set_ws_value('encryption', '1')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('PSK_key', passphrase)

    def __set_wifi_authentication_WPA2_PSK_TKIP_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP AES")

        self.get_handle().set_ws_value('security_mode', '3')
        self.get_handle().set_ws_value('encryption', '3')  # 64: 64 bit WEP, 128:128 bit WEP, 1: TKP, 2: AES, 3: AES+TKIP
        self.get_handle().set_ws_value('PSK_key', passphrase)

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
        self.get_logger().info("Create ssid '%s'" % str(ssid))
        self.get_handle().set_wb_value('SSID' + str(self._profile), ssid)
        self.get_handle().set_ws_value('ssid', ssid)
        if hidden:
            self.get_handle().set_wb_value('SSID' + str(self._profile) + '_broadcast', '0')
        else:
            self.get_handle().set_wb_value('SSID' + str(self._profile) + '_broadcast', '1')

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
        self.get_logger().info("Set wifi standard to '%s'" % str(standard))

        if enable_mimo:
            self._logger.warning("MIMO is not available on this Access Point")

        if not standard in CiscoWAP4410N.SUPPORTED_WIFI_STANDARD.keys():
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Invalid wifi standard type value: {0}".format(standard))

        self.get_handle().set_wb_value('Network_mode', CiscoWAP4410N.SUPPORTED_WIFI_STANDARD[standard])

        if standard != "off":
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

        if int(channel) < 0 or int(channel) >= 15:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "invalid channel number: {0}".format(channel))

        self.get_handle().set_wb_value('Channel', channel)

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM (Delivery Traffic Indication Message)

        :type dtim: int
        :param dtim: The DTIM to set.
        """
        self.get_logger().info("Set wifi DTIM to '%s'" % str(dtim))
        self.get_handle().set_wa_value('DTIM_interval', dtim)

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        self.get_logger().info("Set wifi beacon to '%s'" % str(beacon))
        self.get_handle().set_wa_value('beacon_interval', beacon)

    def set_wifi_wmm(self, mode):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        if mode in ("on", "1", 1):
            self.get_logger().info("Set wifi wmm to on")
            mode = 1
        elif mode in ("off", "0", 0):
            self.get_logger().info("Set wifi wmm to off")
            mode = 0
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Parameter mode is not valid !")

        self.get_handle().set_qos_value('WMM_4_SSID' + str(self._profile), mode)

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
            value = '1'
        elif bandwidth == "40":
            value = '2'
        else:
            msg = "Unsupported bandwidth '%s'" % bandwidth
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if (standard not in ['n', 'n2.4G', 'n5G']) and (int(bandwidth) == 40):
            msg = "Trying to set 40Mhz bandwitdh with a non N standard."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_handle().set_wa_value('Channel_bandwidth', value)

    def set_wifi_power(self, standard, wifi_power):
        """
        Set wifi transmit power in dBm

        :type standard: str
        :param standard: The wifi standard used to control the validity of \
                         the power value
        :type power: str or int
        :param power: wifi transmit power:
            2.4GHz: POWER_VALUES_2G
            5GHz:   POWER_VALUES_5G
        """
        # Nothing to do, just pass
        pass

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

        if standard_type not in CiscoWAP4410N.SUPPORTED_WIFI_STANDARD.keys():
            msg = "wifi standard type: %s not correct" % standard_type
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        if authentication_type not in CiscoWAP4410N.SUPPORTED_WIFI_AUTHENTICATION:
            msg = "wifi authentication type: %s not correct" % authentication_type
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        if not self._is_supported_config(standard_type, authentication_type):
            msg = "wifi standard type: %s with authentication type: %s not supported" % (standard_type, authentication_type)
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # disable multiple ssids
        self._delete_ssids()
        # always work on profile 1
        self._profile = 1
        # Create ssid
        self.create_ssid(ssid, hidden)

        # disable MAC address filtering
        self._set_macfilter('disabled')

        # Set the wifi standard
        self.set_wifi_standard(standard_type, mimo)

        # Set the authentication
        self.set_wifi_authentication(authentication_type, passphrase)

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
            # Enable/disable the wifi wmm if exists
            self.set_wifi_wmm(wmm)
        if bandwidth is not None:
            # Set the wifi bandwidth if exists
            self.set_wifi_bandwidth(bandwidth, standard_type)
        self._logger.info("Applying configuration...")
        # Send the configuration to the equipment
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
            self.set_wifi_standard(self._standard)
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
            self.set_wifi_standard('off')
            configuration_timer = float(self.__bench_params.
                                        get_param_value("ConfigurationWaitingTime"))
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

    def set_wifi_key_exchange(self, key_exchange_type, pin=None):
        """
        Set the key exchange type on the equipment

        :type key_exchange_type: WifiKeyExchangeTypes
        :param key_exchange_type: Key exchange type supported by the equipment

        :type pin: int
        :param passphrase: pin code used for some key exchange(optional)

        :rtype: String
        :return: The pin code in WPS_PIN_FROM_AP (LABEL mode) if pin is None
                  pin in WPS_PIN_FROM_DUT (PIN enrolee mode) if pin is not None
                  "OK" in WPS_PBC mode
        """

        self.get_logger().info("Set wifi key exchange to '%s'" % str(key_exchange_type))

        if key_exchange_type == WifiKeyExchangeTypes.WPS_PBC:
            self.__set_wifi_key_exchange_WPS_PBC()
        elif key_exchange_type == WifiKeyExchangeTypes.WPS_PIN_FROM_AP:
            self.__set_wifi_key_exchange_WPS_PIN()
        elif key_exchange_type == WifiKeyExchangeTypes.WPS_PIN_FROM_DUT:
            self.__set_wifi_key_exchange_WPS_PIN(pin)
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi key exchange type '%s'" % str(key_exchange_type))

    def get_wifi_wps_pin(self):
        """
        Gets the WPS pin code from the access point

        :rtype: int
        :return: The AP's WPS PIN code.
        """
        cfg_handle = self.get_handle()
        if cfg_handle:
            return cfg_handle.get_wps_pin()
        else:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Can't get WPS PIN")

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF"

    def __set_wifi_key_exchange_WPS_PBC(self):
        """
        Induce a software push button to enable the WPS feature.
        """
        handle = self.get_handle()

        # Parse the page to get the push button
        handle.do_wps_pbc()

    def __set_wifi_key_exchange_WPS_PIN(self, pin=None):
        """
        Sets the WPS pin. Either by setting the DUT pin into AP (pin variable set) or by
        getting AP's one and giving it to the DUT (pin = None)

        :type pin: int or str with digits only
        :param pin: WPS pin to set (optional)
        """

        handle = self.get_handle()

        # Set the DUT pin into AP
        if pin:
            if str(pin).isdigit():
                # Set the new pin into the access point
                handle.set_wps_pin(str(pin))
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "invalid pin.")

        else:
            pass
