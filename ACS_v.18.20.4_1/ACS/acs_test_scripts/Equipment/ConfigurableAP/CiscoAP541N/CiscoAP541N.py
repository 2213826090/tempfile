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
:summary: This file implements the Cisco AP541N router configurator
:since:03/03/2012
:author: jpstierlin
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiAuthenticationTypes
from acs_test_scripts.Equipment.ConfigurableAP.CiscoAP541N.WifiConfigurationParser import WifiConfigurationParser

from ErrorHandling.TestEquipmentException import TestEquipmentException


class CiscoAP541N(EquipmentBase, IConfigurableAP):

    """
    Cisco CiscoAP541N router configurator
    """

    # Define specific list for the current equipment
    SUPPORTED_WIFI_STANDARD = {'a': 'a', 'bg': 'bg', 'gb': 'bg', 'bgn': 'bg-n', 'ngb': 'bg-n', 'n': 'n-only-g', 'n2.4G': 'n-only-g', 'an': 'a-n', 'n5G': 'n-only-a'}
    SUPPORTED_WIFI_AUTHENTICATION = ['OPEN', 'WEP64', 'WEP128', 'WPA-PSK-TKIP', 'WPA-PSK-AES', 'WPA2-PSK-AES', 'WPA2-PSK-TKIP', 'WPA2-PSK-TKIP-AES']
    SUPPORTED_CHANNELS5G = ['', '36', '40', '44', '48', '52', '56', '60', '64', '100', '104', '108', '112', '116', '132', '136', '140']
    BASIC_RATES_2G = ['11', '5.5', '2', '1']
    SUPPORTED_RATES_2G = ['54', '48', '36', '24', '18', '12', '11', '9', '6', '5.5', '2', '1']
    BASIC_RATES_5G = ['24', '12', '6']
    SUPPORTED_RATES_5G = ['54', '48', '36', '24', '18', '12', '9', '6']

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None
        self._dom = None
        self._changed = False
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
        self.get_handle().init_configuration()

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
        connect CiscoAP541N via http to configure the access point

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
            # In this case the handle represents the config file to update
            handle = WifiConfigurationParser(self.get_logger(), host, user, password)
            handle.download_configuration()
            handle.init_configuration()

            self._set_handle(handle)

        except:
            msg = "Http connection failed."
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

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
        if self.get_handle() is not None:
            self.get_handle().logout()
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
        self.get_handle().setinterfacevalue('ssid', ssid)
        if hidden:
            self.get_handle().setbssvalue('ignore-broadcast-ssid', 'on')
        else:
            self.get_handle().setbssvalue('ignore-broadcast-ssid', 'off')

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

        :type radiusip: String
        :param radiusip: IP address of the radius server

        :type radiusport: integer
        :param radiusport: Port number of the radius server

        :type radiussecret: String
        :param radiussecret: Shared secret used for radius authentication

        :type standard_type: str
        :param standard_type: The wifi standard used to determine if the \
                            channel has to be set for 2.4GHz or 5GHz

        :raise: TestEquipmentException
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

    def __set_wifi_authentication_OPEN(self):
        """
        set CiscoAP541N AP Encryption type: Open
        """
        self.get_logger().debug("Set wifi authentication to OPEN")

        self.get_handle().setinterfacevalue('security', 'plain-text')

    def __set_wifi_authentication_WEP64(self, passphrase):
        """
        Set AP Encryption type to WEP64

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 64 bits")

        self.get_handle().setinterfacevalue('security', 'static-wep')
        self.get_handle().setinterfacevalue('wep-default-key', '1')
        self.get_handle().setinterfacevalue('wep-key-length', '40')
        self.get_handle().setinterfacevalue('wep-key-ascii', 'yes')
        self.get_handle().setinterfacevalue('wep-key-1', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'on')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'off')
        self.get_handle().setbssvalue('wpa2-allowed', 'off')

    def __set_wifi_authentication_WEP128(self, passphrase):
        """
        Set AP Encryption type to WEP128

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 128 bits")

        self.get_handle().setinterfacevalue('security', 'static-wep')
        self.get_handle().setinterfacevalue('wep-default-key', '1')
        self.get_handle().setinterfacevalue('wep-key-length', '104')
        self.get_handle().setinterfacevalue('wep-key-ascii', 'yes')
        self.get_handle().setinterfacevalue('wep-key-1', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'on')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'off')
        self.get_handle().setbssvalue('wpa2-allowed', 'off')

    def __set_wifi_authentication_WPA_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP")

        self.get_handle().setinterfacevalue('security', 'wpa-personal')
        self.get_handle().setinterfacevalue('wpa-personal-key', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'off')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'on')
        self.get_handle().setbssvalue('wpa2-allowed', 'off')
        self.get_handle().setbssvalue('wpa-cipher-tkip', 'on')
        self.get_handle().setbssvalue('wpa-cipher-ccmp', 'off')

    def __set_wifi_authentication_WPA_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK AES")

        self.get_handle().setinterfacevalue('security', 'wpa-personal')
        self.get_handle().setinterfacevalue('wpa-personal-key', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'off')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'on')
        self.get_handle().setbssvalue('wpa2-allowed', 'off')
        self.get_handle().setbssvalue('wpa-cipher-tkip', 'off')
        self.get_handle().setbssvalue('wpa-cipher-ccmp', 'on')

    def __set_wifi_authentication_WPA2_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK AES")

        self.get_handle().setinterfacevalue('security', 'wpa-personal')
        self.get_handle().setinterfacevalue('wpa-personal-key', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'off')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'off')
        self.get_handle().setbssvalue('wpa2-allowed', 'on')
        self.get_handle().setbssvalue('wpa-cipher-tkip', 'off')
        self.get_handle().setbssvalue('wpa-cipher-ccmp', 'on')

    def __set_wifi_authentication_WPA2_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP")

        self.get_handle().setinterfacevalue('security', 'wpa-personal')
        self.get_handle().setinterfacevalue('wpa-personal-key', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'off')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'off')
        self.get_handle().setbssvalue('wpa2-allowed', 'on')
        self.get_handle().setbssvalue('wpa-cipher-tkip', 'on')
        self.get_handle().setbssvalue('wpa-cipher-ccmp', 'off')

    def __set_wifi_authentication_WPA2_PSK_TKIP_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP AES")

        self.get_handle().setinterfacevalue('security', 'wpa-personal')
        self.get_handle().setinterfacevalue('wpa-personal-key', passphrase)
        self.get_handle().setbssvalue('open-system-authentication', 'off')
        self.get_handle().setbssvalue('shared-key-authentication', 'off')
        self.get_handle().setbssvalue('wpa-allowed', 'off')
        self.get_handle().setbssvalue('wpa2-allowed', 'on')
        self.get_handle().setbssvalue('wpa-cipher-tkip', 'on')
        self.get_handle().setbssvalue('wpa-cipher-ccmp', 'on')

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        set wifi standard: g,n,bg,bgn,a,an

        :type standard: String
        :param standard: a,bg,bgn,n,an,n5G
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        self.get_logger().info("Set wifi standard to '%s'" % str(standard))

        if enable_mimo:
            self._logger.warning("MIMO is not available on this Access Point")

        self.get_handle().setradiovalue('mode', self.SUPPORTED_WIFI_STANDARD[standard])
        if standard not in ['a', 'an', 'n5G']:
            basic_rates = self.BASIC_RATES_2G
            supported_rates = self.SUPPORTED_RATES_2G
        else:
            basic_rates = self.BASIC_RATES_5G
            supported_rates = self.SUPPORTED_RATES_5G
        self.get_handle().setrates('basic-rate', basic_rates)
        self.get_handle().setrates('supported-rate', supported_rates)

    def set_wifi_channel(self, standard, channel):
        """
        set wifi channel:

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz

        :type channel: integer or Digit-String
        :param channel: The wifi channel to set, 2.4GHz:1-14(0:auto), 5GHz:36-165(0:auto)
        """
        channel = str(channel)
        if int(channel) == 0:
            self.get_logger().info("Set wifi channel to 0")
            self.get_handle().setradiovalue('channel-policy', 'best')
        elif not standard in ['a', 'an', 'n5G']:
            # 2.4G
            self.get_logger().info("Set wifi channel to '%s'" % channel)
            self.get_handle().setradiovalue('channel-policy', 'static')
            try:
                self.get_handle().setradiovalue('static-channel', channel)
            except IndexError:
                self.get_handle().addradiovalue('static-channel', channel)
        else:
            # 5G
            if standard in self.SUPPORTED_CHANNELS5G:
                self.get_logger().info("Set wifi channel to '%s'" % channel)
                self.get_handle().setradiovalue('channel-policy', 'static')
                try:
                    self.get_handle().setradiovalue('static-channel', channel)
                except IndexError:
                    self.get_handle().addradiovalue('static-channel', channel)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "channel is out of range.")

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim: Delivery traffic indication message interval
        """
        self.get_logger().info("Set wifi DTIM to '%s'" % str(dtim))
        self.get_handle().setbssvalue('dtim-period', str(dtim))

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        self.get_logger().info("Set wifi beacon to '%s'" % str(beacon))
        self.get_handle().setradiovalue('beacon-interval', str(beacon))

    def set_wifi_wmm(self, wmm):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :raise: TestEquipmentException
        """
        if wmm in ("on", "1", 1):
            self._logger.info("Set wifi wmm to on")
            mode = 'on'
        elif wmm in ("off", "0", 0):
            self._logger.info("Set wifi wmm to off")
            mode = 'off'
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Parameter wmm is not valid !")
        self.get_handle().setradiovalue('wme', mode)

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

        if bandwidth != '20' and bandwidth != '40':
            msg = "Unsupported bandwidth '%s'" % bandwidth
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if (standard not in ['n', 'n2.4G', 'n5G']) and (int(bandwidth) == 40):
            msg = "Trying to set 40Mhz bandwitdh with a non N standard."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_handle().setradiovalue('n-bandwidth', bandwidth)

    def set_wifi_power(self, standard, power):
        """
        Set wifi transmit power in dBm

        :type standard: str
        :param standard: The wifi standard used to control the validity of \
                         the power value
        :type power: str or int
        :param power: wifi transmit power:
            2.4GHz: -1, 2, 5, 8, 11, 14, 17, 20, max
            5GHz:   -1, 2, 5, 8, 11, 14, 17, max
        """
        msg = "set_wifi_power not available for CiscoAP541N"
        raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

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

        :type radiusip: String
        :param radiusip: IP address of the radius server (optional)

        :type radiusport: integer
        :param radiusport: Port number of the radius server (optional)

        :type radiussecret: String
        :param radiussecret: Shared secret used for radius authentication (optional)

        :type configindex: integer
        :param configindex: Configuration index - can be 0 to 3 - 0 is the default SSID data (optional)

        :raise: TestEquipmentException
        """
        self.get_logger().info("Set wifi configuration")

        if configindex != 0:
            msg = "Configuration index can be only zero : %s not correct" % configindex
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # Extract time to wait for configuration of the equipment
        configuration_timer = float(self.__bench_params.
                                    get_param_value("ConfigurationWaitingTime"))

        if standard_type not in self.SUPPORTED_WIFI_STANDARD.keys():
            msg = "wifi standard type: %s not correct" % standard_type
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        if authentication_type not in self.SUPPORTED_WIFI_AUTHENTICATION:
            msg = "wifi authentication type: %s not correct" % authentication_type
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        if not self._is_supported_config(standard_type, authentication_type):
            msg = "wifi standard type: %s with authentication type: %s not supported"\
                % (standard_type, authentication_type)
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # disable multiple ssids
        self._delete_ssids()

        # Create ssid
        self.create_ssid(ssid, hidden)

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

        # Send the configuration to the equipment
        self.get_handle().upload_configuration(configuration_timer)

    def enable_wireless(self):
        """
        enable wireless
        """
        self.get_logger().info("Enable wireless")

        if self._radio_status != "ON":
            self.get_handle().logout()
            self.get_handle().login()
            self.get_handle().setradiovalue('status', 'up')
            configuration_timer = float(self.__bench_params.
                                        get_param_value("ConfigurationWaitingTime"))
            self.get_handle().upload_configuration(configuration_timer)
            self._radio_status = "ON"
        else:
            self._logger.info("Wireless already enabled")

    def disable_wireless(self):
        """
        disable wireless
        """
        self.get_logger().info("Disable wireless")

        if not self.is_radio_off():
            self.get_handle().logout()
            self.get_handle().login()
            self.get_handle().setradiovalue('status', 'down')
            configuration_timer = float(self.__bench_params.
                                        get_param_value("ConfigurationWaitingTime"))
            self.get_handle().upload_configuration(configuration_timer)
            self._radio_status = "OFF"
        else:
            self._logger.info("Wireless already disabled")

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
