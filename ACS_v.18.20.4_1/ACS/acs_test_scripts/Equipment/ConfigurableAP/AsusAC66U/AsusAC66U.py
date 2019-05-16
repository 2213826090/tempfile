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
:summary: implementation of ASUS AC66U family configurable AP
:since:04/11/2014
:author: sdebuiss, jfranchx
"""

from Equipment.IEquipment import EquipmentBase
from Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.ConfigurableAP.AsusAC66U.ControlAsusAC66U import ControlAsusRtAC66U


class AsusAC66U(EquipmentBase, IConfigurableAP):
    """
    Implementation of AsusAC66U configurable AP
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self._bench_params = bench_params
        self.__handle = None
        self._standard = None
        self._controlled_ap = None
        self._radio_status = "undef"

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
        return self._controlled_ap

    def _set_handle(self, handle):
        """
        Sets the connection handle of the equipment
        """
        self._controlled_ap = handle

    @staticmethod
    def _raise_exception(error_code="DefaultCode", error_msg="DefaultMsg"):
        """
        Function to raise exceptions from ControlAsusAC66U class

        :type error_code: str
        :param error_code: code of the error exception
        :type error_msg: str
        :param error_msg: message of the error
        """
        if error_code in [TestEquipmentException.OPERATION_FAILED, TestEquipmentException.INVALID_PARAMETER]:
            raise TestEquipmentException(error_code, error_msg)
        else:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

    # ------------------------------------------------------------------------------------------------------------------
    # Functions to convert parameters from ACS version to ASUS version
    def _convert_parameter_authentication(self, authentication_type, encryption_type=None):
        """
        Function to convert authentication type and encryption type from ACS to ASUS
        encryption_mode is not used in this implementation

        :type authentication_type: str
        :param authentication_type: ACS authentication type
        :type encryption_type: str
        :param encryption_type: ACS encryption type
        :rtype: (str, str)
        :return: ASUS authentication type, ASUS encryption mode
        """
        asus_encryption_mode = "AES"
        if authentication_type == "OPEN":
            asus_authentication = "Open System"
        elif authentication_type == "WPA-WPA2-PSK-TKIP-AES" or authentication_type == "WPA-WPA2-PSK-AES":
            asus_authentication = "WPA-Auto-Personal"
            if authentication_type == "WPA-WPA2-PSK-TKIP-AES":
                asus_encryption_mode = "TKIP+AES"
        elif authentication_type == "WPA-PSK-TKIP-AES" or authentication_type == "WPA-PSK-AES" or authentication_type == "WPA-PSK-TKIP":
            asus_authentication = "WPA-Personal"
            if authentication_type == "WPA-PSK-TKIP-AES":
                asus_encryption_mode = "TKIP+AES"
            elif authentication_type == "WPA-PSK-TKIP":
                asus_encryption_mode = "TKIP"
        elif authentication_type == "WPA2-PSK-AES":
            asus_authentication = "WPA2-Personal"
        elif authentication_type == "EAP-WPA":
            asus_authentication = "WPA-Enterprise"
        elif authentication_type == "EAP-WPA2":
            asus_authentication = "WPA2-Enterprise"
        else:
            self._raise_exception(TestEquipmentException.INVALID_PARAMETER,
                                  "Authentication %s not supported" % authentication_type)

        return asus_authentication, asus_encryption_mode

    def _convert_parameter_bandwidth(self, bandwidth):
        """
        Function to convert bandwidth from ACS to ASUS

        :type bandwidth: str
        :param bandwidth: ACS bandwidth
        :rtype: str
        :return: ASUS bandwidth
        """
        if bandwidth not in ['20', '40', '80']:
            self._raise_exception(TestEquipmentException.INVALID_PARAMETER, "Bandwidth %s unknown" % bandwidth)
        elif bandwidth == '20':
            return "20 MHz"
        elif bandwidth == '40':
            return "40 MHz"
        else:
            return "80 MHz"

    def _convert_parameter_hidden_ssid(self, hidden):
        """
        Function to convert hidden ssid parameter from ACS to radio for ASUS

        :type hidden: str
        :param hidden: ACS hidden ssid parameter
        :rtype: str
        :return: ASUS hidden ssid parameter
        """
        if isinstance(hidden, bool) is not True:
            self._raise_exception(TestEquipmentException.INVALID_PARAMETER, "Hidden SSID parameter is not boolean")
        elif hidden is True:
            return "ON"
        else:
            return "OFF"

    def _convert_parameter_standard_to_radio(self, standard):
        """
        Function to convert standard from ACS to radio for ASUS

        :type standard: str
        :param standard: ACS standard
        :rtype: str
        :return: ASUS radio
        """
        if standard not in ['a', 'b', 'g', 'n', 'an', 'bg', 'gb', 'bgn', 'ngb', 'n2.4G', 'n5G', 'ac']:
            self._raise_exception(TestEquipmentException.INVALID_PARAMETER,
                                  "Standard %s not supported by ASUS AP" % standard)
        elif standard in ['a', 'n5G', 'an', 'ac']:
            return "5GHz"
        else:
            return "2.4GHz"

    def _convert_parameter_wmm(self, wmm):
        """
        Function to convert wmm from ACS to radio for ASUS

        :type wmm: str or int
        :param wmm: ACS wmm
        :rtype: str
        :return: ASUS wmm
        """
        if wmm in ['on', '1', 1]:
            return 'Enable'
        elif wmm in ['off', '0', 0]:
            return 'Disable'
        else:
            self._raise_exception(TestEquipmentException.INVALID_PARAMETER, "Invalid parameter for wmm : " + str(wmm))

    # ------------------------------------------------------------------------------------------------------------------
    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        # Retrieve parameters from BenchConfig for connection
        host = str(self._bench_params.get_param_value("IP"))
        username = str(self._bench_params.get_param_value("username"))
        password = str(self._bench_params.get_param_value("password"))

        self._controlled_ap = ControlAsusRtAC66U()
        self._controlled_ap.configure_module(self._logger.debug,
                                             self._raise_exception,
                                             TestEquipmentException.INVALID_PARAMETER,
                                             TestEquipmentException.OPERATION_FAILED)

        self._controlled_ap.connect_to_asus(host, username, password)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        if self.get_handle() is None:
            self._logger.warning("AsusController not instanced, nothing to release")
            return

        self._controlled_ap.disconnect_from_asus()
        self._controlled_ap = None

    def create_ssid(self, ssid, hidden=False):
        """
        Create ssid to the equipment

        :type ssid: str
        :param ssid: SSID to create

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        hide = "OFF"
        if hidden is True:
            hide = "ON"
        self._controlled_ap.create_ssid(ssid, hide, "all")

    def load_config_file(self, kpi_test, srv_address=None):
        """
        load a configuration file to the AP.
        The file contains commands to set AP configuration.

        :type kpi_test: str
        :param kpi_test: name of the test to run. Used for config file retrieval in Benchconfig.

        :type srv_address: str
        :param srv_address: The ip address of the computer where the file is
        """
        self._logger.info("Starting Asus upload config file method")
        self.init()
        self._logger.info("Init ok")
        # Read config_file path from BenchConfig
        try:
            config_file = self._bench_params._BenchConfigParameters__dict["Config_file_list"][kpi_test]["value"]
        except Exception as exception:
            msg = "Unable to find AP config file in BenchConfig for this test : %s" % str(exception)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # Make UI actions to upload the configuration file to the AP
        self._controlled_ap.load_config_file(config_file)
        self._logger.info("File should be uploaded !")

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
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        # Set default value for some parameters
        asus_radio_state = "Enable"
        self._radio_status = "ON"
        asus_tx_value = 100

        asus_authentication, asus_encryption_mode = self._convert_parameter_authentication(authentication_type)
        asus_bandwidth = self._convert_parameter_bandwidth(bandwidth)
        asus_hide = self._convert_parameter_hidden_ssid(hidden)
        asus_radio = self._convert_parameter_standard_to_radio(standard_type)
        asus_wmm = self._convert_parameter_wmm(wmm)

        self._controlled_ap.set_full_config(radio=asus_radio, radio_state=asus_radio_state, ssid=ssid, hide=asus_hide,
                                            standard=standard_type, bandwidth=asus_bandwidth, channel=channel,
                                            extension_channel=None, authentication_type=asus_authentication,
                                            encryption_mode=None, passphrase=passphrase, radius_ip=radiusip,
                                            radius_port=radiusport, radius_secret=radiussecret, beacon=beacon,
                                            dtim=dtim, wmm=asus_wmm, tx_value=asus_tx_value)

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        Set wifi standard

        :type standard: String
        :param standard: Wifi standard to set
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        self._controlled_ap.set_wifi_standard(standard)

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
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_authentication, asus_encryption_mode = self._convert_parameter_authentication(authentication_type)
        asus_radio = self._convert_parameter_standard_to_radio(standard_type)

        self._controlled_ap.set_wifi_authentication(asus_authentication, passphrase, asus_radio, asus_encryption_mode)

        if radiusip is not None and radiusport is not None and radiussecret is not None:
            self._controlled_ap.set_page_radius(radiusip, radiusport, radiussecret, asus_radio)
        else:
            if asus_authentication in ["WPA-Enterprise", "WPA2-Enterprise"]:
                self._logger.warning(TestEquipmentException.INVALID_PARAMETER,
                                     "No radius server configured on the AP for EAP security !")

    def set_wifi_channel(self, standard, channel):
        """
        Set wifi channel

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_radio = self._convert_parameter_standard_to_radio(standard)
        self._controlled_ap.set_wifi_channel(asus_radio, channel)

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim:
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        self._controlled_ap.set_wifi_dtim(dtim, "all")

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        self._controlled_ap.set_wifi_beacon(beacon, "all")

    def set_wifi_wmm(self, mode):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_wmm = self._convert_parameter_wmm(mode)
        self._controlled_ap.set_wifi_wmm(asus_wmm, "all")

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
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_radio = self._convert_parameter_standard_to_radio(standard)
        asus_bandwidth = self._convert_parameter_bandwidth(bandwidth)
        self._controlled_ap.set_wifi_bandwidth(asus_bandwidth, asus_radio)

    def set_wifi_power(self, standard, power):
        """
        Set wifi transmit power in dBm

        :type standard: str
        :param standard: The wifi standard used to control the validity of the power value
        :type power: str or int
        :param power: wifi transmit power. Range is from 1 to 100.
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_radio = self._convert_parameter_standard_to_radio(standard)
        self._controlled_ap.set_wifi_tx_power_adjustment(str(power), asus_radio)

    def enable_wireless(self, standard=None):
        """
        enable wireless network

        :type standard: str
        :param standard: type of frequency to enable

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_radio = self._convert_parameter_standard_to_radio(standard)
        self._controlled_ap.enable_wireless(asus_radio)
        self._radio_status = "ON"

        return True

    def disable_wireless(self):
        """
        disable wireless network

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        self._controlled_ap.disable_wireless("all")
        self._radio_status = "OFF"

        return True

    def set_acl_mode(self, mode):
        """
        set access control list mode

        :type mode: str
        :param mode: acl mode to set ("disable", "enable")
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        self._controlled_ap.set_acl_mode(mode)

    def add_mac_address_to_acl(self, mac_addr):
        """
        add mac address to acl list

        :type mac_addr: str
        :param mac_addr: mac_addr mode to filter
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def del_mac_address_from_acl(self, mac_addr):
        """
        delete mac address from acl list

        :type mac_addr: str
        :param mac_addr: mac_addr mode to remove from filter
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dhcp(self, dhcp_status,
                 low_excluded_ip=None,
                 high_excluded_ip=None,
                 subnet=None,
                 subnet_mask=None,
                 lease=None,
                 default_gateway_address=None):
        """
        set dhcp server on the Configurable AP

        :type dhcp_status: str or int
        :param dhcp_status: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :type low_excluded_ip: str
        :param low_excluded_ip: An ip address "x.x.x.x" corresponding to the lower end of the excluded ip range

        :type high_excluded_ip: str
        :param high_excluded_ip: An ip address "x.x.x.x" corresponding to the higher end of the excluded ip range

        :type subnet: str
        :param subnet: An ip address "x.x.x.x" corresponding to the subnet address to be masked

        :type subnet_mask: str
        :param subnet_mask: An ip mask "x.x.x.x"

        :type lease: str
        :param lease:  { days [ hours ] [ minutes ] | infinite }, only numbers and spaces or "infinite"

        :type default_gateway_address: str
        :param default_gateway_address: An ip address "x.x.x.x" corresponding to the router address
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_regulatorydomain_configurable(self):
        """
        Function used to know if it is possible to configure the Regulatory
        Domain for this Configuable Access Point
        If True, the function set_regulatorydomain() must be implemented
        If False, the function get_regulatorydomain() should be implemented

        :rtype: boolean
        :return: True if the Regulatory Domain is configurable
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_regulatorydomain_sent(self):
        """
        Function used to know if the Access Point automatically sends the
        Regulatory Domain to their clients.

        :rtype: boolean
        :return: True if the AP automatically sends the regulatory domain
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_regulatorydomain(self):
        """
        Return current regulatory domain for the Access Point

        :rtype: String
        :return: The current regulatory domain set
        """

    def get_beacon_period(self):
        """
        Return latest current beacon period set

        :rtype: String
        :return: The latest current beacon period set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_regulatorydomain(self, reg_domain):
        """
        Set the regulatory domain to the Access Point

        :type reg_domain: String
        :param reg_domain: The current regulatory domain to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_key_exchange(self, key_exchange_type, pin=None):
        """
        Set the key exchange type on the equipment

        :type key_exchange_type: String
        :param key_exchange_type: Key exchange type supported by the equipment

        :type pin: int
        :param passphrase: pin code used for some key exchange(optional)

        :rtype: String
        :return: The pin code in WPS_PIN_FROM_AP (LABEL mode) if pin is None
                  pin in WPS_PIN_FROM_DUT (PIN enrolee mode) if pin is not None
                  "OK" in WPS_PBC mode
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_wps_pin(self):
        """
        Gets the WPS pin code from the access point

        :rtype: int
        :return: The AP's WPS PIN code.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF"

    def simulate_dfs(self, from_freq, to_freq, nbr_beacon_to_wait, dfs_mode="enable"):
        """
        Set AP DFS simulation from a channel to another

        :type from_freq: str
        :param from_freq: The current channel you want to switch from

        :type to_freq: str
        :param to_freq: The channel you want to switch to

        :type nbr_beacon_to_wait: str
        :param nbr_beacon_to_wait: The delay in number of beacon before do the switch

        :type dfs_mode: str
        :param dfs_mode: the dfs mode set ap into
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_client_macaddr_list(self):
        """
        Get the Client MAC addresses list.
        The MAC Addresses returned are in lower case.
        The format of MAC addresses is: xx:xx:xx:xx:xx:xx where x is 0-9 or a-f.

        :rtype: list
        :return: list of MAC addresses of the clients connected to the Access Point
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_selected_channel(self, standard=None):
        """
        Get the selected WiFi channel

        :type standard: str
        :param standard: the standard used on the AP.

        :rtype: int
        :return: the selected channel number
        """
        if self.get_handle() is None:
            self._raise_exception(TestEquipmentException.OPERATION_FAILED, "AsusController not instanced")

        asus_radio = self._convert_parameter_standard_to_radio(standard)
        current_channel = self._controlled_ap.get_selected_channel(asus_radio)
        if current_channel == "Auto":
            self._logger.warning("Current channel on %s is configured with Auto mode !" % asus_radio)
            return current_channel
        return int(current_channel)
