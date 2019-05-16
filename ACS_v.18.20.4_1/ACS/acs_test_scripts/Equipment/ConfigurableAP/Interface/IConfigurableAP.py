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
:summary: virtual interface with configurable Access Point
:since:19/01/2012
:author: ssavrimoutou
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IConfigurableAP(object):

    """
    Virtual interface for configurable Access Points
    """

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def create_ssid(self, ssid, hidden=False):
        """
        create ssid on the equipment

        :type ssid: str
        :param ssid: SSID to create

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_config_file(self, kpi_test, srv_address=None):
        """
        load a configuration file to the AP.
        The file contains commands to set AP configuration.

        :type kpi_test: str
        :param kpi_test: name of the test to run. Used for config file retrieval in Benchconfig.

        :type srv_address: str
        :param srv_address: The ip address of the computer where the file is
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def clean_wifi_config(self):
        """
        Clear all WiFi configurations and turn Off radio(s)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        Set wifi standard

        :type standard: String
        :param standard: Wifi standard to set
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_channel(self, standard, channel):
        """
        Set wifi channel

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz
        :type channel: integer or Digit-String
        :param channel: The wifi channel to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim:
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_wmm(self, mode):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_wireless(self, standard=None):
        """
        enable wireless network

        :type standard: str
        :param standard: type of frequency to enable

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disable_wireless(self):
        """
        disable wireless network

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_acl_mode(self, mode):
        """
        set access control list mode

        :type mode: str
        :param mode: acl mode to set ("disable", "enable")
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
