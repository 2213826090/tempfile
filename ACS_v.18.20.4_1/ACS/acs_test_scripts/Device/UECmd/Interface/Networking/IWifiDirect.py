"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: UMG PSI Validation
:summary: This file implements the interface for Wifi Direct specific methods
:since: 2014-07-21
:author: emarchan

"""
from ErrorHandling.DeviceException import DeviceException
from abc import ABCMeta


class IWifiDirect(object):
    """
    Abstract class that defines the interface to be implemented
    by network handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    __metaclass__ = ABCMeta

    def open_wifi_direct_menu_settings(self):
        """
        Display the Wifi direct settings page
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_direct_dev_name(self, name=""):
        """
        Sets the name of the device seen in wifi direct

        :type name: String
        :param name: The name you want to set
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_scan_peer_devices(self):
        """
        initiates a wifi direct discovery of the peers (scan)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_remove_remembered_groups(self):
        """
        Removes Wifi direct remembered groups
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_peer_devices(self):
        """
        Gets the list of wifi direct peer devices scanned
        :rtype: List of String
        :return name: list of wifi direct peer devices scanned
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_dev_address_from_name(self, wifi_direct_name=""):
        """
        Gets a Wifi Direct device address from its name.

        :type wifi_direct_name: String
        :param wifi_direct_name: The name of the peer device you want the address of.
        :rtype: string
        :rparam: The peer device address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_connect(self, peer_device_address="", timeout=20, go_intent=8, force_freq=0, connect_type="",
                            authentication_type="", pin=""):
        """
        Connect to a wifi direct peer device.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type timeout: int
        :param timeout: Connection timeout.
        :type go_intent: int
        :param go_intent: Group owner intent.
        :type force_freq: int
        :param force_freq: Frequency used for the communication (forced)
        :type connect_type: string
        :param connect_type: Type of connection (auto - connecting to an autonomous group
                                            intent - connecting using go_intent parameter set to 0)
        :type authentication_type: string
        :param authentication_type: Type of authentication  (WPS-PBC - connecting using PushButton
                                                   WPS-PIN - connecting using PIN code)
        :type pin_code: string
        :param pin_code: Pin code used when auth_type is WPS-PIN        .
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_accept_connection(self, incoming_peer=""):
        """
        Grants a connection request from a peer device.

        :type incoming_peer: string
        :param incoming_peer: MAC address of the peer you want
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_disconnect(self, wifi_direct_name=""):
        """
        Disconnect from a wifi direct peer device.

        :type wifi_direct_name: String
        :param wifi_direct_name: The address of the device you want to disconnect from.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_remembered_groups(self):
        """
        Gets the Wifi direct remembered groups.
        :rtype: List of String
        :return: list of wifi direct remembered groups.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_wifi_direct_groups(self, group_owner="", groups_list=""):
        """
        Checks if the name of the group (regarding the group owner) is included in a list of groups.

        :type group_owner: String
        :param group_owner: Name of the peer device that initiated the group creation.
        :type groups_list: String
        :param groups_list: List of group names you check the name of. Separator is |

        :rtype: Boolean
        :return True if the group owner's group is in the list. False else
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_interface(self):
        """
        Gets the WiFi direct interface.
        :rtype: str
        :return: WiFi direct interface
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_direct_frequency(self, wifi_direct_frequency):
        """
        Sets the WiFi direct frequency.
        :type wifi_direct_frequency: int
        :param wifi_direct_frequency: Frequency to force. Set a valid WiFi frequency in MHz.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_direct_mode(self, wifi_direct_mode):
        """
        Sets the WiFi direct mode (CLI/GO).
        :type wifi_direct_mode: String
        :param wifi_direct_mode: Mode to set. Possible values : GO or CLI.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_mode(self, wifi_direct_interface):
        """
        Gets the WiFi direct mode (CLI/GO).
        :type wifi_direct_interface: String
        :param wifi_direct_interface: Name of the WiFi Direct interface.
        :rtype: str
        :return: WiFi Direct mode
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_p2p(self):
        """
        Specific Edison

        Start WiFi Direct on dut.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_flush(self):
        """
        Specific Edison

        Removes Wifi direct connections
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_mac_address(self):
        """
        Gets the device Wifi Direct mac address.

        :rtype: String
        :return mac_addr: the device Wifi Direct mac address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_nw_config(self):
        """
        Specific Edison

        Configures the P2P network interface
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_discover(self):
        """
        Specific Edison

        Start P2P device discovery
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_create_group(self, group_freq="", auth_type=""):
        """
        Specific Edison

        Creates a Autonomous Group, forcing the DUT to be GO.

        :type group_freq: string
        :param group_freq: Frequency of the Autonomous Group
        :type auth_type: string
        :param auth_type: Type of authentication  (WPS-PBC - connecting using PushButton
                                                   WPS-PIN - connecting using PIN code)
        :rtype: String
        :return name: Pin code if auth_type=WPS_PIN
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_direct_invite(self,peer_device="",force_freq=0):
        """
        Specific Edison

        Invite a peer to join a group or to reinvoke a persistent group.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type force_freq: int
        :param force_freq: Frequency used for the communication (forced)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_rssi(self, interface):
        """
        Gets the WiFi direct RSSI.
        :type interface: str
        :param interface: P2P interface to check
        :rtype: str
        :return: WiFi direct RSSI in dBm
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "get_wifi_direct_rssi")
