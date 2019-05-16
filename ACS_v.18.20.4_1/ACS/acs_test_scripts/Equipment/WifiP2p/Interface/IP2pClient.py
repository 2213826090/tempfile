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
:summary: virtual interface to p2p cli
:since:25/06/2013
:author: smaurel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IP2pClient(object):
    """
    Virtual interface for WiFi Sniffer Equipment
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

    def set_lan_interface_ip_address(self, ip_address, timeout):
        """
        Set the interface ip address.
        If the WPA Cli is not run, the SSH connection is use,
        else, a new SSH connection is opened and close just after
        :type ip_address: str
        :param ip_address: the ip address to set on interface
        :type timeout: int
        :param timeout: timeout in second to wait the reply from WPA_Cli
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start(self, lan_interface):
        """
        Start the P2P Client
        :type lan_interface: str
        :param lan_interface: the interface to connect on
        :rtype: str
        :return: the P2P Client Mac address
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop(self):
        """
        Stop the P2P Client
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_all_network(self):
        """
        remove the remaining p2p groups networks
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def find_network(self):
        """
        Find available p2p network
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def find_dut(self, dut_mac, timeout):
        """
        Find and listen a DUT in the network list

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type timeout: int
        :param timeout: maximum delay (in seconds) to search
        :rtype: str
        :return: None if not found. The Dut name if found
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_find(self):
        """
        Stop the P2P find
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def listen(self):
        """
        Active the P2p listen mode
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_listen_channel(self, channel):
        """
        Set the P2p listen channel
        :type channel: int
        :param channel: channel to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_device_name(self, name):
        """
        Set the P2p device name
        :type name: str
        :param name: the device name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_pbc(self, dut_mac, go_intent, persistent=False, frequency=2412):
        """
        Launch a connection to the DUT or accept a connection from the DUT on PBC mode.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_pin_display(self, dut_mac, go_intent, persistent=False, frequency=2412):
        """
        Launch a connection to the DUT with PIN request mode.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        :rtype: str
        :return: The Pin code
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_pin_enter(self, dut_mac, pin_code, go_intent, persistent=False, frequency=2412):
        """
        Accept a connection from the DUT on PIN request mode.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type pin_code: str
        :param pin_code: The pin code to connect
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disconnect(self):
        """
        disconnect the other DUT
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
