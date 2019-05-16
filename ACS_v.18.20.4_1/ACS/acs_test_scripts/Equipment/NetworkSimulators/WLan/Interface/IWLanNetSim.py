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
:summary: virtual interface with WLan network simulators
simulators
:since: 28/03/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IWLanNetSim(object):

    """
    IWLanNetSim class: virtual interface with WLAN network simulators.
    """

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Wraps to PerformFullPreset function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_measurement_mode(self, mode):
        """
        Wraps to SetMeasurementMode function
        :type mode: str
        :param mode: measurement mode. Possible values:
            - "TXMODE"
            - "RXMODE"
        :raise TestEquipmentException: Invalid measurement mode.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def add_loss_entry(self, channel, offset):
        """
        Wraps to AddLossEntry driver function
        :type channel: integer
        :param channel: The channel number
        :type channel: double
        :param channel: The offset to set in dBm
        :raise TestEquipmentException: failed to add loss entry
        :rtype: integer
        :return: the error code of the driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_loss_table_state(self, state):
        """
        Wraps the SetLossTableState driver function
        :type state: str
        :param state: The loss table state. Possible values :
            - OFF (Disable the path loss table)
            - ON (Enable the path loss table (Default Value))
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_signal_state(self, state):
        """
        Wraps the SetSignalState driver function
        :type state: str
        :param state: The signal generator state. Possible values :
            - OFF (Disable the signal generator function)
            - ON (Enable the signal generator function (Default Value))
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wlan_ip_params(self, ip_addr, subnet):
        """
        Wraps the SetWlanIpParams driver function
        :type ip_addr: str
        :param ip_addr: WLAN IP Address.
        :type subnet: str
        :param subnet: WLAN Subnet Mask.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_transmit_power(self, power):
        """
        Wraps the SetTransmitPower driver function
        :type power: double
        :param power: Power level at the test port connector.
        :raise TestEquipmentException: Failed to set transmit power.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wlan_standard(self, standard):
        """
        Wraps the SetWlanStandard driver function
        :type standard: str
        :param standard: WLAN Standard. Possible values :
            - "802.11a"
            - "802.11b"
            - "802.11g"
        :raise TestEquipmentException: Invalid Wlan standard.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wlan_channel(self, channel):
        """
        Wraps the SetWlanChannel driver function
        :type channel: integer
        :param channel: WLAN channel to set (1 to 196).
        :raise TestEquipmentException: Invalid channel, must be 1 to 196.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_wlan_data_rate(self, rate):
        """
        Wraps the SetWlanDataRate driver function
        :type rate: str
        :param rate: WLAN data rate. Possible values :
            - "1"
            - "2"
            - "5.5"
            - "6"
            - "9"
            - "11"
            - "12"
            - "18"
            - "24"
            - "36"
            - "48"
            - "54"
        :raise TestEquipmentException: Invalid data rate.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_test_mode(self, mode):
        """
        Wraps the SetTestMode driver function
        :type mode: str
        :param mode: Equipment test mode. Possible values :
            - "DIRECT"
            - "NETWORK"
        :raise TestEquipmentException: Invalid test mode.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_network_type(self, nw_type):
        """
        Wraps the SetNetworkType driver function
        :type nw_type: str
        :param nw_type: Network type. Possible values :
            - "ADHOC"
            - "ACCESS_POINT"
            - "STATION"
        :raise TestEquipmentException: Invalid network type.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def create_network(self, ssid):
        """
        Wraps the CreateNetwork driver function
        :type ssid: str
        :param ssid: Service Set Identity of the Wlan network (32 characters max).
        :raise TestEquipmentException: SSID has more than 32 characters.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_power(self, power):
        """
        Wraps the SetDutPower driver function
        :type power: integer
        :param power: DUT power level in dBm (between -30 and 30).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def launch_auto_setup(self):
        """
        Wraps the LaunchAutoSetup driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ip_properties_mode(self, mode):
        """
        Wraps the SetIpPropertiesMode driver function
        :type mode: str
        :param mode: Mode for IP properties. Possible values :
            - "AUTO"
            - "MANUAL"
        :raise TestEquipmentException: Invalid IP properties mode.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_ip(self, ip_addr):
        """
        Wraps the SetDutIp driver function
        :type ip_addr: str
        :param ip_addr: Device Under Test IP V4 address.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_mac_address(self, mac):
        """
        Wraps the SetDutMacAddress driver function
        :type mac: str
        :param mac: The DUT MAC address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_per_packet_number(self, packet_number):
        """
        Wraps the SetPERPacketNumber driver function
        :type packet_number: integer
        :param packet_number : Number of packets to send for Packet
        Error Rate Measurement.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_dut_connected_before_timeout(self, dut_mac_address, timeout):
        """
        Checks that DUT is connected until timeout seconds.
        :type dut_mac_address: str
        :param dut_mac_address: MAC address of the DUT.
        :type timeout: integer
        :param timeout: Timeout in seconds.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_dut_disconnected_before_timeout(self, dut_mac_address, timeout):
        """
        Checks that DUT is disconnected until timeout seconds.
        :type dut_mac_address: str
        :param dut_mac_address: MAC address of the DUT.
        :type timeout: integer
        :param timeout: Timeout in seconds.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_connectivity(self, expected_ue_power, margin):
        """
        Checks that DUT is correctly connected to the wlan network simulator.
        Connectivity is well connected if retrieved power is equal to
        expectedUEPower +/- margin.
        :type expected_ue_power: integer
        :param expected_ue_power: Expected user equipment power (in dBm).
        :type margin: integer
        :param margin: admissible margin (in dB).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_packet_error_rate(self):
        """
        Gets the packet error rate
        :rtype: double
        :return: measured packet error Rate.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_beacon(self, rate_set, interval, preamble):
        """
        Configure beacon that will set:
        - beacon operational rate set
        - interval between beacons
        - beacon preamble

        :type rate_set: str
        :param rate_set: operational rate set. Allowed values are:
            - ALL
            - SINGLE
            - MULTIPLE
            - USER

        :type interval: int
        :param interval: interval in ms between beacons (20 to 1000).

        :type preamble: str
        :param preamble: beacon preamble. Allowed values are:
            - LONG
            - SHORT

        :return: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
