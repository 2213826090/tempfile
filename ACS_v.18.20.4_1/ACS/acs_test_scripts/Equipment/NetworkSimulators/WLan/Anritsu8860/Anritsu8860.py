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
:summary: implementation of Anritsu 8860 WLAN network simulator
:since: 28/03/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.NetworkSimulators.WLan.Interface.IWLanNetSim import IWLanNetSim
from acs_test_scripts.Equipment.NetworkSimulators.WLan.Anritsu8860.Wrapper import WAnritsu8860 as W


class Anritsu8860(IWLanNetSim, DllLoader):

    """
    Implementation of Anritsu 8860 WLAN network simulator
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        IWLanNetSim.__init__(self)
        # Construct DllLoader object
        DllLoader.__init__(self, name, model, eqt_params)
        # Initialize attributes
        self.__bench_params = bench_params
        self.__handle = None

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def __connect_via_GPIB(self):
        """
        Connect to equipment via GPIB
        """
        board_id = int(self.__bench_params.get_param_value("GPIBBoardId"))
        gpib_addr = int(self.__bench_params.get_param_value("GPIBAddress"))
        (err, handle, msg) = W.Connect(self, "GPIB", board_id, gpib_addr, "")
        self.__error_check(err, msg)
        # Update handle value
        self._set_handle(handle)

    def __connect_via_TCPIP(self):
        """
        Connect to equipment via TCPIP
        """
        ip_addr = int(self.__bench_params.get_param_value("TcpIpAddress"))
        (err, handle, msg) = W.Connect(self, "TCPIP", 0, 0, ip_addr)
        self.__error_check(err, msg)
        # Update handle value
        self._set_handle(handle)

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

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        # Load the equipment driver
        self.load_driver()

        # Get transport mode and try to connect to equipment
        transport = str(self.__bench_params.get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            connect = getattr(self, "_" + self.__class__.__name__ +
                              "__connect_via_" + transport)
            connect()
        else:
            msg = "%s transport is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")
        if self.get_handle() is not None:
            (err, msg) = W.Disconnect(self)
            self.unload_driver()
            self.__error_check(err, msg)
            # Update handle value
            self._set_handle(None)

    def perform_full_preset(self):
        """
        Wraps to PerformFullPreset function
        """
        (err, msg) = W.PerformFullPreset(self)
        self.__error_check(err, msg)

    def set_measurement_mode(self, mode):
        """
        Wraps to SetMeasurementMode function
        :type mode: str
        :param mode: measurement mode. Possible values:
            - "TXMODE"
            - "RXMODE"
        :raise TestEquipmentException: Invalid measurement mode.
        """
        (err, msg) = W.SetMeasurementMode(self, mode)
        self.__error_check(err, msg)

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
        (err, msg) = W.AddLossEntry(self, channel, offset)
        self.__error_check(err, msg)

    def set_loss_table_state(self, state):
        """
        Wraps the SetLossTableState driver function
        :type state: str
        :param state: The loss table state. Possible values :
            - OFF (Disable the path loss table)
            - ON (Enable the path loss table (Default Value))
        """
        (err, msg) = W.SetLossTableState(self, state)
        self.__error_check(err, msg)

    def set_signal_state(self, state):
        """
        Wraps the SetSignalState driver function
        :type state: str
        :param state: The signal generator state. Possible values :
            - OFF (Disable the signal generator function)
            - ON (Enable the signal generator function (Default Value))
        """
        (err, msg) = W.SetSignalState(self, state)
        self.__error_check(err, msg)

    def set_wlan_ip_params(self, ip_addr, subnet):
        """
        Wraps the SetWlanIpParams driver function
        :type ip_addr: str
        :param ip_addr: WLAN IP Address.
        :type subnet: str
        :param subnet: WLAN Subnet Mask.
        """
        (err, msg) = W.SetWlanIpParams(self, ip_addr, subnet)
        self.__error_check(err, msg)

    def set_transmit_power(self, power):
        """
        Wraps the SetTransmitPower driver function
        :type power: double
        :param power: Power level at the test port connector.
        :raise TestEquipmentException: Failed to set transmit power.
        """
        (err, msg) = W.SetTransmitPower(self, power)
        self.__error_check(err, msg)

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
        (err, msg) = W.SetWlanStandard(self, standard)
        self.__error_check(err, msg)

    def set_wlan_channel(self, channel):
        """
        Wraps the SetWlanChannel driver function
        :type channel: integer
        :param channel: WLAN channel to set (1 to 196).
        :raise TestEquipmentException: Invalid channel, must be 1 to 196.
        """
        (err, msg) = W.SetWlanChannel(self, channel)
        self.__error_check(err, msg)

    def set_wlan_data_rate(self, rate):
        """
        Wraps the SetWlanDataRate driver function
        :type rate: str
        :param rate: WLAN data rate. Possible values :
            - {"1"; "2"; "5.5"; "6"; "9"; "11"; "12"; "18"; "24";
               "36"; "48"; "54"}
        :raise TestEquipmentException: Invalid data rate.
        """
        (err, msg) = W.SetWlanDataRate(self, rate)
        self.__error_check(err, msg)

    def set_test_mode(self, mode):
        """
        Wraps the SetTestMode driver function
        :type mode: str
        :param mode: Equipment test mode. Possible values :
            - "DIRECT"
            - "NETWORK"
        :raise TestEquipmentException: Invalid test mode.
        """
        (err, msg) = W.SetTestMode(self, mode)
        self.__error_check(err, msg)

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
        (err, msg) = W.SetNetworkType(self, nw_type)
        self.__error_check(err, msg)

    def create_network(self, ssid):
        """
        Wraps the CreateNetwork driver function
        :type ssid: str
        :param ssid: Service Set Identity of the Wlan network (32 characters max).
        :raise TestEquipmentException: SSID has more than 32 characters.
        """
        (err, msg) = W.CreateNetwork(self, ssid)
        self.__error_check(err, msg)

    def set_dut_power(self, power):
        """
        Wraps the SetDutPower driver function
        :type power: integer
        :param power: DUT power level in dBm (between -30 and 30).
        """
        (err, msg) = W.SetDutPower(self, int(power))
        self.__error_check(err, msg)

    def launch_auto_setup(self):
        """
        Wraps the LaunchAutoSetup driver function
        """
        (err, msg) = W.LaunchAutoSetup(self)
        self.__error_check(err, msg)

    def set_ip_properties_mode(self, mode):
        """
        Wraps the SetIpPropertiesMode driver function
        :type mode: str
        :param mode: Mode for IP properties. Possible values :
            - "AUTO"
            - "MANUAL"
        :raise TestEquipmentException: Invalid IP properties mode.
        """
        (err, msg) = W.SetIpPropertiesMode(self, mode)
        self.__error_check(err, msg)

    def set_dut_ip(self, ip_addr):
        """
        Wraps the SetDutIp driver function
        :type ip_addr: str
        :param ip_addr: Device Under Test IP V4 address.
        """
        (err, msg) = W.SetDutIp(self, ip_addr)
        self.__error_check(err, msg)

    def set_dut_mac_address(self, mac):
        """
        Wraps the SetDutMacAddress driver function
        :type mac: str
        :param mac: The DUT MAC address to set.
        """
        (err, msg) = W.SetDutMacAddress(self, mac)
        self.__error_check(err, msg)

    def set_per_packet_number(self, packet_number):
        """
        Wraps the SetPERPacketNumber driver function
        :type packet_number: integer
        :param packet_number : Number of packets to send for Packet
        Error Rate Measurement.
        """
        (err, msg) = W.SetPERPacketNumber(self,
                                          packet_number)
        self.__error_check(err, msg)

    def check_dut_connected_before_timeout(self, dut_mac_address, timeout):
        """
        Checks that DUT is connected until timeout seconds.
        :type dut_mac_address: str
        :param dut_mac_address: MAC address of the DUT.
        :type timeout: int
        :param timeout: Timeout in seconds.
        """
        connect_time = 0
        connect_state = False
        self.get_logger().info(
            "Check DUT gets connected until %d seconds",
            timeout)
        while connect_time <= timeout:
            time.sleep(1)
            connect_time += 1
            (err, dut_state, msg) = W.GetDUTConnectionState(
                self,
                dut_mac_address)
            self.__error_check(err, msg)

            if dut_state == b"CONNECTED":
                self.get_logger().info("DUT is connected!")
                connect_state = True
                break

        if not connect_state:
            error_msg = "Connection failure: timeout!"
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, error_msg)

    def check_dut_disconnected_before_timeout(self, dut_mac_address, timeout):
        """
        Checks that DUT is disconnected until timeout seconds.
        :type dut_mac_address: str
        :param dut_mac_address: MAC address of the DUT.
        :type timeout: int
        :param timeout: Timeout in seconds.
        """
        connect_time = 0
        connect_state = False
        self.get_logger().info(
            "Check DUT gets disconnected before %d seconds", timeout)
        while connect_time <= timeout:
            time.sleep(1)
            connect_time += 1

            (err, dut_state, msg) = W.GetDUTConnectionState(
                self,
                dut_mac_address)
            self.__error_check(err, msg)

            if dut_state == b"DISCONNECTED":
                self.get_logger().info("DUT is disconnected!")
                connect_state = True
                break

        if not connect_state:
            error_msg = "Disconnection failure: timeout!"
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, error_msg)

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
        (err, retrieved_ue_power, msg) = W.GetPowerPeak(self)
        self.__error_check(err, msg)

        self.get_logger().info(
            "Wlan power peak: %s dBm",
            str(retrieved_ue_power))
        positive_margin = margin
        if margin < 0:
            positive_margin = -margin
        min_power = expected_ue_power - positive_margin
        max_power = expected_ue_power + positive_margin
        if (retrieved_ue_power < min_power) or (retrieved_ue_power > max_power):
            msg = "DUT isn't correctly connected to the WLAN network simulator"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    def get_packet_error_rate(self):
        """
        Gets the packet error rate
        :rtype: double
        :return: measured packet error Rate.
        """
        (err, per, msg) = W.GetPacketErrorRate(self)
        self.__error_check(err, msg)
        return per

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
        :param interval: interval in ms between beacons (20 to 1000)

        :type preamble: str
        :param preamble: beacon preamble. Allowed values are:
            - LONG
            - SHORT

        :return: None
        """
        # Set beacon operational rate set
        (err, msg) = W.SetOperationalRate(self, rate_set)
        self.__error_check(err, msg)
        # Set beacon interval
        (err, msg) = W.SetBeaconInterval(self, interval)
        self.__error_check(err, msg)
        # Set beacon preamble
        (err, msg) = W.SetBeaconPreamble(self, preamble)
        self.__error_check(err, msg)
