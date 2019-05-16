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
:summary: implementation of Agilent 8960 cellular network simulator
:since: 08/03/2011
:author: ymorel
"""

import weakref
import time
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech2G.Cell2G import Cell2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Wrapper import WBase as W
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment


class Agilent8960(ICellNetSim, DllLoader):

    """
    Implementation of Agilent 8960 cellular network simulator

    The str message returned by the equipment when no error occured
    after sending a gpib command
    """
    GPIB_SUCCESS = '+0,"No error"'

    def __init__(self, name, model, eqt_params, bench_params, rat="2G3G"):
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
        :type rat: str
        :param rat: RAT used by equipment: combination of 2G, 3G or TD-SCDMA
        note 3G and TD-SCDMA are exclusive
        """
        ICellNetSim.__init__(self)
        # Construct DllLoader object
        DllLoader.__init__(self, name, model, eqt_params)
        # Initialize attributes
        self.__name = name
        self.__model = model
        self.__eqt_params = eqt_params
        self.__bench_params = bench_params
        self.__cell2g = None
        self.__cell3g = None
        self.__handle = None
        # pyvisa interface connection
        self.__visa = None

        # Initialize features
        self.__init_features(rat)

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()
        del self.__cell2g
        del self.__cell3g

    def __init_features(self, rat="2G3G"):
        """
        Initializes 8960 regarding rat specified

        :type rat: str
        :param rat: RAT used by equipment: combination of 2G, 3G or TD-SCDMA
        note 3G and TD-SCDMA are exclusive
        """
        if "2G" in rat:
            self.__cell2g = Cell2G(weakref.proxy(self))
        if "TD-SCDMA" in rat:
            from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.CellTDSCDMA import CellTDSCDMA
            self.__cell3g = CellTDSCDMA(weakref.proxy(self))
        elif "3G" in rat:
            from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Cell3G import Cell3G
            self.__cell3g = Cell3G(weakref.proxy(self))

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
        (err, handle, msg) = W.Connect(self, board_id, gpib_addr)
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
        Sets the connection handle
        :type handle: unsigned integer
        :param handle: the new connection handle
        """
        self.__handle = handle

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self.get_logger().debug("Initialization")

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
        self.get_logger().debug("Release")
        if self.get_handle() is not None:
            (err, msg) = W.Disconnect(self)
            self.unload_driver()
            self.__error_check(err, msg)
            # Update handle value
            self._set_handle(None)

    def get_cell_2g(self):
        """
        :rtype: Cell2G
        """
        if self.__cell2g is not None:
            return self.__cell2g
        msg = "2G is not available"
        self.get_logger().error(msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def get_cell_3g(self):
        """
        :rtype: ICell3G
        """
        if self.__cell3g is not None:
            return self.__cell3g
        msg = "3G is not available"
        self.get_logger().error(msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def perform_full_preset(self):
        """
        Wraps to PerformFullPreset function
        """
        (err, msg) = W.PerformFullPreset(self)
        self.__error_check(err, msg)

    def get_app_format(self):
        """
        Gets the application format of the network simulator
        """
        (err, app_format, msg) = W.GetAppFormat(self)
        self.__error_check(err, msg)

        return app_format

    def switch_app_format(self, app_format):
        """
        Switches application format of the network simulator
        :type format: str
        :param format: the desired application format. Possible values:
            - "1xEV-DO"
            - "AMPS/136"
            - "GSM/GPRS"
            - "IS-2000/IS-95/AMPS"
            - "IS-856"
            - "WCDMA"
        """
        # Get current application format
        (err, current_format, msg) = W.GetAppFormat(self)
        self.__error_check(err, msg)
        # Switch application if necessary
        if app_format not in current_format:
            self.get_logger().info(
                "Switch application format from %s to %s",
                current_format, app_format)
            (err, msg) = W.SetAppFormat(self, app_format)
            self.__error_check(err, msg)

    def configure_amplitude_offset_table(self, frequency_list=None, offset_list=None, second_antenna_offset_list=None):
        """
        Configures the amplitude offset table to compensate cable loss.
        :type frequency_list: str
        :param frequency_list: the frequency list.
        :type offset_list: str
        :param offset_list: the offset list corresponding to the frequency
        :type second_antenna_offset_list: str
        :param second_antenna_offset_list: the offset list corresponding to the frequency
        listed above for the diverse antenna.
        """
        # Retrieve from bench configuration the frequency and offset table if not given
        if frequency_list is None or offset_list is None:
            available_params = self.__bench_params.get_dict().keys()
            # AmplitudeOffsetTable
            if "AmplitudeOffsetTable" in available_params:
                amplitude_file = ConfigsParser(self.__bench_params.get_param_value("AmplitudeOffsetTable"))
                amplitude_table = amplitude_file.parse_amplitude_offset_table()
                frequency_list = amplitude_table.get("FrequencyList")
                offset_list = amplitude_table.get("OffsetList")
            elif "AmplFrequencyList" in available_params and \
                 "AmplOffsetList" in available_params:
                frequency_list = self.__bench_params.get_dict()["AmplFrequencyList"]["value"]
                offset_list = self.__bench_params.get_dict()["AmplOffsetList"]["value"]
        # Set Frequency points using FrequencyList (from Amplitude offset table)
        (err, msg) = W.SetCorrectionFrequency(self, frequency_list)
        self.__error_check(err, msg)
        # Set Amplitude Offset correction using offset list
        (err, msg) = W.SetCorrectionGain(self, offset_list)
        self.__error_check(err, msg)
        # Turning amplitude offset state to ON
        (err, msg) = W.SetCorrectionState(self, "ON")
        self.__error_check(err, msg)

    def set_ip4_lan_address(self, ip_addr):
        """
        Sets IPv4 LAN address
        :type ip_addr: str
        :param ip_addr: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4LanAddress driver function
        """
        (err, msg) = W.SetIp4LanAddress(self, ip_addr)
        self.__error_check(err, msg)

    def set_ip4_lan_address2(self, ip_addr):
        """
        Sets second IPv4 LAN address
        :type ip_addrr: str
        :param ip_addr: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4LanAddress2 driver function
        """
        (err, msg) = W.SetIp4LanAddress2(self, ip_addr)
        self.__error_check(err, msg)

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4DefaultGateway driver function
        """
        (err, msg) = W.SetIp4DefaultGateway(self, gateway)
        self.__error_check(err, msg)

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
        """
        (err, msg) = W.SetIp4SubnetMask(self, mask)
        self.__error_check(err, msg)

    def get_external_device_connection_status(self, device_number=1):
        """
        This function gets the external device connection status
        :type device_number: integer
        :param device_number: Number of the device to get connection status
        (1 or 2 because 8960 manages max 2 devices)
        :raise TestEquipmentException: failed to call GetExternalDeviceConnectionStatus driver function
        :rtype: str
        :return: The connection status of the device to check (1 or 2)
                 Status :
                    - CONG : Connecting
                    - CONN : Connected
                    - DISC : Disconnecting
                    - NCON : Not Connected
        """
        (err, conn_status, msg) = \
            W.GetExternalDeviceConnectionStatus(self, device_number)
        self.__error_check(err, msg)
        return conn_status

    def get_external_device_connected_ip_address(self, device_number=1):
        """
        This function gets the connected external device IP address
        :type device_number: integer
        :param device_number: Number of the device to get connection status
        (1 or 2 because 8960 manages max 2 devices)
        :raise TestEquipmentException: failed to call GetExternalDeviceConnectedIpAddress driver function
        :rtype: str
        :return: The connected device IP address
        """
        (err, ip, msg) = W.GetExternalDeviceConnectedIpAddress(self, device_number)
        self.__error_check(err, msg)

        return ip

    def set_external_ip_address(self, ip):
        """
        This function sets the external IP address
        :type ip: str
        :param ip: The external IP address to set
        :raise TestEquipmentException: failed to call SetExternalIpAddress driver function
        """
        if not NetworkingUtil.is_valid_ipv4_address(ip):
            msg = "Invalid external IP address passed to the equipment: %s" % ip
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        (err, msg) = W.SetExternalIpAddress(self, ip)
        self.__error_check(err, msg)

    def connect_to_external_device(self):
        """
        This function permits to connect to an external
        device if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        (err, msg) = W.ConnectToExternalDevice(self)
        self.__error_check(err, msg)

    def disconnect_from_external_device(self):
        """
        This function permits to disconnect from an external device
        :raise TestEquipmentException: failed to call DisconnectFromExternalDevice driver function
        """
        (err, msg) = W.DisconnectFromExternalDevice(self)
        self.__error_check(err, msg)

    def check_external_device_connection(self, device_number=1, external_ip=None, connection_time=0):
        """
        This function tests the external device connection before specified timeout
        If the device number is provided, the check is done on the good device.
        If the external ip address is provided, the check will also tests if the
        device see the good ip address as external ip address. If not, the check will
        only test if the equipment is in a connected state
        :type device_number: integer
        :param device_number: Number of the device to get connection status
        (8960 manages max 2 devices)
        :type external_ip: str
        :param external_ip: Ip address of the waited external device
        :type connection_time: integer
        :param connection_time: Connection timeout
        """
        timer = 0
        connected = \
            self.is_external_device_connected(device_number, external_ip)

        while (timer < connection_time) and (connected == False):
            time.sleep(1)
            timer += 1
            connected = \
                self.is_external_device_connected(device_number, external_ip)

        if not connected:  # Fail
            msg = "External device connection failed after %d seconds !" \
                % timer
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        self.get_logger().info(
            "External device successfully connected after %d seconds!",
            timer)

    def check_external_device_disconnection(self, device_number=1, disconnection_time=0):
        """
        This function tests the external device disconnection before specified timeout
        If the device number is provided, the check is done on the good device.
        :type device_number: integer
        :param device_number: Number of the device to get disconnection status
        (8960 manages max 2 devices)
        :type disconnection_time: integer
        :param disconnection_time: Disconnection timeout
        """
        timer = 0
        disconnected = \
            self.is_external_device_disconnected(device_number)

        while (timer < disconnection_time) and (disconnected == False):
            time.sleep(1)
            timer += 1
            disconnected = \
                self.is_external_device_disconnected(device_number)

        if not disconnected:  # Fail
            msg = "External device disconnection failed after %d seconds !" \
                % timer
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        self.get_logger().info(
            "External device successfully disconnected after %d seconds!",
            timer)

    def is_external_device_connected(self, device_number=1, external_ip=None):
        """
        This function tests the external device connection.
        If the device number is provided, the check is done on the good device.
        If the external ip address is provided, the check will also tests the
        device see the good ip address as external ip address
        :type device_number: integer
        :param device_number: Number of the device to get connection status
        (8960 manages max 2 devices)
        :type external_ip: str
        :param external_ip: Ip address of the waited external device
        :rtype: boolean
        :return: True if the external device is connected, else return False
        """
        if external_ip is not None:
            self.get_logger().info("Check external connection to IP Lan 2: %s", external_ip)
        else:
            self.get_logger().info("Check external connection", external_ip)

        result = False

        conn_res = self.get_external_device_connection_status(device_number)
        if conn_res == "CONN":
            result = True

        if external_ip is not None:
            res_ext_ip = \
                self.get_external_device_connected_ip_address(device_number)
            if external_ip == res_ext_ip:
                result = True
        else:
            result = False

        return result

    def is_external_device_disconnected(self, device_number=1):
        """
        This function tests the external device disconnection.
        If the device number is provided, the check is done on the good device.
        :type device_number: integer
        :param device_number: Number of the device to get connection status
        (8960 manages max 2 devices)
        :rtype: boolean
        :return: True if the external device is disconnected, else return False
        """
        self.get_logger().info("Check external connection")

        result = False

        conn_res = self.get_external_device_connection_status(device_number)
        if conn_res == "NCON":
            result = True

        return result

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: None
        """
        (err, msg) = W.SendCommand(self, command)
        self.__error_check(err, msg)

    def _get_visa_equipment(self):
        """
        Get agilent equipment with pyvisa interface
        It is used only for query command
        """

        # release DllLoader object
        self.release()

        self.get_logger().debug("Init agilent visa equipment")

        # Init agilent equipment with pyvisa interface
        self.__visa = VisaEquipment(self.__name,
                                    self.__model,
                                    self.__eqt_params,
                                    self.__bench_params)

        self.__visa.connect()

    def query_command(self, command):
        """
        query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """

        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command : *CLS")
        self.send_command("*CLS")

        # Get agilent equipment with pyvisa interface
        self._get_visa_equipment()

        # Query the command
        self.get_logger().debug("Query GPIB command : %s" % command)
        response = self.__visa.query(command)

        # Check errors in the System Error buffer
        return_msg = self.__visa.query("SYST:ERR?")

        if return_msg != Agilent8960.GPIB_SUCCESS:
            self.get_logger().error(return_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, return_msg)

        # Release agilent equipment with pyvisa interface
        self.__visa.disconnect()
        self.__visa = None

        # reinit DllLoader object
        self.init()

        return response

    def get_data_throughput(self, duration, type_list):
        """
        Get the data throughput during the duration time

        :type duration: int
        :param duration: measurement duration

        :type type_list: String list
        :param type_list: The type of throughput to measure: OTARx,OTATx,IPTX,IPRX,etc

        :rtype: dict
        :return: the dictionary of test result in the following order:
        Average data throughput(bps/Kbps/Mbps) ,Current data throughput(bps/Kbps/Mbps),
        Peak data throughput(bps/Kbps/Mbps), Total data transfer(bytes/KB/MB)
        """

        data_throughput_dict = {}

        # The agilent gpib comand for data measure
        monitor_clear_cmd = "CALL:COUNt:DTMonitor:CLEar"

        self.send_command(monitor_clear_cmd)
        self.get_logger().info("Start data throughput measurement")

        time.sleep(duration)

        for meas_type in type_list:
            get_throughput_cmd = "CALL:COUNt:DTMonitor:%s:DRATe?" % meas_type
            # The result is in a comma separated str with value in the Following order
            # Average data throughput(bps) ,Current data throughput(bps),
            # Peak data throughput(bps), Total data transfer(bytes)
            results = self.query_command(get_throughput_cmd)
            self.get_logger().debug(results)
            results = results.split(',')
            # Convert data throughput result with the good units: bps/kbps/Mbps
            for i in range(len(results) - 1):
                # If the throughput is Mbps
                if (float(results[i]) / 1000) >= 1000:
                    results[i] = str(round(float(results[i]) / (1000 * 1000), 3)) + "Mbps"
                # If the throughput is kbps
                elif (float(results[i]) / 1000) >= 1:
                    results[i] = str(round(float(results[i]) / 1000, 3)) + "Kbps"
                else:
                    results[i] = str(round(float(results[i]), 3)) + "bps"

            # Convert total data transfer(bytes/kb/MB)with the good units: bytes/kb/MB
            if (float(results[-1]) / 1024) >= 1024:
                results[-1] = str(round(float(results[-1]) / (1024 * 1024), 2)) + "MB"
            # If the throughput is kbps
            elif (float(results[-1]) / 1024) >= 1:
                results[-1] = str(round(float(results[-1]) / 1024, 2)) + "KB"
            else:
                results[-1] = str(round(float(results[-1]), 2)) + "bytes"

            data_throughput_dict.update({meas_type: results})

        self.get_logger().info("Stop data throughput measurement")

        return data_throughput_dict

    def set_ipv6_router_mode(self, state):
        """
        Enable or Disable network simulator IPV6 router state
        :type state: str
        :param ip_num: IPV6 router state on/off, default value is off.
        """

        gpib_command = "SYST:COMM:LAN:ROUT:STAT:IP6"
        if state in ("ON", "OFF"):
            self.get_logger().info("Setting the router mode of the Agilent8960 to "
                                   "%s" % state)
            self.send_command("%s %s" % (gpib_command, state))
        else:
            msg = "The state parameter should be either on or off"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def connect_to_external_epc(self, epc_address):
        """
        This function permits to connect to an external
        epc (LTE) if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        self.send_command("SYST:COMM:LAN:EPC:ADDR '%s'" % epc_address)
        self.get_logger().info("Connect to external EPC (LTE)")
        self.send_command("SYST:COMM:LAN:EPC:CONN")

    def disconnect_from_external_epc(self):
        """
        This function permits to disconnect from an external
        epc (LTE) if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        self.get_logger().info("Disconnect from external EPC (LTE)")
        self.send_command("SYST:COMM:LAN:EPC:DCON")

    def check_external_epc_disconnection(self, disconnection_time=0):
        """
        This function tests the external EPC disconnection before specified timeout

        :type disconnection_time: integer
        :param disconnection_time: Disconnection timeout
        """
        timer = 0
        disconnected = \
            self.is_external_epc_disconnected()

        while (timer < disconnection_time) and (disconnected == False):
            time.sleep(1)
            timer += 1
            disconnected = \
                self.is_external_device_disconnected()

        if not disconnected:  # Fail
            msg = "External device disconnection failed after %d seconds !" \
                % timer
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        self.get_logger().info(
            "External device successfully disconnected after %d seconds!",
            timer)

    def is_external_epc_connected(self, external_ip=None):
        """
        This function tests the external EPC connection.

        :type external_ip: str
        :param external_ip: Ip address of the waited external device

        :rtype: boolean
        :return: True if the external device is connected, else return False
        """
        if external_ip is not None:
            self.get_logger().info("Check external EPC connection to IP Lan 2: %s", external_ip)
        else:
            self.get_logger().info("Check external EPC connection", external_ip)

        result = False

        conn_res = self.get_external_epc_connection_status()
        if conn_res == "CONN":
            result = True

        if external_ip is not None:
            res_ext_ip = \
                self.get_external_epc_connected_ip_address()
            if external_ip == res_ext_ip:
                result = True
        else:
            result = False

        return result

    def is_external_epc_disconnected(self):
        """
        This function tests the external EPC disconnection.

        :rtype: boolean
        :return: True if the external device is disconnected, else return False
        """
        self.get_logger().info("Check external EPC connection")

        result = False

        conn_res = self.get_external_epc_connection_status()
        if conn_res == "NCON":
            result = True

        return result

    def check_external_epc_connection(self, external_ip=None, connection_time=0):
        """
        This function tests the external EPC connection before specified timeout

        :type external_ip: str
        :param external_ip: Ip address of the waited external device
        :type connection_time: integer
        :param connection_time: Connection timeout
        """
        timer = 0
        connected = \
            self.is_external_epc_connected(external_ip)

        while (timer < connection_time) and (connected == False):
            time.sleep(1)
            timer += 1
            connected = \
                self.is_external_epc_connected(external_ip)

        if not connected:  # Fail
            msg = "External EPC connection failed after %d seconds !" \
                % timer
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        self.get_logger().info(
            "External EPC successfully connected after %d seconds!",
            timer)

    def get_external_epc_connection_status(self):
        """
        This function gets the External EPC Conn Status .

        :rtype: str
        :return: The connection status of the EPC
                 Status :
                    - CONN : Connected
                    - NCON : Not Connected
        """
        self.get_logger().info("Check connection to external EPC")
        conn_status = self.query_command("SYST:STAT:COMM:LAN:EPC:CONN?")

        return conn_status

    def get_external_epc_connected_ip_address(self):
        """
        This function gets the connected external EPC IP address

        :rtype: str
        :return: The connected EPC IP address
        """
        self.get_logger().info("Get external EPC IP address")
        ip = self.query_command("SYST:STAT:COMM:LAN:EPC:CONN:ADDR?")

        return ip

    def set_ip_addresses(self,
                         ip_lan1,
                         ip_lan2,
                         ip_subnet_mask,
                         ip_default_gateway,
                         ns1_ip_dut,
                         ip_dns1,
                         ip_dns2):
        """
        This function sets the IP addresses of the Network Simulator

        :type ip_lan1: str
        :param ip_lan1: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :type ip_lan2: str
        :param ip_lan2: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :type ip_subnet_mask: str
        :param ip_subnet_mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :type ns1_ip_dut: str
        :param ns1_ip_dut: the DUT IP address to set.
        :type ip_dns1: str
        :param ip_dns1: the DUT primary DNS IP address to set.
        :type ip_dns2: str
        :param ip_dns2: the DUT secondary DNS IP address to set.
        """
        self.set_ip4_lan_address(ip_lan1)
        self.set_ip4_lan_address2(ip_lan2)
        self.set_ip4_subnet_mask(ip_subnet_mask)
        self.set_ip4_default_gateway(ip_default_gateway)

        # The following code use duplicate methods from 2G and 3G data
        # 3G data will be used
        self.get_cell_3g().get_data().clear_all_dut_ip_addresses()
        self.get_cell_3g().get_data().set_dut_ip_address(1, ns1_ip_dut)
        self.get_cell_3g().get_data().set_dut_primary_dns(ip_dns1)
        self.get_cell_3g().get_data().set_dut_secondary_dns(ip_dns2)

    def set_sms_http(self, http_input="OFF", http_output="OFF"):
        """
        This function sets the HTTP input/output interfaces for :
          - mobile terminated SMS
          - mobile originated SMS

        :type http_input: str
        :param http_input: Enables (ON) or disables (OFF) the HTTP
        input interface for mobile terminated SMS
        :type http_output: str
        :param http_output: Enables (ON) or disables (OFF) the HTTP
        input interface for mobile originated SMS
        """
        gpib_command_input = "CALL:SMS:HTTP:INP"
        if http_input in ("ON", "OFF"):
            self.get_logger().info("Setting the HTTP input interface for MT SMS to "
                                   "%s" % http_input)
            self.send_command("%s %s" % (gpib_command_input, http_input))
        else:
            msg = "The state parameter should be either on or off"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        gpib_command_output = "CALL:SMS:HTTP:OUTP"
        if http_output in ("ON", "OFF"):
            self.get_logger().info("Setting the HTTP output interface for MT SMS to "
                                   "%s" % http_output)
            self.send_command("%s %s" % (gpib_command_output, http_output))
        else:
            msg = "The state parameter should be either on or off"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
