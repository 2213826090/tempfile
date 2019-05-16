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

:organization: INTEL QCTV
:summary: implementation of Agilent 8960 cellular network simulator
:since: 22/09/2014
:author: jduran4x
"""

import time
import os
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICellNetSim import ICellNetSim
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Common.DutConfig import DutConfig
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Database.CommandsDatabase import CommandsDatabase
from acs_test_scripts.TestStep.Utilities.Visa import VisaEquipmentBase
from acs_test_scripts.TestStep.Utilities.Visa import VisaInterface
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME
from Core.Factory import Factory
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class Agilent8960(ICellNetSim, VisaEquipmentBase):
    """
    Implementation of Agilent 8960 cellular network simulator

    The str message returned by the equipment when no error occured
    after sending a gpib command
    """
    GPIB_SUCCESS = '+0,"No error"'
    CONFIG_FILE_PATH = os.path.join(ICellNetSim.CONFIG_DIR, "A8960Configurations.xml")
    DB_FILE = os.path.join(ICellNetSim.DATABASE_DIR, "commands.db")

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: BenchConfigParameters
        :param bench_params: the dictionary containing equipment bench parameters
        """
        ICellNetSim.__init__(self)
        # Construct VisaEquipmentBase object
        VisaEquipmentBase.__init__(self, name, model, eqt_params, bench_params)
        # Initialize attributes
        self.__name = name
        self.__model = model
        self.__eqt_params = eqt_params
        self.__handle = None
        self._cell = None
        self.__dut_config = DutConfig(self._visa)

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()
        del self._cell

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def get_database_access(self):
        """
        returns the initialized database intarface object
        :rtype: CommandsDatabase
        :return:
        """
        db = CommandsDatabase(self.DB_FILE)
        db.connect()
        return db

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
        self.get_logger().info("Initialization")

        if self.get_handle() is None:
            raise Exception("Agilent 8960 VISA creation error")

    def set_cells_technology(self, cells_tech):
        """
        Sets cells technology used by the equipment

        :type cells_tech: List[str]
        :param cells_tech: List of cell technology (2G|GSM, 3G|WCDMA, TDSCDMA, 4G|LTE)
        """
        if len(cells_tech) == 1:
            cell_tech = cells_tech[0]
            if cell_tech in ["3G", "WCDMA"]:
                self._use_cell_3g()
            elif cell_tech in ["2G", "GSM"]:
                self._use_cell_2g()
            else:
                error_msg = "Wrong value for Cell tech {0} (Possible values: 4G|LTE)"
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)
        else:
            error_msg = "Only one cell can be set on CMW500 ({0} given)".format(len(cells_tech))
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, error_msg)

    def _use_cell_2g(self):
        """
        Initialize the 2G cell to be used.
        """
        from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech2G.Cell2GVisa import Cell2G
        self.get_logger().info("Select and prepare the 2G cell")
        self._cell = Cell2G(self._visa)
        self.switch_app_format("GPRS")
        self.perform_full_preset()
        self._cell.set_cell_off()
        self.apply_bench_config()

    def _use_cell_3g(self):
        """
        Initialize the 3G cell to be used.
        """
        from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Cell3GVisa import Cell3G
        self.get_logger().info("Select and prepare the 3G cell")
        self._cell = Cell3G(self._visa)
        self.switch_app_format("WCDMA")
        self.perform_full_preset()
        self._cell.set_cell_off()
        self.apply_bench_config()

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")
        if self.get_handle() is not None:
            self._set_handle(None)

    def get_cell_2g(self):
        """
        gets the cell object if it is Cell2G. Returns None if it is not.
        :rtype: Cell2G
        :return: the cell object
        """

        from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech2G.Cell2GVisa import Cell2G
        if isinstance(self._cell, Cell2G):
            return self._cell
        else:
            return None

    def get_cell_3g(self):
        """
        gets the cell object if it is Cell3G. Returns None if it is not.
        :rtype: Cell3G
        :return: the cell object
        """
        from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Cell3GVisa import Cell3G
        if isinstance(self._cell, Cell3G):
            return self._cell
        else:
            return None

    def get_cell(self):
        """
        gets the cell object. Returns None if it is not.
        :rtype: CellCommon
        :return: the cell object
        """
        from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Common.CellCommon import CellCommon
        if isinstance(self._cell, CellCommon):
            return self._cell
        else:
            return None

    def create_visa_interface(self, transport, kwargs):
        """
        Create visa interface
        :type transport: str
        :param transport: the bench configuration name of the equipment
        :type kwargs: dict
        :param kwargs: various needed parameters depending on the transport
        """
        self._visa = VisaInterface8960(transport, **kwargs)

    def perform_full_preset(self):
        """
        provoque a full preset of the equipment
        """
        self.reset()
        self._visa.send_command("SYST:PRES2")

    def apply_bench_config(self, bench_config=None):
        """
        Applies configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        available_params = self._bench_params.get_dict().keys()
        self.configure_amplitude_offset_table()
        # IP_Lan1
        if "IP_Lan1" in available_params:
            param = self._bench_params.get_param_value("IP_Lan1")
            self.set_ip4_lan_address(param)
        # IP_Lan2
        if "IP_Lan2" in available_params:
            param = self._bench_params.get_param_value("IP_Lan2")
            self.set_ip4_lan_address2(param)
        # DUT_IP_Address
        if "DUT_IP_Address" in available_params:
            param = self._bench_params.get_param_value("DUT_IP_Address")
            self.__dut_config.set_dut_ip_address(1, param)
        # DUT_IP_Address2
        if "DUT_IP_Address2" in available_params:
            param = self._bench_params.get_param_value("DUT_IP_Address2")
            self.__dut_config.set_dut_ip_address(2, param)
        # DNS1
        if "DNS1" in available_params:
            param = self._bench_params.get_param_value("DNS1")
            self.__dut_config.set_dut_primary_dns(param)
        # DNS2
        if "DNS2" in available_params:
            param = self._bench_params.get_param_value("DNS2")
            self.__dut_config.set_dut_secondary_dns(param)
        # Subnet_Mask
        if "Subnet_Mask" in available_params:
            param = self._bench_params.get_param_value("Subnet_Mask")
            self.set_ip4_subnet_mask(param)
        # Default_Gateway
        if "Default_Gateway" in available_params:
            param = self._bench_params.get_param_value("Default_Gateway")
            self.set_ip4_default_gateway(param)
        # Fast_Dormancy

    def get_app_format(self):
        """
        Gets the application format of the network simulator
        """
        return self._visa.query_command("SYST:APPL:FORM?")

    def switch_app_format(self, app_format):
        """
        Switches application format of the network simulator
        :type app_format: str
        :param app_format: the desired application format. Possible values:
            - "1xEV-DO"
            - "AMPS/136"
            - "GSM/GPRS"
            - "IS-2000/IS-95/AMPS"
            - "IS-856"
            - "WCDMA"
        """
        # Get current application format
        current_format = self.get_app_format()
        if app_format not in current_format:
            self.get_logger().info(
                "Switch application format from %s to %s",
                current_format, app_format)
            self._visa.send_command("SYST:APPL:FORM \"%s\"" % app_format)
            self.get_logger().info("Wait 5s after switch request for equipment stability")
            time.sleep(5)

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
            available_params = self._bench_params.get_dict().keys()
            # AmplitudeOffsetTable
            if "AmplitudeOffsetTable" in available_params:
                amplitude_file = ConfigsParser(self._bench_params.get_param_value("AmplitudeOffsetTable"))
                amplitude_table = amplitude_file.parse_amplitude_offset_table()
                frequency_list = amplitude_table.get("FrequencyList")
                offset_list = amplitude_table.get("OffsetList")
            elif "AmplFrequencyList" in available_params and \
                 "AmplOffsetList" in available_params:
                frequency_list = self._bench_params.get_dict()["AmplFrequencyList"]["value"]
                offset_list = self._bench_params.get_dict()["AmplOffsetList"]["value"]
        # Set Frequency points using FrequencyList (from Amplitude offset table)
        self._visa.send_command("SYST:CORR:FREQ %s" % frequency_list)
        # Set Amplitude Offset correction using offset list
        self._visa.send_command("SYST:CORR:GAIN %s" % offset_list)
        # Turning amplitude offset state to ON
        self._visa.send_command("SYST:CORR:STAT ON")

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
        self._visa.send_command("SYST:COMM:LAN:ADDR '%s'" % ip_addr)

    def set_ip4_lan_address2(self, ip_addr):
        """
        Sets second IPv4 LAN address
        :type ip_addr: str
        :param ip_addr: the IP address to set. 15 characters formatted
        as follows: A.B.C.D. The range of values for A = 0 to 126
        and 128 to 223. The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4LanAddress2 driver function
        """
        self._visa.send_command("SYST:COMM:LAN:ADDR2 '%s'" % ip_addr)

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4DefaultGateway driver function
        """
        self._visa.send_command("SYST:COMM:LAN:DGAT '%s'" % gateway)

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
        """
        self._visa.send_command("SYST:COMM:LAN:SMAS '%s'" % mask)

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
        return self._visa.query_command("SYST:STAT:COMM:LAN:EXT%d?" % device_number)

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
        return self._visa.query_command("SYST:STAT:COMM:LAN:EXT%d?" % device_number)

    def set_external_epc_connection(self, ns_lte_ip_lan1, ns_lte_dl_earfcn):
        """
        Set External EPC connection, only working with a PXT

        :type ns_lte_ip_lan1: str
        :param ns_lte_ip_lan1: the IP address to set for External EPC connection.
        15 characters formatted as follows: A.B.C.D.
        The range of values for A = 0 to 126 and 128 to 223.
        The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :type ns_lte_dl_earfcn: str
        :param ns_lte_dl_earfcn: the E-UTRA EARFCN value to set (0 - 65535)
        """

        # Prepare connection to the LTE network simulator
        self._cell.set_page_response("RESPOND")
        self._cell.set_red_ie_inclusion_state("ON")
        self._cell.set_eutra_earfcn(ns_lte_dl_earfcn)

        # Connect to external PXT
        self.connect_to_external_epc(ns_lte_ip_lan1)

        # Verify ethernet connection before timeout
        # 5 seconds using the PXT active IP address
        self.check_external_epc_connection(ns_lte_ip_lan1, 5)

    def set_external_ip_address(self, ip):
        """
        This function sets the external IP address
        :type ip: str
        :param ip: The external IP address to set
        :raise TestEquipmentException: failed to call SetExternalIpAddress driver function
        """
        self._visa.send_command("SYST:COMM:LAN:EXT:ADDR '%s'" % ip)

    def connect_to_external_device(self):
        """
        This function permits to connect to an external
        device if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        self._visa.send_command("SYST:COMM:LAN:EXT:CONN")

    def disconnect_from_external_device(self):
        """
        This function permits to disconnect from an external device
        :raise TestEquipmentException: failed to call DisconnectFromExternalDevice driver function
        """
        self._visa.send_command("SYST:COMM:LAN:EXT:DCON")

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

        while (timer < connection_time) and (connected is False):
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

        while (timer < disconnection_time) and (disconnected is False):
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

    def _get_visa_equipment(self):
        """
        Get agilent equipment with pyvisa interface
        """
        return self._visa

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

        self._visa.send_command(monitor_clear_cmd)
        self.get_logger().info("Start data throughput measurement")

        time.sleep(duration)

        for meas_type in type_list:
            get_throughput_cmd = "CALL:COUNt:DTMonitor:%s:DRATe?" % meas_type
            # The result is in a comma separated str with value in the Following order
            # Average data throughput(bps) ,Current data throughput(bps),
            # Peak data throughput(bps), Total data transfer(bytes)
            results = self.__visa.query_command(get_throughput_cmd)
            self.get_logger().debug(results)
            results = results.split(',')
            # Convert data throughput result with the good units: bps/kbps/Mbps
            for i in range(len(results) - 1):
                # If the throughput is Mbps
                if (float(results[i]) / 1000) >= 1000:
                    results[i] = str(round(float(results[i]) / (1000 * 1000), 2)) + "Mbps"
                # If the throughput is kbps
                elif (float(results[i]) / 1000) >= 1:
                    results[i] = str(round(float(results[i]) / 1000, 2)) + "Kbps"
                else:
                    results[i] = str(round(float(results[i]), 2)) + "bps"

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
        :param state: IPV6 router state on/off, default value is off.
        """

        if state in ("ON", "OFF"):
            self.get_logger().info("Setting the router mode of the Agilent8960 to "
                                   "%s" % state)
            self._visa.send_command("SYST:COMM:LAN:ROUT:STAT:IP6 %s" % state)
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
        self._visa.send_command("SYST:COMM:LAN:EPC:ADDR '%s'" % epc_address)
        self.get_logger().info("Connect to external EPC (LTE)")
        self._visa.send_command("SYST:COMM:LAN:EPC:CONN")

    def disconnect_from_external_epc(self):
        """
        This function permits to disconnect from an external
        epc (LTE) if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        self.get_logger().info("Disconnect from external EPC (LTE)")
        self._visa.send_command("SYST:COMM:LAN:EPC:DCON")

    def check_external_epc_disconnection(self, disconnection_time=0):
        """
        This function tests the external EPC disconnection before specified timeout

        :type disconnection_time: integer
        :param disconnection_time: Disconnection timeout
        """
        timer = 0
        disconnected = \
            self.is_external_epc_disconnected()

        while (timer < disconnection_time) and (disconnected is False):
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

        while (timer < connection_time) and (connected is False):
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
        conn_status = self._visa.query_command("SYST:STAT:COMM:LAN:EPC:CONN?")

        return conn_status

    def get_external_epc_connected_ip_address(self):
        """
        This function gets the connected external EPC IP address

        :rtype: str
        :return: The connected EPC IP address
        """
        self.get_logger().info("Get external EPC IP address")
        ip = self._visa.query_command("SYST:STAT:COMM:LAN:EPC:CONN:ADDR?")

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
        :type ip_default_gateway: str
        :param ip_default_gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :type ns1_ip_dut: str
        :param ns1_ip_dut: the DUT IP address to set.
        :type ip_dns1: str
        :param ip_dns1: the DUT primary DNS IP address to set.
        :type ip_dns2: str
        :param ip_dns2: the DUT secondary DNS IP address to set.
        """
        self.apply_bench_config()

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
            self._visa.send_command("%s %s" % (gpib_command_input, http_input))
        else:
            msg = "The state parameter should be either on or off"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        gpib_command_output = "CALL:SMS:HTTP:OUTP"
        if http_output in ("ON", "OFF"):
            self.get_logger().info("Setting the HTTP output interface for MT SMS to "
                                   "%s" % http_output)
            self._visa.send_command("%s %s" % (gpib_command_output, http_output))
        else:
            msg = "The state parameter should be either on or off"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: None
        """
        self._visa.send_command(command)

    def query_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: String
        :return: Message from equipment giving the value of a dedicated GPIB command
        """
        self._visa.query(command)


class VisaInterface8960(VisaInterface):
    """
    Add error handling to Agilent 8960 visa command
    """

    def __init__(self, transport, **kwargs):
        VisaInterface.__init__(self, transport, **kwargs)
        self._logger = Factory().create_logger("%s.%s.VISA_8960" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME))

    def send_command(self, command, bts=None):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :type bts: str
        :param bts: the BTS to apply the command
        """
        # Send command to the equipment
        self.write("*CLS")
        self.write(command)
        self.check_error()

    def query_command(self, command):
        """
        Query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """

        # Send command *CLS to the equipment to clear the Error Queue
        self.write("*CLS")

        # Query the command
        response = self.query(command)

        self.check_error()

        return response

    def check_error(self):
        """
        Check if an error occurred on 8960
        """
        # Retrieve error code from 8960
        err = self.query("SYSTEM:ERROR?")
        # Clear error message queue
        self.write("*CLS")
        self.write("DISPlay:WINDow:ERRor:CLEar")

        if "No error" in err:
            return
        else:
            # Error returned by 8960 is similar to -300, Device specific error
            # So split error code from error message
            err_list = err.split(",")
            err_code = int(err_list[0])
            err_msg = err_list[1]
            if err_code < 0:
                self._logger.error(err_msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
            elif err_code > 0:
                self._logger.warning(err_msg)
