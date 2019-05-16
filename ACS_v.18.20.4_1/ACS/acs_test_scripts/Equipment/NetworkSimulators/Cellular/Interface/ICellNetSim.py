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
:summary: virtual interface of base functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

import os
from Core.PathManager import Paths
from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ICellNetSim(object):

    """
    ICellNetSim class: virtual interface of base functionalities for cellular
    network simulators.
    """

    CONFIG_DIR = os.path.join(Paths.CONFIGS, "NetworkSimulators", "Cellular")
    DATABASE_DIR = os.path.join(Paths.CONFIGS, "NetworkSimulators", "Cellular", "Database")

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

    def get_cell_2g(self):
        """
        Access to 2G cellular interface.
        :rtype: ICell2G
        :return: the 2G cellular object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cell_3g(self):
        """
        Access to 3G cellular interface.
        :rtype: ICell3G
        :return: the 3G cellular object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cell(self):
        """
        gets the cell object. Returns None if it is not.
        :rtype: CellCommon
        :return: the cell object
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cell_4g(self):
        """
        Access to LTE cellular interface.
        :rtype: ICell4G
        :return: the 4G cellular object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cells_technology(self, cells_tech):
        """
        Sets cells technology used by the equipment

        :type cells_tech: List[str]
        :param cells_tech: List of cell technology (2G|GSM, 3G|WCDMA, TDSCDMA, 4G|LTE)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def apply_bench_config(self, bench_config):
        """
        Applies configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_app_format(self):
        """
        Gets the application format of the network simulator
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def switch_app_format(self, app_format):
        """
        Switches application format of the network simulator.
        :type app_format: str
        :param app_format: the desired application format. Possible values:
            - "1xEV-DO"
            - "AMPS/136"
            - "GSM/GPRS"
            - "IS-2000/IS-95/AMPS"
            - "IS-856"
            - "WCDMA"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_amplitude_offset_table(self, frequency_list, offset_list, second_antenna_offset_list):
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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ul_dl_attenuation(self, ul_att, dl_att):
        """
        Sets uplink and downlink attenuations
        :type ul_att: double
        :param ul_att: uplink attenuation to set:
            - 2G: -50 dB to +90 dB
            - 3G: -50 dB to +50 dB
        :type dl_att: double
        :param dl_att: downlink attenuation to set:
            - 2G: -50 dB to +90 dB
            - 3G: -50 dB to +50 dB
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4DefaultGateway driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_external_ip_address(self, ip):
        """
        This function sets the external IP address
        :type ip: str
        :param ip: The external IP address to set
        :raise TestEquipmentException: failed to call SetExternalIpAddress driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_to_external_device(self):
        """
        This function permits to connect to an external
        device if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_from_external_device(self):
        """
        This function permits to disconnect from an external device
        :raise TestEquipmentException: failed to call DisconnectFromExternalDevice driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def query_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: String
        :return: Message from equipment giving the value of a dedicated GPIB command
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_configuration_state_file(self, filename):
        """
        Loads equipment parameters from a file.
        @type filename: string
        @param filename: the configuration file to load with his path
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_configuration_file(self, filename):
        """
        Loads equipment parameters from a file.
        :type filename: str
        :param filename: the configuration file to load with his path
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_scenario(self):
        """
        Start the scenario
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_scenario(self):
        """
        Stop the scenario
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_data_throughput(self, duration, type_list):
        """
        Get the data throughput during the duration time

        :type duration: int
        :param duration: measurement duration

        :type type_list: String list
        :param type_list: The type of throughput to measure: OTARx,OTATx,IPTX,IPRX,etc

        :rtype: dict
        :return: the dictionary of test result in the following order:
        Average data throughput(bps) ,Current data throughput(bps),
        Peak data throughput(bps), Total data transfer(bytes)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ipv6_router_mode(self, state):
        """
        Enable or Disable network simulator IPV6 router state
        :type state: str
        :param ip_num: IPV6 router state on/off, default value is off.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_cell_config(self, config_type, eqt_id=1):
        """
        Configure equipment to specific cells via .xml files
        in order to reach specific throughput

        :type config_type: str
        :param config_type: the type of configuration(band, category, etc) to load

        :type eqt_id: int
        :param eqt_id: Equipment number
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_to_external_epc(self, epc_address):
        """
        This function permits to connect to an external
        epc (LTE) if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_from_external_epc(self):
        """
        This function permits to disconnect from an external
        epc (LTE) if good connection parameters are set
        :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_external_epc_disconnection(self, disconnection_time=0):
        """
        This function tests the external EPC disconnection before specified timeout

        :type disconnection_time: integer
        :param disconnection_time: Disconnection timeout
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_external_epc_connected(self, external_ip=None):
        """
        This function tests the external EPC connection.

        :type external_ip: str
        :param external_ip: Ip address of the waited external device

        :rtype: boolean
        :return: True if the external device is connected, else return False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_external_epc_disconnected(self):
        """
        This function tests the external EPC disconnection.

        :rtype: boolean
        :return: True if the external device is disconnected, else return False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_external_epc_connection(self, external_ip=None, connection_time=0):
        """
        This function tests the external EPC connection before specified timeout

        :type external_ip: str
        :param external_ip: Ip address of the waited external device
        :type connection_time: integer
        :param connection_time: Connection timeout
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_external_epc_connection_status(self):
        """
        This function gets the External EPC Conn Status .

        :rtype: str
        :return: The connection status of the EPC
                 Status :
                    - CONN : Connected
                    - NCON : Not Connected
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_external_epc_connected_ip_address(self):
        """
        This function gets the connected external EPC IP address

        :rtype: str
        :return: The connected EPC IP address
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_ho_message(self, number):
        """
        Send Handover message
        :type number: int
        :param number: the desired handover message to send.
        Possible values:
                - 1 : DL info CS Notify
                - 2 : B1 RRC Handover A to B
                - 3 : B3 RRC Handover A to B
                - 4 : B13 RRC Handover A to B
                - 5 : B17 RRC Handover A to B
                - 6 : PS HO To WCDMA
                - 7 : Blind HO
                - 8 : SRVCC To WCDMA
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_ims(self):
        """
        Starts IMS virtual network
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_ims(self, bench_config):
        """
        Applies IMS configuration provided in the Bench Config
        :type bench_config: BenchConfigParameters
        :param bench_config: Bench config parameters
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ims_cscf_state(self):
        """
        Gets IMS CSCF state

        :rtype: str
        :return: IMS CSCF state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)