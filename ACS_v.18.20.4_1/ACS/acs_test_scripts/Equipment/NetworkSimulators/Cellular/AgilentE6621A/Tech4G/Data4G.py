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
:summary: Data LTE implementation for Agilent E6621A
:since: 27/04/2012
:author: lvacheyx
.. note:: BZ3071 - PING MO over LTE network
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData4G import IData4G
from ErrorHandling.AcsBaseException import AcsBaseException


class Data4G(IData4G):

    """
    Data LTE implementation for Agilent E6621A (PXT)
    """

    def __init__(self, root, cell_name="A"):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent6621A)
        """
        IData4G.__init__(self)
        self.__root = root
        self.__cell_name = cell_name
        if self.__cell_name is "A":
            self.__id = 1
        elif self.__cell_name is "B":
            self.__id = 2
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                    "Parameter \"%s\" is not valid for Data4g" % str(cell_name))

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

    def get_root(self):
        """
        Gets the root of the equipment
        :rtype: Agilent8960
        :return: the root of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def ue_detach(self):
        """
        Sends the specified DETACH message contained in the scenario
        """
        self.get_logger().info("Send UE detach message")
        self.__root.send_command("BSE:FUNC:UEDET:MESSA1:SEND")

    def get_data_connection_status(self, cell_id):
        """
        GetDataConnectionStatus
        :return:
            - integer: error code of the driver function
            - str: the data connection status. Possible returned values:
                - "OFF": Default returned value
                - "IDLE"
                - "CON"
                - "REG"
                - "LOOP"
                - "REL"
                - "UNAV"
                - "UNAV" => Default returned value
        """

        if cell_id == "A":
            status = self.__root.query_command("BSE:STAT:ACELL?")
        elif cell_id == "B":
            status = self.__root.query_command("BSE:STAT:BCELL?")
        else:
            msg = "Get Data connection status as failed due to invalid cell-id"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

        return status

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id="A"):
        """
        Checks that the data connection is set at the required state
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
                - "OFF": Default returned value
                - "IDLE"
                - "CON"
                - "REG"
                - "LOOP"
                - "REL"
                - "UNAV"
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        :type blocking: boolean
        :param blocking: boolean to know if the function raises an error
        or simply return true or false if the status is reached or not
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        :rtype: boolean
        :return: True if state is reached, else returns False
        """
        registered = False
        idle = False
        timer = int(timeout)
        self.get_logger().info(
            "Check data connection is %s before %d seconds",
            state,
            timeout)

        current_state = self.get_data_connection_status(cell_id)

        while (timer > 0) and (current_state != state):
            if state == "REG" and current_state == "IDLE":
                registered = True
                break
            elif state == "IDLE" and current_state == "CON":
                idle = True
                break
            time.sleep(1)
            current_state = self.get_data_connection_status(cell_id)
            timer -= 1

        if current_state != state and not registered and not idle:
            if blocking:
                # Failed to reach desired state
                msg = "Failed to reach %s data state!" % state
                self.get_logger().error(msg)

                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # Failed to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().error("Failed to reach %s data state!", state)

            return False
        else:
            self.get_logger().info("Data connection status %s and has been reached in %d sec", current_state, int(timeout) - int(timer))
            return True

    def set_epc_on(self):
        """
        Set the Evolved Packet Core (EPC) to Embedded (Embed) to enable IP connectivity :
        IP traffic routing, accepting uplink IP data from the UE and putting it on to
        the network. Similarly for the downlink, it acts as a proxy for UE IP addresses,
        accepting and forwarding IP data to the stack.
        It is necessary to perform end-to-end functional tests.
        Otherwise, all uplink IP data will be discarded.
        """
        # Enable EPC application to run on the PXT
        self.get_logger().info("Enable EPC application to run on the PXT")
        self.__root.send_command("BSE:EPC EMB")

    def set_epc_off(self):
        """
        Set the Evolved Packet Core (EPC) off when you are doing RF transmitter
        or receiver measurements
        """
        # Disable EPC application to run on the PXT
        self.get_logger().info("Disable EPC application to run on the PXT")
        self.__root.send_command("BSE:EPC OFF")

    def get_network_type(self):
        """
        Returns the expected network type
        :rtype: str
        :return: the expected network type
        """
        network_type = "LTE"

        return (network_type)

    def set_connection_setup(self, conn_setup):
        """
        Stub to align with RS CMW500 interface
        """
        self.get_logger().info("set_connection_setup function is stubbed for AgilentE6621A")
        pass

    def start_ftp_service(self):
        """
        Stub to align with RS CMW500 interface
        """
        self.get_logger().info("start_ftp_service function is stubbed for AgilentE6621A")
        pass

    def stop_ftp_service(self):
        """
        Stop the FTP server in the DAU.
        """
        self.get_logger().info("stop_ftp_service function is stubbed for AgilentE6621A")
        pass

    def set_apn(self, apn):
        """
        Set APN on Cellular network.

        :type apn: str
        :param apn: The Access Point Name to be configured on equipment

        :rtype: None
        """
        self.get_logger().info("Set APN on Cellular network")
        self.__root.send_command("BSE:CONF:NAS:DEF:EBC%s:APN \"%s\"" % (self.__id, apn))

    def get_data_throughput(self, duration, type_list):
        """
        Get the data throughput during the duration time from PXT

        :type duration: Int
        :param duration: measurement duration

        :type type_list: String list
        :param type_list: The type of throughput to measure: UL, DL

        :rtype: dict
        :return: the dictionary of test result:
        Average data transfer (bytes)
        """
        data_throughput_dict = {}

        # Init data measurement
        start_measure_cmd = "BSE:ER:THROU:MEAS:TABL:CLE"
        self.__root.send_command(start_measure_cmd)
        self.get_logger().info("Start data throughput measurement")

        time.sleep(duration)

        for meas_type in type_list:
            if meas_type == "DL":
                get_throughput_cmd = "BSE:ER:THROUghput:MEASure:TABLe?"
            elif meas_type == "UL":
                get_throughput_cmd = "BSE:ER:THROUghput:UL:MEASure:TABLe?"
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Wrong formating of the throughput dict.")
            # The result is in a comma separated str with value in the Following order
            # Current data throughput(bps) ,Min data throughput(bps),
            # Max data throughput(bps), Average data transfer(bytes)
            results = self.__root.query_command(get_throughput_cmd)
            self.get_logger().debug(results)
            # we want just to retrieve the average data throughput
            result = results.split(',')[-1]

            data_throughput_dict.update({meas_type: result})

        return data_throughput_dict

    def set_dut_ip_address(self, ip_addr):
        """
        Sets the IP address assigned to the DUT
        we can assign an IPv4 address, an IPv6 address according of the format
        of the IP passed as parameter.
        In order to set both IPV4 and IPV6 addresses please call this function
        twice: once with the IPV4 as parameter and once with an IPV6 address
        as parameter.

        :type ip_addr: str
        :param ip_addr: the value of the IPV4 or IPV6 address
        """
        is_ipv6 = False
        if ":" in ip_addr:
            is_ipv6 = True
        # in the case of IPV6 address
        if is_ipv6:
            self.get_logger().info("Set the IPV6 address to: \"%s\" on Cell-%s"
                                   % (ip_addr, self.__cell_name))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC%s:ADDR:TYPE IPV6" % str(self.__id))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC%s:ADDR:V6 \"%s\""
                                     % (str(self.__id), ip_addr))
        # in the case of IPV4 address
        else:
            self.get_logger().info("Set the IPV4 address to: \"%s\" on Cell-%s"
                                   % (ip_addr, self.__cell_name))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC%s:ADDR:TYPE IPV4" % str(self.__id))
            # Set IP address for both EPS bearer 1 and 2
            # As windows device camp by default on EPS bearer 2
            # See ST BZ5296 for more details
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC1:ADDR:V4 \"%s\""
                                     % (ip_addr))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC2:ADDR:V4 \"%s\""
                         % (ip_addr))

    def get_ip_address(self, ip_version, blocking=True):
        """
        gets what IP address is assigned to the DUT
        we can assign an IPv4 address, an IPv6 address,
        or both an IPv4 and IPv6 to the UE

        :type ip_version: str
        :param ip_version: version to use when establishing the connection.
        Possible values: IP4, IP6, IPV4V6.

        :type blocking: boolean
        :param blocking: boolean to know if the function raises an error
        or simply return a message if no IP address is assigned

        :rtype: str
        :return: The IP address assigned for the IP version requested

        :raise TestEquipmentException: if the IP address is not assigned for the IP version requested
        """
        output = ""

        if ip_version == "IPV6":
            output = self.__root.query_command("BSE:CONF:NAS:DEF:EBC1:ADDR:V6?")
        elif ip_version == "IPV4":
            output = self.__root.query_command("BSE:CONF:NAS:DEF:EBC1:ADDR:V4?")
        elif ip_version == "IPV4V6":
            output_v6 = self.__root.query_command("BSE:CONF:NAS:DEF:EBC1:ADDR:V6?")
            output_v4 = self.__root.query_command("BSE:CONF:NAS:DEF:EBC1:ADDR:V4?")
            output = output_v4 + "for IPV4" + output_v6 + " for IPV6"

        if output not in (None, ""):
            self.get_logger().info("the value of %s assigned is: %s" % (ip_version, output))
            return output

        else:
            msg = "No %s address assigned" % ip_version
            self.get_logger().error(msg)
            if blocking:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
            else:
                return output

    def set_ip_address_type(self, ip_version):
        """
        sets the type of IP address assigned to the DUT
        we can assign an IPv4 address, an IPv6 address,
        or both an IPv4 and IPv6 to the UE

        :type ip_version: str
        :param ip_version: version to use when establishing the connection.
        Possible values: IPV4, IPV6, IPV4V6.

        :raise TestEquipmentException: if the type of IP version is invalid
        """
        # Set the IP address depending on the version chosen
        if ip_version in ("IPV4", "IPV6", "IPV4V6"):
            self.get_logger().info("Set the IP address type to: %s" % ip_version)
            if ip_version == "IPV4V6":
                ip_version = "V4V6"
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC1:ADDR:TYPE %s" % ip_version)

        else:
            msg = "%s  is an invalid IP_VERSION. Possible values are: IPV4, IPV6 and IPV4V6 only" % ip_version
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         msg)

    def get_ip_address_type(self):
        """
        gets what type of IP address is assigned to the DUT

        :rtype: str
        :return: The Type of IP version set on the NW

        :raise TestEquipmentException: if the type of IP version is invalid
        """
        # returns the type of IP address assigned to DUT
        output = self.__root.query_command("BSE:CONF:NAS:DEF:EBC1:ADDR:TYPE?")

        if output in ("IPV4", "IPV6", "V4V6"):
            if output == "V4V6":
                output = "IPV4V6"
            self.get_logger().info("the type of IP assigned is: %s" % output)
            return output
        else:
            msg = "%s  is an invalid IP_VERSION. Possible values are: IPV4, IPV6 and IPV4V6 only" % output
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         msg)

    def configure_eps_bearer_config_2(self, apn, ip_version, ip_addr):
        """
        Sets the APN, IP version and IP address for the Default EPS Bearer Config #2

        :type apn: str
        :param apn: The Access Point Name to be configured on equipment
        :type ip_version: str
        :param ip_version: version to use when establishing the connection.
        Possible values: IPV4, IPV6, IPV4V6.
        :type ip_addr: str
        :param ip_addr: the value of the IPV4 or IPV6 address

        :raise TestEquipmentException: if the type of IP version is invalid
        """

        # Set the APN for EPS Bearer Config 2
        self.get_logger().info("Set APN for EPS bearer config 2 on Cellular network")
        self.__root.send_command("BSE:CONF:NAS:DEF:EBC2:APN \"%s\"" % (apn))

        # Set the IP address depending on the version chosen for EPS Bearer Config 2
        if ip_version in ("IPV4", "IPV6", "IPV4V6"):
            self.get_logger().info("Set the IP address type to: %s for EPS Bearer Config 2" % ip_version)
            if ip_version == "IPV4V6":
                ip_version = "V4V6"
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC2:ADDR:TYPE %s" % ip_version)

        else:
            msg = "%s  is an invalid IP_VERSION. Possible values are: IPV4, IPV6 and IPV4V6 only" % ip_version
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         msg)

        # Sets the IP address for EPS Bearer Config 2
        is_ipv6 = False

        if ":" in ip_addr:
            is_ipv6 = True
        # in the case of IPV6 address
        if is_ipv6:
            self.get_logger().info("Set the IPV6 address to: \"%s\"" % (ip_addr))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC2:ADDR:V6 \"%s\"" % (ip_addr))
        # in the case of IPV4 address
        else:
            self.get_logger().info("Set the IPV4 address to: \"%s\"" % (ip_addr))
            self.__root.send_command("BSE:CONF:NAS:DEF:EBC2:ADDR:V4 \"%s\"" % (ip_addr))

    def set_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed by:
        IPv6 Prefix and IPv6 ID
        """
        self.get_logger().info("set_ip6_network_parameters function is stubbed for AgilentE6621A")
        pass

    def set_ip_addr_configuration_mode(self, mode, ip_type="ALL"):
        """
        Selects the method to be used for IP address configuration

        :type mode: str
        :param mode: STATIC - ip take from what has been manually defined,
                     AUTOMATIC - ip assignment from a default configuration
                     DYNAMIC - dynamic mode , DHCP for ipv4 or auto configuration for ipv6

        :type ip_type: str
        :param ip_type: the ip type, IPV4, IPV6 or dont fill it to force all type.
        """
        self.get_logger().info("set_ip_addr_configuration_mode function is stubbed for AgilentE6621A")
        pass