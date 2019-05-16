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
:summary: Data LTE implementation for R&S CMW500 using visa interface
:since: 16/09/2014
:author: jduran4x
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData4G import IData4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.Data import Data as DataCommon


class Data4G(IData4G, DataCommon):
    """
    Data LTE implementation for Rohde & Schwarz CMW500
    This class inherit from the IData4G interface and from the Data class
    containing all equipment command that available independently from the
    technology used.
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        DataCommon.__init__(self, visa)

    def config_window_size_off(self):
        """
        Disables the setting of the window size on the CMW500
        """
        DataCommon.disable_window_size_setting(self)

    def ue_detach(self):
        """
        Sends the specified DETACH message contained in the scenario
        """
        self.set_connection_setup("DET")

    def get_data_connection_status(self, cell_id):
        """
        GetDataConnectionStatus
        :param cell_id: id of the cell for which the status is being requested
        :type cell_id: str
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
        :raise TestEquipmentException: if cell_id value is incorrect

        [JPHC, 28 juillet 2012]
        Need to retrieve current connection status from R&S (much finer detail reported and translate to the PXT language for reporting back to UC)
        """

        if cell_id == "A":
            status = self._visa.query_command("FETC:LTE:SIGN1:PSW:STAT?")
        elif cell_id == "B":
            status = self._visa.query_command("FETC:LTE:SIGN2:PSW:STAT?")
        elif cell_id == "" or cell_id is None:
            status = self._visa.query_command("FETC:LTE:SIGN:PSW:STAT?")
        else:
            msg = "Get Data connection status as failed due to invalid cell-id"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

        if status == "OFF" or status == "SMES" or status == "RMES" or status == "IHAN" or status == "OHAN":
            # there is no OFF state as such in the PXT since RF output control is
            # decoupled from signaling status... also capturing here other CMW500 not
            # present in PXT, although semantics are not respected
            status = "UNAV"
        elif status == "ON":
            # a situation in which RF signal is being transmitted but the UE has not
            # yet registered is considered ON in CMW500 and OFF for PXT
            status = "OFF"
        elif status == "SIGN":
            # for CMW500 there are some transition states that do not appear in PXT...
            # think that attach process is signaled by the SIGN but we may need to
            # change this
            status = "REG"
        elif status == "CONN":
            status = "IDLE"
        elif status == "CEST" or status == "ATT":
            status = "CON"  # CMW500 CEST is the equivalent to PXT CON
        elif status == "DISC":
            status = "REL"  # CMW500 DISC is the equivalent to PXT REL

        return status

    def get_network_type(self):
        """
        Returns the expected network type
        :rtype: str
        :return: the expected network type
        """
        network_type = "LTE"

        return network_type

    def get_su_id(self, su_list):
        """
        Set signaling unit wanted from a list of signalling unit
        :type su_list: str list
        :param su_list: list of available signalling units
        """
        if '"LTE Sig1"' in su_list:
            self.su_id = '"LTE Sig1"'
        else:
            self.su_id = ""

    def get_su_state(self):
        """
        Returns the cell state
        :rtype: str
        :return: cell state (ON|OFF)
        """
        return self._visa.query_command("SOUR:LTE:SIGN:CELL:STAT?")

    def set_connection_setup(self, conn_setup):
        """
        To initiate a connection setup
        :type state: str
        :param state: the state expected. Possible values :
                - "CONN" : Initiate a mobile terminated connection setup
                - "HAND" : Initiate a handover
                - "DET" : Initiate a Detach procedure
                - "SMS" : Send a SMS
        """
        self.get_logger().info("Configuring a connection setup of type %s" % conn_setup)
        self._visa.send_command("CALL:LTE:SIGN:PSW:ACT %s" % conn_setup)

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
            output = self._visa.query_command("SENSe:LTE:SIGN:UESinfo:UEADdress:IPV6?")
        elif ip_version == "IPV4":
            output = self._visa.query_command("SENSe:LTE:SIGN:UESinfo:UEADdress:IPV4?")
        elif ip_version == "IPV4V6":
            output_v6 = self._visa.query_command("SENSe:LTE:SIGN:UESinfo:UEADdress:IPV6?")
            output_v4 = self._visa.query_command("SENSe:LTE:SIGN:UESinfo:UEADdress:IPV4?")
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

    def get_ip_address_type(self):
        """
        gets what type of IP address is assigned to the DUT

        :rtype: str
        :return: The Type of IP version set on the NW

        :raise TestEquipmentException: if the type of IP version is invalid
        """
        # returns the type of IP address assigned to DUT
        ip_version = self._visa.query_command("CONFigure:LTE:SIGN:CONNection:IPVersion?")

        if ip_version == "IPV46":
            ip_version = "IPV4V6"

        if ip_version in ("IPV4", "IPV6", "IPV4V6"):
            self.get_logger().info("the type of IP assigned is: %s" % ip_version)
            return ip_version
        else:
            msg = "%s  is an invalid IP_VERSION. Possible values are: IPV4, IPV6 and IPV4V6 only" % ip_version
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         msg)

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
                ip_version = "IPV46"

            self._visa.send_command("CONFigure:LTE:SIGN:CONNection:IPVersion %s" % ip_version)
        else:
            msg = "%s  is an invalid IP_VERSION. Possible values are: IPV4, IPV6 and IPV4V6 only" % ip_version
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         msg)

    def get_ext_bler_measurement(self, direction, duration):
        """
        Perform extended BLER (Block Error Rate) measurement and return BLER value
        :type direction: str
        :param direction: BLER measurement type (UL|DL)
        :type duration: int
        :param duration: BLER measurement duration in seconds

        :rtype: float
        :return: measured BLER in %
        """
        # Abort previous BLER measurement
        self._visa.send_command("ABORt:LTE:SIGN:EBLer")
        self.get_logger().info("Configuring extended BLER for %d s" % duration)
        # Perform a single shot BLER measurement during duration in number of subframe (1 subframe=1ms)
        self._visa.send_command("CONFigure:LTE:SIGN:EBLer:SCONdition NONE")
        self._visa.send_command("CONFigure:LTE:SIGN:EBLer:REPetition SING")
        self._visa.send_command("CONFigure:LTE:SIGN:EBLer:SFRames %d" % (duration * 1000))
        self._visa.send_command("CONFigure:LTE:SIGN:EBLer:ERCalc ERC1")

        self.get_logger().info("Starting extended BLER measurement for %d s" % duration)
        self._visa.send_command("INIT:LTE:SIGN:EBLer")
        # Wait end of measurement
        time.sleep(duration + 10)
        self.get_logger().info("Get extended %s BLER measurement" % (direction + "link"))
        if direction == "down":
            # Get all result from BLER measurement
            result = self._visa.query_command("FETCh:LTE:SIGN:EBLer:RELative?")
            result = result.split(',')
            self.get_logger().debug("BLER measurement result %s" % result)
            self._visa.send_command("ABORt:LTE:SIGN:EBLer")
            # check if measurement is valid
            if int(result[0]) != 0:
                msg = "BLER measurement failed! Status: %s: %s" % (result[0], self._reliability_indicator[int(result[0])])
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)
            # Get BLER from result
            bler = float(result[3])
        else:
            result = self._visa.query_command("FETCh:LTE:SIGN:EBLer:UPLink?")
            result = result.split(',')
            self.get_logger().debug("UL BLER measurement result %s" % result)
            self._visa.send_command("ABORt:LTE:SIGN:EBLer")
            # check if measurement is valid
            if int(result[0]) != 0:
                msg = "UL BLER measurement failed! Status: %s: %s" % (result[0], self._reliability_indicator[int(result[0])])
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)
            # Get BLER from result
            bler = (float(result[4]) / (float(result[3]) + float(result[4]))) * 100.0
        self.get_logger().debug("BLER measured %.2f" % bler)

        return bler
