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
:summary: data 2G implementation for Agilent 8960 cellular network simulator
:since: 08/03/2011
:author: ymorel
"""

from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData2G import \
    IData2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Wrapper.Tech2G import \
    WData2G as W
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class Data2G(IData2G):

    """
    Data 2G implementation for Agilent 8960
    """

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        IData2G.__init__(self)
        self.__root = root

    def __error_check(self, err, msg, acs_msg=None):
        """
        Error checking and warning reporting

        :type err: int
        :param err: the error code of the driver function

        :type msg: str
        :param msg: the driver message

        :type acs_msg: str
        :param acs_msg: a more accurate message that replaces driver eventual insufficient message

        :raise TestEquipmentException: if err < 0
        """
        if acs_msg is not None and acs_msg != "":
            end_msg = acs_msg
        else:
            end_msg = msg

        if err < 0:
            self.get_logger().error(end_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, end_msg)
        elif err > 0:
            self.get_logger().debug(end_msg)

    def get_root(self):
        """
        Gets the root of the equipment
        :rtype: Agilent8960
        :return: the root of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the root logger
        """
        return self.get_root().get_logger()

    def set_dut_ip_address(self, ip_num, ip_addr):
        """
        Sets the DUT IP address.
        :type ip_num: integer
        :param ip_num: number of the IP address to set (1 to 4).
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        (err, msg) = W.SetDutIpAddress(self.get_root(), ip_num, ip_addr)
        self.__error_check(err, msg)

    def clear_all_dut_ip_addresses(self):
        """
        Clears all configured DUT IP addresses and sets them to " "
        during the clear process, the network simulator may complain because of no IP addresses assigned
        """
        # Agilent8960 has four DUT IP addresses
        for ip_num in range(1, 5):
            (err, msg) = W.SetDutIpAddress(self.get_root(), ip_num, "")
            self.__error_check(err, msg)
            self.get_logger().info("the network simulator ip_addresses are temporary cleared, please ignore the previous warning displayed")

    def get_dut_ip_address(self, ip_num):
        """
        Gets the DUT IP address.
        :type ip_num: integer
        :param ip_num: the number of the IP address to return (1 to 4).
        :rtype: str
        :return: the IP address of the DUT.
        """
        (err, ip_addr, msg) = W.GetDutIpAddress(self.get_root(), ip_num)
        self.__error_check(err, msg)
        return ip_addr

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        (err, msg) = W.SetDutPrimaryDns(self.get_root(), ip_addr)
        self.__error_check(err, msg)

    def get_dut_primary_dns(self):
        """
        Gets DUT primary DNS address.
        :rtype: str
        :return: the IP address of the primary DNS of the DUT.
        """
        (err, ip_addr, msg) = W.GetDutPrimaryDns(self.get_root())
        self.__error_check(err, msg)
        return ip_addr

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        (err, msg) = W.SetDutSecondaryDns(self.get_root(), ip_addr)
        self.__error_check(err, msg)

    def get_dut_secondary_dns(self):
        """
        Gets DUT secondary DNS address.
        :rtype: str
        :return: the IP address of the device under test secondary DNS
        """
        (err, ip_addr, msg) = W.GetDutSecondaryDns(self.get_root())
        self.__error_check(err, msg)
        return ip_addr

    def get_data_connection_status(self):
        """
        Gets data connection status.
        :rtype: str
        :return: the data connection status. Possible returned values:
            - "IDLE"
            - "ATTACHING"
            - "DETACHING"
            - "ATTACHED"
            - "STARTING"
            - "ENDING"
            - "TRANSFERRING"
            - "PDP_ACTIVATING"
            - "PDP_ACTIVE"
            - "PDP_DEACTIVATING"
            - "CS_DATA_CON_ACTIVE"
            - "SUSPENDED"
        """
        (err, status, msg) = W.GetDataConnectionStatus(self.get_root())
        self.__error_check(err, msg)
        return status

    def set_initial_ps_data_rrc_state(self, state):
        """
        Sets initial PS data RRC state.
        :type state: str
        :param state: the desired state. Possible values:
            - "DCH"
            - "FACH"
        """
        (err, msg) = W.SetInitialPsDataRRCState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_connection_type(self, conn_type):
        """
        Sets the connection type.
        :type conn_type: str
        :param conn_type: the connection type to set.
        Possible values:
            - "A"
            - "B"
            - "ACKB"
            - "BLER"
            - "AUTO"
            - "SRBL"
        """
        (err, msg) = W.SetConnectionType(self.get_root(), conn_type)
        self.__error_check(err, msg)

    def set_dtm_multislot_config(self, config):
        """
        Sets DTM multislot configuration.
        :type config: str
        :param config: the DTM multislot configuration to set. Possible values:
            - "D1U1" | "D1U2" | "D1U3" | "D1U4" | "D1U5" | "D1U6"
            - "D2U1" | "D2U2" | "D2U3" | "D2U4" | "D2U5"
            - "D3U1" | "D3U2" | "D3U3" | "D3U4"
            - "D4U1" | "D4U2" | "D4U3"
            - "D5U1" | "D5U2"
            - "D6U1"
            - "CUST"
        """
        (err, msg) = W.SetDtmMultislotConfig(self.get_root(), config)
        self.__error_check(err, msg)

    def set_pdtch_modulation_coding_scheme(self, downlink, uplink):
        """
        Sets the PDTCH modulation coding scheme.
        :type downlink: str
        :param downlink: the downlink coding scheme to set. Possible values:
            - "MCS1" | "MCS2" | "MCS3" | "MCS4" | "MCS5" | "MCS6" |
              "MCS7" | "MCS8" | "MCS9"
        :type uplink: str
        :param uplink: the uplink coding scheme to set. Possible values:
            - "MCS1" | "MCS2" | "MCS3" | "MCS4" | "MCS5" | "MCS6" |
              "MCS7" | "MCS8" | "MCS9"
        """
        (err, msg) = W.SetPdtchModulationCodingScheme(
            self.get_root(),
            downlink,
            uplink)
        self.__error_check(err, msg)

    def set_pdtch_puncturing_scheme(self, scheme):
        """
        Sets the PDTCH puncturing scheme.
        :type scheme: str
        :param scheme: the PDTCH puncturing scheme to set. Possible values:
            - "PS1"
            - "PS2"
            - "PS3"
        """
        (err, msg) = W.SetPdtchPuncturingScheme(self.get_root(), scheme)
        self.__error_check(err, msg)

    def set_pdtch_uplink_coding_scheme(self, scheme):
        """
        Sets the PDTCH uplink coding scheme.
        :type scheme: str
        :param scheme: the PDTCH uplink coding scheme to set. Possible values:
            - "CS1"
            - "CS2"
            - "CS3"
            - "CS4"
        """
        (err, msg) = W.SetPdtchUplinkCodingScheme(self.get_root(), scheme)
        self.__error_check(err, msg)

    def set_pdtch_downlink_coding_scheme(self, scheme):
        """
        Sets PDTCH downlink coding scheme.
        :type scheme: str
        :param scheme: the PDTCH downlink coding scheme to set. Possible values:
            - "CS1"
            - "CS2"
            - "CS3"
            - "CS4"
        """
        (err, msg) = W.SetPdtchDownlinkCodingScheme(self.get_root(), scheme)
        self.__error_check(err, msg)

    def set_multislot_config(self, config):
        """
        Sets multislot configuration.
        :type config: str
        :param config: the multislot configuration to set. Possible values:
            - "D1U1" | "D1U2" | "D1U3" | "D1U4" | "D1U5" | "D1U6"
            - "D2U1" | "D2U2" | "D2U3" | "D2U4" | "D2U5"
            - "D3U1" | "D3U2" | "D3U3" | "D3U4"
            - "D4U1" | "D4U2" | "D4U3"
            - "D5U1" | "D5U2"
            - "D6U1"
            - "CUST"
        """
        (err, msg) = W.SetMultislotConfig(self.get_root(), config)
        self.__error_check(err, msg)

    def set_custom_multislot_config(
        self,
        main_timeslot,
        dl_slot_enabled,
        dl_slot_level,
        ul_slot_enabled,
            ul_slot_gamma):
        """
        Sets a custom multislot configuration.
        :type mainTimeslot: integer
        :param mainTimeslot: The number of the main time slot (0 to 7).
        :type dlSlotEnabled: str
        :param dlSlotEnabled: a str of 8 "ON" | "OFF" words separated by ','.
        :type dlSlotLevel: str
        :param dlSlotLevel: a str of 8 levels in dB separated by ','
        (each level is a double from -127.0 to +127.0).
        :type ulSlotEnabled: str
        :param ulSlotEnabled: a str of 8 "ON" | "OFF" words separated by ','.
        :type ulSlotGamma: str
        :param ulSlotGamma: a str of 8 gamma power control separated by ','
        (each gamma is an integer from 0 to 31).
        """
        (err, msg) = W.SetCustomMultislotConfig(
            self.get_root(),
            main_timeslot,
            dl_slot_enabled,
            dl_slot_level,
            ul_slot_enabled,
            ul_slot_gamma)
        self.__error_check(err, msg)

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id=None):
        """
        Checks that the data connection is set at the required state
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
            - "ATTACHED"
            - "PDP_ACTIVE"
            - "TRANSFERRING"
            - "SUSPENDED"
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        :type blocking: boolean
        :param blocking: boolean to know if the function raises an error
        or simply return true or false if the status is reached or not
        :rtype: boolean
        :return: True if state is reached, else returns False
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        """
        attached = False
        timer = timeout
        self.get_logger().info(
            "Check data connection is %s before %d seconds",
            state,
            timeout)

        (err, current_state, msg) = W.GetDataConnectionStatus(self.get_root())
        self.__error_check(err, msg)

        while (timer > 0) and (current_state != state):
            if state == "ATTACHED" and current_state == "PDP_ACTIVE":
                attached = True
                break
            time.sleep(1)
            (err, current_state, msg) = \
                W.GetDataConnectionStatus(self.get_root())
            self.__error_check(err, msg)
            timer -= 1

        if current_state != state and not attached:
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
            self.get_logger().info("Data connection is %s and has been reached in %d seconds" % (current_state, timeout - timer))
            return True

    def set_pdtch_coding_scheme(self, dl_coding_scheme, ul_coding_scheme):
        """
        Sets the downlink and uplink coding scheme
        :type dl_coding_scheme: str
        :param dl_coding_scheme: the downlink coding scheme to set
        :type ul_coding_scheme: str
        :param ul_coding_scheme: the uplink coding scheme to set
        """
        # Setting downlink coding scheme
        (err, msg) = W.SetPdtchDownlinkCodingScheme(
            self.get_root(),
            dl_coding_scheme)
        err_msg = "Cannot set DL coding scheme (%s)" % msg
        self.__error_check(err, msg, err_msg)

        # Setting uplink coding scheme
        (err, msg) = W.SetPdtchUplinkCodingScheme(
            self.get_root(),
            ul_coding_scheme)
        err_msg = "Cannot set UL coding scheme (%s)" % msg
        self.__error_check(err, msg, err_msg)

    def check_data_connection_transferring(self, timeout, check_ota_throughput=False, blocking=True):
        """
        Check data connection, looking for "suspended" state.

        :type timeout: integer
        :param timeout: allowed time in seconds to reach 'suspended' state

        :type check_ota_throughput: boolean
        :param check_ota_throughput: boolean defining if check is done on Over The Air
        throughputs (True) or IP throughput (False).

        :type blocking: boolean
        :param blocking: boolean defining if the function is blocking (returns
        an Exception) or not (returns True or False)

        :rtype: boolean
        :return: True if state is reached, else returns False

        .. warning:: check_ota_throughput parameter only used for 3g cells.
        """
        return self.check_data_connection_state("TRANSFERRING", timeout, blocking)

    def check_data_connection_transferring_time(self, transferring_timeout, transferring_time, check_ota_throughput = False, blocking = True):
        """
        Check data connection, looking for "transferring" state.

        :type transferring_timeout: integer
        :param transferring_timeout: allowed time in seconds to reach 'transferring' state

        :type transferring_time: integer
        :param transferring_time: time in seconds of continuous transfer

        :type check_ota_throughput: boolean
        :param check_ota_throughput: boolean defining if check is done on Over The Air
        throughputs (True) or IP throughput (False).

        :type blocking: boolean
        :param blocking: boolean defining if the function is blocking (returns
        an Exception) or not (returns True or False)

        :rtype: boolean
        :return: True if state is reached, else returns False

        .. warning:: check_ota_throughput set to True will
        test connection by checking OTA throughputs (Tx and Rx),
        whereas check_ota_throughput set to False will test connection
        by checking IP throughputs (Tx and Rx).
        """
        timer = transferring_timeout
        self.get_logger().info(\
            "Check if there is a continuous data transfer during %d seconds before %d seconds" \
            % (transferring_time, transferring_timeout))

        (err, current_state, msg) = W.GetDataConnectionStatus(self.get_root())
        self.__error_check(err, msg)
        transfer_count = 0

        while timer > 0:
            time.sleep(1)
            (err, current_state, msg) = \
                W.GetDataConnectionStatus(self.get_root())
            self.__error_check(err, msg)

            if(current_state == "TRANSFERRING"):
                transfer_count += 1
            else:
                transfer_count = 0
            timer -= 1
            if transfer_count >= transferring_time:
                self.get_logger().info("Continuous data transfer during %d seconds has been reached", transferring_time)
                return True

        if blocking:
            # Failed to transfer data continuously
            msg = "Failed to transfer data continuously during %d seconds!" % transferring_time
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
        else:
            # Failed to transfer data continuously (Test failed no TestEquipmentException raised)
            self.get_logger().error("Failed to transfer data continuously during %d seconds!" % transferring_time)
        return False

    def check_data_connection_suspended(self, timeout, blocking=True):
        """
        Check data connection, looking for "suspended" state.

        :type timeout: integer
        :param timeout: allowed time in seconds to reach 'suspended' state

        :type check_ota_throughput: boolean
        :param check_ota_throughput: boolean defining if check is done on Over The Air
        throughputs (True) or IP throughput (False).

        :type blocking: boolean
        :param blocking: boolean defining if the function is blocking (returns
        an Exception) or not (returns True or False)

        :rtype: boolean
        :return: True if state is reached, else returns False

        .. warning:: check_ota_throughput parameter only used for 3g cells.
        """
        return self.check_data_connection_state("SUSPENDED", timeout, blocking)

    def get_network_type(self):
        """
        Returns the expected network type

        :rtype: str
        :return: the expected network type.
        """
        # Get 2G cell service type on network simulator
        (err, network_type, msg) = W.GetServingCell(self.get_root())
        self.__error_check(err, msg)
        self.get_logger().info("Cell Service is %s", network_type)

        return network_type
