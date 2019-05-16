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
:summary: data 3G implementation for Agilent 8960
:since: 01/04/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData3G import IData3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Wrapper.Tech3G import WData3G as W


class Data3G(IData3G):

    """
    Data 3G implementation for Agilent 8960
    """

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        self.__root = root

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().debug(msg)

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

    def set_dut_ip_address(self, ip_num, ip_addr):
        """
        Sets DUT IP address.
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
        # Agilent8960 has two DUT IP addresses
        for ip_num in range(1, 3):
            (err, msg) = W.SetDutIpAddress(self.get_root(), ip_num, "")
            self.__error_check(err, msg)
            self.get_logger().info("the network simulator ip_addresses are temporary cleared, please ignore the previous warning displayed")

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        (err, msg) = W.SetDutPrimaryDns(self.get_root(), ip_addr)
        self.__error_check(err, msg)

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS IP address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        (err, msg) = W.SetDutSecondaryDns(self.get_root(), ip_addr)
        self.__error_check(err, msg)

    def set_initial_ps_data_rrc_state(self, state):
        """
        Sets the initial PS data RRC state.
        :type state: str
        :param state: the desired state. Possible values:
            - "DCH"
            - "FACH"
        """
        (err, msg) = W.SetInitialPsDataRRCState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_category_control_mode(self, mode):
        """
        Sets category control mode.
        :type mode: str
        :param mode: the desired mode. Possible values:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetCategoryControlMode(self.get_root(), mode)
        self.__error_check(err, msg)

    def set_macd_pdu_size(self, size):
        """
        Sets ...
        :type size: str
        :param size: the desired mode. Possible values:
            - "BITS336"
            - "BITS656"
        """
        (err, msg) = W.SetMacDPduSize(self.get_root(), size)
        self.__error_check(err, msg)

    def get_hsdpa_reported_category(self):
        """
        Gets HSDPA reported category
        :rtype: integer
        :return: the HSDPA reported category, an integer from 1 to 64,
        -1 if equipment cannot report the value.
        """
        (err, cat, msg) = W.GetHsdpaReportedCategory(self.get_root())
        self.__error_check(err, msg)
        self.get_logger().info("HSDPA reported category : %d", cat)
        return cat

    def get_hsupa_reported_category(self):
        """
        Gets HSUPA reported category.
        :rtype: integer
        :return: the HSUPA reported category, an integer from 1 to 6,
        -1 if the equipment cannot report the value,
        -2 if the functionality isn't supported
        """
        (err, cat, msg) = W.GetHsupaReportedCategory(self.get_root())
        self.__error_check(err, msg)
        self.get_logger().info("HSUPA reported category : %d", cat)
        return cat

    def set_hsupa_tti(self, tti):
        """
        Sets HSUPA TTI.
        :type tti: integer
        :param tti: the TTI to set. Possible values:
            - 2  => 2ms
            - 10 => 10ms
        """
        (err, msg) = W.SetHsupaTti(self.get_root(), tti)
        self.__error_check(err, msg)

    def set_ps_data_configuration_type(self, cfg_type):
        """
        Sets PS data configuration type.
        :type cfg_type: str
        :param cfg_type: the PS data configuration type to set. Possible values:
            - "FIXED"
            - "REPORTED"
            - "USER_DEFINED"
        """
        (err, msg) = W.SetPsDataConfigurationType(self.get_root(), cfg_type)
        self.__error_check(err, msg)

    def set_cqi_value(self, cqi):
        """
        Sets CQI value.
        :type cqi: integer
        :param cqi: the CQI value to set, an integer from 5 to 30.
        """
        (err, msg) = W.SetCqiValue(self.get_root(), cqi)
        self.__error_check(err, msg)

    def set_gprs_radio_access_bearer(self, ul_rab, dl_rab):
        """
        Sets GPRS radio access bearer.
        :type ul_rab: str
        :param ul_rab: uplink RAB. Possible values:
            - "64k"
            - "128k"
            - "384k"
            - "HSUPA"
        :type dl_rab: str
        :param dl_rab: uplink RAB. Possible values:
            - "64k"
            - "384k"
            - "HSDPA"
        """
        (err, msg) = W.SetGprsRadioAccessBearer(self.get_root(), ul_rab, dl_rab)
        self.__error_check(err, msg)

    def get_data_connection_status(self):
        """
        Gets data connection status.
        :rtype: str
        :return: the data connection status. Possible returned values:
            - "OFF": Default returned value
            - "IDLE"
            - "ATTACHING"
            - "ATTACHED"
            - "ATTACHED_INCOMPLETE"
            - "DETACHING"
            - "IDLE"
            - "OFF" => Default returned value
            - "DETACHED"
            - "PDP_ACTIVATING"
            - "PDP_ACTIVE"
            - "PDP_DEACTIVATING"
        """
        (err, status, msg) = W.GetDataConnectionStatus(self.get_root())
        self.__error_check(err, msg)
        return status

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id=None):
        """
        Checks that the data connection is set at the required sate
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
            - "ATTACHED"
            - "PDP_ACTIVE"
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
        :attention: This parameter is only used in 4G (LTE)
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
            self.get_logger().info("Data connection is %s and has been reached in %d seconds" % (current_state, int(timeout) - int(timer)))
            return True

    def set_automatic_rrc_state(self, state):
        """
        Sets automatic RRC state transition control state.
        :type state: str
        :param state: automatic RRC state transition control state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetAutomaticRrcState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_automatic_dl_data_state(self, state):
        """
        Sets the automatic transitions on DL IP data state.
        :type state: str
        :param state: automatic transition on DL IP data state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetAutomaticDlDataState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_cell_dch_inactivity_timer(self, timer):
        """
        Sets the cell DCH inactivity timer length, and sets its state to ON.
        :type timer: double
        :param timer: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
        """
        (err, msg) = W.SetCellDchInactivityTimer(self.get_root(), timer)
        self.__error_check(err, msg)

    def set_cell_dch_transition_state(self, state):
        """
        Sets the cell DCH inactivity timer destination RRC state.
        :type state: str
        :param state: CELL_DCH inactivity timer destination RRC state:
            - "IDLE"
            - "FACH"
        """
        (err, msg) = W.SetCellDchTransitionState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_cell_fach_inactivity_timer(self, timer):
        """
        Sets the cell FACH inactivity timer length, and sets its state to ON.
        :type timer: double
        :param timer: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
        """
        (err, msg) = W.SetCellFachInactivityTimer(self.get_root(), timer)
        self.__error_check(err, msg)

    def set_cell_fach_transition_state(self, state):
        """
        Sets the cell FACH inactivity timer destination RRC state.
        :type state: str
        :param state: CELL_FACH inactivity timer destination RRC state:
            - "DCH"
            - "PCH"
            - "UPCH"
            - "IDLE"
        """
        (err, msg) = W.SetCellFachTransitionState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_cpc_state(self, state):
        """
        Sets the CPC state.
        :type state: str
        :param state: CPC state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetCpcState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_ccpch_channelization_code(self, code):
        """
        Set the secondary common control physical channel's channelization code
        :type code: integer
        :param code: channelization code to set (1 to 63)
        """
        (err, msg) = W.SetCcpchChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_pich_channelization_code(self, code):
        """
        Set the page indicator channel's channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        (err, msg) = W.SetPichChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_aich_channelisation_code(self, code):
        """
        Set the acquisition indicator channel's channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        (err, msg) = W.SetAichChannelisationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_dpch_15ksps_channelization_code(self, code):
        """
        Set the channelization code for the downlink DPCH with a physical
        data rate of 15 ksps
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        (err, msg) = W.SetDpch15kspsChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_dpch_30ksps_channelization_code(self, code):
        """
        Set the channelization code for the downlink DPCH with a physical
        data rate of 30 ksps
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        (err, msg) = W.SetDpch30kspsChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_dpch_hsdpa_15ksps_channelization_code(self, code):
        """
        Set the HSDPA/HSPA DPCH 15 ksps (OVSF 256) channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        (err, msg) = \
            W.SetDpchHsdpa15kspsChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_dpch_hsdpa_30ksps_channelization_code(self, code):
        """
        Set the HSDPA/HSPA DPCH 30 ksps (OVSF 128) channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        (err, msg) = \
            W.SetDpchHsdpa30kspsChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_eagch_channelization_code(self, code):
        """
        Set the E-AGCH channelization code
        :type code: integer
        :param state: channelization code to set (2 to 255)
        """
        (err, msg) = W.SetEagchChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_ehich_channelization_code(self, code):
        """
        Set the E-HICH/E-RGCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        (err, msg) = W.SetEhichChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_hsdpa_ps_data_first_hs_pdsch_channelization_code(self, code):
        """
        Set PS data first HS-PDSCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 11)
        """
        (err, msg) = W.SetHsdpaPsDataFirstHsPdschChannelizationCode(
            self.get_root(),
            code)
        self.__error_check(err, msg)

    def set_hsdpa_rb_test_first_hs_pdsch_channelization_code(self, code):
        """
        Set RB test mode first HS-PDSCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 11)
        """
        (err, msg) = W.SetHsdpaRbTestFirstHsPdschChannelizationCode(
            self.get_root(),
            code)
        self.__error_check(err, msg)

    def set_hs_scch2_channelization_code(self, code):
        """
        Set the HS-SCCH 2 channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        (err, msg) = W.SetHsScch2ChannelizationCode(self.get_root(), code)
        self.__error_check(err, msg)

    def set_hs_scch3_config_state(self, state):
        """
        Set the FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration state
        :type state: str
        :param state: FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetHsScch3ConfigState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_hs_scch4_config_state(self, state):
        """
        Set the FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration state
        :type state: str
        :param state: FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetHsScch4ConfigState(self.get_root(), state)
        self.__error_check(err, msg)

    def set_hsdpa_ocns_config_state(self, states):
        """
        Set the HSDPA/HSPA OCNS 1 to 6 channel configuration states
        :type states: array of str
        :param states: each HSDPA/HSPA OCNS 1 to 6 channel configuration
        state is "ON" or "OFF" word
        """
        (err, msg) = W.SetHsdpaOcnsConfigState(self.get_root(), states)
        self.__error_check(err, msg)

    def set_hsdpa_ocns_channelization_codes(self, codes):
        """
        Set the channelization codes of HSDPA orthogonal channel noise
        simulator (OCNS) 1 to 6
        :type codes: array of integers
        :param codes: each HSDPA orthogonal channel noise simulator
        channelization code is an integer from 1 to 127
        """
        (err, msg) = W.SetHsdpaOcnsChannelizationCodes(self.get_root(), codes)
        self.__error_check(err, msg)

    def set_hsdpa_cell1_connected_pccpch_sch_level(self, level):
        """
        Set the HSDPA cell 1 connected P-CCPCH/SCH level and set the HSDPA cell 1
        connected P-CCPCH/SCH state to on
        :type level: double
        :param level: HSDPA cell 1 connected P-CCPCH/SCH level (-20.00 dB to 0.00 dB)
        """
        (err, msg) = \
            W.SetHsdpaCell1ConnectedPCcpchSchLevel(self.get_root(), level)
        self.__error_check(err, msg)

    def set_hsdpa_cell1_connected_pich_level(self, level):
        """
        Set the HSDPA cell 1 connected PICH level and set the HSDPA cell 1
        connected PICH state to on
        :type level: double
        :param level: HSDPA cell 1 connected PICH level (-20.00 dB to 0.00 dB)
        """
        (err, msg) = W.SetHsdpaCell1ConnectedPichLevel(self.get_root(), level)
        self.__error_check(err, msg)

    def check_data_connection_transferring(self, timeout, check_ota_throughput=False, blocking=True):
        """
        Check data connection, looking for "transferring" state.

        :type timeout: integer
        :param timeout: allowed time in seconds to reach 'transferring' state

        :type check_ota_throughput: boolean
        :param check_ota_throughput: boolean defining if check is done on Over The Air
        throughputs (True) or IP throughput (False).

        :type blocking: boolean
        :param blocking: boolean defining if the function is blocking (returns
        an Exception) or not (returns True or False)

        :rtype: boolean
        :return: True if state is reached, else returns False

        :attention: check_ota_throughput set to True will
        test connection by checking OTA throughputs (Tx and Rx),
        whereas check_ota_throughput set to False will test connection
        by checking IP throughputs (Tx and Rx).
        """
        self.get_logger().info(
            "Check data connection state is transferring during %d seconds",
            timeout)
        # If an handover has been executed, we must compare OTA throughput
        # in order to check transferring mode.
        # Else, we check connection by comparing IP throughput.
        if check_ota_throughput:
            ul_correct = True
            dl_correct = True
            for _time in range(0, timeout):
                time.sleep(1)
                # retrieve downlink and uplink throughputs
                (err, rx_list, msg) = W.GetOverTheAirRxRate(self.get_root())
                self.__error_check(err, msg)
                current_rx = rx_list[1]
                (err, tx_list, msg) = W.GetOverTheAirTxRate(self.get_root())
                self.__error_check(err, msg)
                current_tx = tx_list[1]
                # checks that current throughput aren't equal to 0
                # or isn't an invalid value (9.91e+37).
                if current_rx == 0 or current_rx == float("9.91e+37"):
                    ul_correct = False
                if current_tx == 0 or current_tx == float("9.91e+37"):
                    dl_correct = False
            # if both ul_increase and dl_increase are False,
            # raise an exception: data connection isn't in transferring mode.
            if ul_correct == False and dl_correct == False:
                if blocking:
                    msg = "Failed to reach transferring data state!"
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

                return False
            else:
                self.get_logger().info("Transferring data state is reached")
                return True
        else:
            # clear data counters
            (err, msg) = W.ClearDataCounters(self.get_root())
            self.__error_check(err, msg)

            # retrieve downlink and uplink throughputs
            (err, data_list, msg) = W.GetDataCounters(self.get_root())
            self.__error_check(err, msg)
            ul_length = data_list[2]
            dl_length = data_list[0]
            ul_increase = True
            dl_increase = True
            # verify during timieout seconds that transfer is enabled,
            # that mean curr_ul and curr_dl constantly increase
            for _time in range(1, timeout):
                time.sleep(1)
                (err, data_list, msg) = W.GetDataCounters(self.get_root())
                self.__error_check(err, msg)
                curr_ul = data_list[2]
                curr_dl = data_list[0]
                if ul_length >= curr_ul:
                    ul_increase = False
                if dl_length >= curr_dl:
                    dl_increase = False
                ul_length = curr_ul
                dl_length = curr_dl
            # if both ul_increase and dl_increase are False,
            # raise an exception: data connection isn't in transferring mode.
            if ul_increase == False and dl_increase == False:
                if blocking:
                    msg = "Failed to reach transferring data state!"
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

                return False
            else:
                self.get_logger().info("Transferring data state is reached")
                return True

    def check_data_connection_transferring_time(self, transferring_timeout, transferring_time, check_ota_throughput=False, blocking=True):
        """
        Check if data has been transferred continuously during transferring_time before transferring_timeout.

        :type transferring_timeout: integer
        :param transferring_timeout: allowed time in seconds to reach continuous data transfer

        :type transferring_time: integer
        :param transferring_time: time in seconds of continuous data transfer

        :type check_ota_throughput: boolean
        :param check_ota_throughput: boolean defining if check is done on Over The Air
        throughputs (True) or IP throughput (False).

        :type blocking: boolean
        :param blocking: boolean defining if the function is blocking (returns
        an Exception) or not (returns True or False)

        :rtype: boolean
        :return: True if state is reached, else returns False

        :attention: check_ota_throughput set to True will
        test connection by checking OTA throughputs (Tx and Rx),
        whereas check_ota_throughput set to False will test connection
        by checking IP throughputs (Tx and Rx).
        """
        self.get_logger().info(\
            "Check if there is a continuous data transfer during %d seconds before %d seconds" \
            % (transferring_time, transferring_timeout))
        # If an hand-over has been executed, we must compare OTA throughput
        # in order to check continuous data transfer.
        # Else, we check connection by comparing IP throughput.
        ul_count = 0
        dl_count = 0
        if(check_ota_throughput):
            for _time in range(0, transferring_timeout):
                time.sleep(1)
                # retrieve downlink and uplink throughputs
                (err, rx_list, msg) = W.GetOverTheAirRxRate(self.get_root())
                self.__error_check(err, msg)
                current_rx = rx_list[1]
                (err, tx_list, msg) = W.GetOverTheAirTxRate(self.get_root())
                self.__error_check(err, msg)
                current_tx = tx_list[1]
                dl_count += 1
                ul_count += 1
                # checks that current throughput aren't equal to 0
                # or isn't an invalid value (9.91e+37).
                if current_rx == 0 or current_rx == float("9.91e+37"):
                    dl_count = 0
                if current_tx == 0 or current_tx == float("9.91e+37"):
                    ul_count = 0
                if dl_count > transferring_time or ul_count > transferring_time:
                    self.get_logger().info("Continuous data transfer during %d seconds has been reached", transferring_time)
                    return True
            # Failed to transfer data continuously
            # raise an exception: data connection isn't in transferring mode.
            if blocking:
                msg = "Failed to transfer data continuously during %d seconds!" % transferring_time
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

            return False
        else:
            # clear data counters
            (err, msg) = W.ClearDataCounters(self.get_root())
            self.__error_check(err, msg)

            # retrieve downlink and uplink throughputs
            (err, data_list, msg) = W.GetDataCounters(self.get_root())
            self.__error_check(err, msg)
            ul_length = data_list[2]
            dl_length = data_list[0]

            # verify during timeout seconds that data transfer is continuous
            for _time in range(0, transferring_timeout):
                time.sleep(1)
                (err, data_list, msg) = W.GetDataCounters(self.get_root())
                self.__error_check(err, msg)
                curr_ul = data_list[2]
                curr_dl = data_list[0]
                dl_count += 1
                ul_count += 1
                if ul_length >= curr_ul:
                    ul_count = 0
                if dl_length >= curr_dl:
                    dl_count = 0
                ul_length = curr_ul
                dl_length = curr_dl
                if dl_count > transferring_time or ul_count > transferring_time:
                    self.get_logger().info("Continuous data transfer during %d seconds has been reached", transferring_time)
                    return True
            # Failed to transfer data continuously
            # raise an exception: data connection isn't in transferring mode.
            if blocking:
                "Failed to transfer data continuously during %d seconds!" % transferring_time
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

            return False

    def set_rrc_transition(self, transition):
        """
        Permits to perform UE transitions on pdp active state
        :type transition: str
        :param transition: the transition to set. Possible values:
            - "DCH"
            - "FACH"
            - "IDLE"
            - "PCH"
            - "UPCH"
        """
        (err, msg) = W.SetRrcTransition(self.get_root(), transition)
        self.__error_check(err, msg)

    def get_gprs_radio_access_bearer(self):
        """
        Gets GPRS Radio Access Bearer value.
        :rtype: str
        :return: the GPRS Radio Access Bearer. Possible returned values:
            - "GPRSRAB1" Default returned value
            - "GPRSRAB2"
            - "GPRSRAB3"
            - "GPRSRAB4"
            - "GPRSRAB5"
            - "PSDH64"
            - "PSDH128"
            - "PSDH384"
            - "PHSP"
        """
        (err, status, msg) = W.GetGprsRadioAccessBearer(self.get_root())
        self.__error_check(err, msg)
        return status

    def get_rbtest_channel_type(self):
        """
        Gets RBTEST channel type value.
        :rtype: str
        :return: the RBTEST channel type. Possible returned values:
            - HSDP12 (12.2k RMC + HSDPA)
            - HSP12 (12.2k RMC + HSPA)
            - HSPA
            - RMC12 (12.2k RMC)
            - RMC64 (64k RMC)
            - RMC144 (144k RMC)
            - RMC384 (384k RMC)
            - RMC33NC (33k No Coding RMC)
            - RMCAM1264 (12.2k UL/64k DL AM RMC)
            - RMCAM12144 (12.2k UL/144k DL AM RMC)
            - RMCAM12384 (12.2k UL/384k DL AM RMC)
            - RMCAM64384 (64k UL/384k DL AM RMC)
        """
        (err, status, msg) = W.GetRbtestChannelType(self.get_root())
        self.__error_check(err, msg)
        return status

    def get_network_type(self):
        """
        Returns the expected network type

        :rtype: str
        :return: the expected network type
        """
        # Read the GPRS RAB and RBTEST Channel type values
        rab = self.get_gprs_radio_access_bearer()
        self.get_logger().info("GPRS Radio Access Bearer is %s", rab)

        # Read the Current UE HS-DSCH and E-DCH categories
        hsdpa_cat = self.get_hsdpa_reported_category()
        hsupa_cat = self.get_hsupa_reported_category()

        network_type = ""
        # For network types, Check if GPRS Radio Access Bearer is set for WCDMA, HSDPA, HSUPA or HSPA
        # Five different RAB for WCDMA
        # Then the RBTEST Channel type gives the network type between HSUPA, HSPA and HSPA Plus
        if (rab == "GPRSRAB1") or (rab == "GPRSRAB2") or (rab == "GPRSRAB3") or \
                (rab == "GPRSRAB4") or (rab == "GPRSRAB5"):
            network_type = "WCDMA"

        # Check if HSDPA category is reported
        elif (hsdpa_cat >= 1) and (hsdpa_cat <= 12) and (hsupa_cat <= 0):
            network_type = "HSDPA"

        # Check if HSPA category is reported
        elif (hsdpa_cat >= 1) and (hsdpa_cat <= 12) and \
                (hsupa_cat >= 1) and (hsupa_cat <= 6):
            network_type = "HSPA"

        # Check if HSPAP (HSPA+) category is reported
        elif (hsdpa_cat >= 13) and (hsdpa_cat <= 24):
            network_type = "HSPAP"

        # Check if HSUPA category is reported
        elif (hsdpa_cat <= 0) and (1 <= hsupa_cat <= 6):
            network_type = "HSUPA"

        else:  # network RAT is wrong
            msg = " Network RAT (%s) does not match any known equipment RAT !" \
                % network_type
            self.get_logger().warning(msg)

        return network_type

    def set_edch_cell_capability(self, state):
        """
        Sets the E-DCH Cell capability to On / Off

        :type state: str
        :param state: E-DCH Cell capability state :
            - "ON"
            - "OFF"
        """
        if state == "ON":
            self.get_logger().info("Set E-DCH Cell capability On")
            self.__root.send_command("CALL:EDCH:CIND CAP")
        elif state == "OFF":
            self.get_logger().info("Set E-DCH Cell capability Off")
            self.__root.send_command("CALL:EDCH:CIND CNIN")
        else:
            # Failed to reach desired state
            msg = "Failed to reach %s E-DCH Cell capability state !" % state
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

    def set_hsdpa_cell_capability(self, state):
        """
        Sets the HSDPA Cell capability to On / Off

        :type state: str
        :param state: HSDPA Cell capability state :
            - "ON"
            - "OFF"
        """
        if state == "ON":
            self.get_logger().info("Set HSDPA Cell capability On")
            self.__root.send_command("CALL:HSDP:CIND CAP")
        elif state == "OFF":
            self.get_logger().info("Set HSDPA Cell capability Off")
            self.__root.send_command("CALL:HSDP:CIND CNIN")
        else:
            # Failed to reach desired state
            msg = "Failed to reach %s HSDPA Cell capability state !" % state
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

    def set_rrc_idle_with_scri(self, state):
        """
        Sets the RRC Idle With SCRI State.
        :type state: str
        :param state: the SCRI state, should be "on" or "off"
        :raise TestEquipmentException: If the SCRI_state parameter is not in his range.
        """
        if not state.lower() in ("on", "off"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The SCRI state should be on or off.")

        self.get_logger().info("Set RRC IDLE with SCRI to "
                               "%s" % (state.lower()))
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:SCRI:"
                                 "RRCIdle %s" % (state.lower()))

    def set_sccpch_prach(self, state):
        """
        Sets SCCPCH/PRACH Configuration
        :type state: str
        :param state: Value that will be set for the SCCPCH_PRACH
            Configuration. Range : signaling|straffic
        :raise TestEquipmentException: If the state parameter is not in his range.
        """

        if not state.lower() in ("signaling", "straffic"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The SCCPCH_PRACH configuration"
                                "can only be set to signaling or"
                                " straffic")

        if state.lower() == "signaling":
            gpib_parameter = "SIGN"
        if state.lower() == "straffic":
            gpib_parameter = "STR"

        self.get_logger().info("Set SCCPCH PRACH to %s" % (state.lower()))
        self.__root.send_command("CALL:CCPChannel:SECondary:CONFigure %s"
                                 % gpib_parameter)

    def set_auto_rrc_state_transition_control(self, state):
        """
        Sets Automatic RRC State Transition Control State.
        This setting controls whether the automatic RRC state transitions are
        enabled or not (including the transition based on downlink IP data).
        This setting cannot be changed while a PDP Context is Active.
        :type state: str
        :param state: State of the automatic RRC state transition control.
        Range: on|off
        :raise TestEquipmentException: If the state parameter is not in his range.
        """
        if not state.lower() in ("on", "off"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The automatic RRC state transition control"
                                " state can only be set to on or off")

        if state.lower() == "on":
            gpib_parameter = "1"
        if state.lower() == "off":
            gpib_parameter = "0"

        self.get_logger().info("Set automatic RRC state transition "
                               "to %s" % (state.lower()))
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:AUTO %s"
                                 % gpib_parameter)

    def set_fast_dormancy_state(self, state):
        """
        Sets Fast Dormancy State.
        This setting controls whether the fast dormancy is enabled or not.
        This setting cannot be changed while a PDP Context is Active.
        :type state: str
        :param state: State of the fast dormancy.
        Range: on|off
        :raise TestEquipmentException: If the state parameter is not in his range.
        """
        if not state.lower() in ("on", "off"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The fast dormancy state can only be set to on or off")

        if state.lower() == "on":
            gpib_parameter = "1"
        if state.lower() == "off":
            gpib_parameter = "0"

        self.get_logger().info("Set fast dormancy state "
                               "to %s" % (state.lower()))
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:SCRI:FDORmancy %s"
                                 % gpib_parameter)

    def set_fast_dormancy_destination(self, state):
        """
        Sets the Fast Dormancy Dest RRC State for CELL_DCH ..
        :type state: str
        :param state: Fast Dormancy Dest RRC State for CELL_DCH.
        Range: PCH|UPCH|FACH|IDLE
        :raise TestEquipmentException: If the state parameter is not in his range.
        """
        if not state.lower() in ("pch", "upch", "fach", "idle"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The Fast Dormancy Dest RRC State for CELL_DCH can only be set to PCH|UPCH|FACH|IDLE")

        self.get_logger().info("Set Fast Dormancy Dest RRC State for CELL_DCH "
                               "to %s" % state)
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:SCRI:FDORmancy:DCH:DESTination %s"
                                 % state)

    def set_fast_dormancy_support(self, state, rrc_inactivity_timer=None):
        """
        Enable/Disable the fast dormancy support.
        Enable = RCC idle with SCRI to on; SCCPCH_PRACH to signaling; automatic
        RRC state transition control to on
        Disable = RCC idle with SCRI to off; SCCPCH_PRACH to straffic; automatic
        RRC state transition control to off
        :type state: str
        :param state: State of the fast dormancy support. Range: enable|disable
        :type rrc_inactivity_timer: int
        :param rrc_inactivity_timer: Rrc transition inactivity timer in seconds (0-1800)
        if the value is None, Rrc transition inactivity timer will be default values
        :raise exception: If the state parameter is not in his range.
        """

        if not state.lower() in ("enable", "disable"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "The fast dormancy support parameter can "
                                "only be enable or disable")

        if state.lower() == "enable":
            self.get_logger().info("Enabling Fast Dormancy support.")
            self.set_rrc_idle_with_scri("on")
            self.set_sccpch_prach("straffic")
            self.set_auto_rrc_state_transition_control("on")
            self.set_fast_dormancy_destination("IDLE")
            if rrc_inactivity_timer is not None:
                self.set_fast_dormancy_timer(rrc_inactivity_timer)

        if state.lower() == "disable":
            self.get_logger().info("Disabling Fast Dormancy support.")
            self.set_rrc_idle_with_scri("off")
            self.set_sccpch_prach("signaling")
            self.set_auto_rrc_state_transition_control("off")

    def set_dut_ip_version(self, ip_num, ip_protocol):
        """
        Set the type of the communication the test set will use when setting
        up a connection with the DUT
        :type ip_num: int
        :param ip_num: number of the IP address to set (1 or 2).

        :type ip_protocol: str
        :param ip_protocol: protocol to use when establishing the connection.
        Possible values: IP4, IP6, DUAL.
        """
        if ip_num == 1:
            # Setting the GPIB command for the first DUT IP.
            gpib_command = "CALL:MS:IP:VERS"
            log_message = "Setting the first DUT IP address version"
        elif ip_num == 2:
            # Setting the GPIB command for the second DUT IP.
            gpib_command = "CALL:MS:IP:VERS2"
            log_message = "Setting the second DUT IP address version"
        else:
            msg = "The IP number should be either 1 or 2."
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Checking the selected protocol is within the range.
        if ip_protocol.upper() in ("IP4", "IP6", "DUAL"):
            self.get_logger().info("%s %s" % (log_message, ip_protocol.upper()))
            self.__root.send_command("%s %s" % (gpib_command, ip_protocol.upper()))
        else:
            msg = "The IP version should be : IP4 | IP6 | DUAL"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def get_cpc_state(self):
        """
        Get the CPC state (O = ON / 1 = OFF).

        :rtype: str
        :return: CPC state (ON/OFF)
        """
        cpc_value = int(self.__root.query_command("CALL:CPC:STAT?"))

        if cpc_value == 0:
            cpc_state = "OFF"

        if cpc_value == 1:
            cpc_state = "ON"

        return cpc_state

    def get_current_drx_activated_state(self):
        """
        Get the Current DRX Activated State

        :rtype: str
        :return: Current DRX Activated State (ON/OFF)
        """
        current_drx_activated_state = int(self.__root.query_command("CALL:STATus:CPC:DRX:ACTivated:STATe?"))

        if current_drx_activated_state == 0:
            current_drx_activated_state = "OFF"

        if current_drx_activated_state == 1:
            current_drx_activated_state = "ON"

        self.get_logger().info("Current DRX Activated state reported is : %s", current_drx_activated_state)

        return current_drx_activated_state

    def get_current_dtx_activated_state(self):
        """
        Get the Current DTX Activated State

        :rtype: str
        :return: Current DTX Activated State (ON/OFF)
        """
        current_dtx_activated_state = int(self.__root.query_command("CALL:STATus:CPC:DTX:ACTivated:STATe?"))

        if current_dtx_activated_state == 0:
            current_dtx_activated_state = "OFF"

        if current_dtx_activated_state == 1:
            current_dtx_activated_state = "ON"

        self.get_logger().info("Current DTX Activated state reported is : %s", current_dtx_activated_state)

        return current_dtx_activated_state

    def send_hsscch_order(self):
        """
        Sends the HS-SCCH order to deactivate CPC feature
        """
        # Sets the DRX Order Info (deactivation on DL DRX activated state)
        self.__root.send_command("CALL:CPC:DRX:ORDer 0")
        # Sets the DTX Order Info (deactivation on UL DTX activated state)
        self.__root.send_command("CALL:CPC:DTX:ORDer 0")
        # Perform the Send HS-SCCH Order action.
        self.__root.send_command("CALL:CPC:HSSCchannel:ORDer:SEND")

    def set_fast_dormancy_timer(self, rrc_inactivity_timer):
        """
        Set fast dormancy parameters
        :type rrc_inactivity_timer: int
        :param rrc_inactivity_timer: Rrc transition inactivity timer in seconds (0-1800)
        :raise exception: if rrc_inactivity_timer isn't int
        """

        if not isinstance(rrc_inactivity_timer, int):
            msg = "rrc_inactivity_timer should  be int %s" % str(rrc_inactivity_timer)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        elif rrc_inactivity_timer > 1800:
            msg = "rrc_inactivity_timer should  be less than 1800s"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Triggered from CELL_DCH to Idle where no RLC PDUs have been received
        # Set "CELL_DCH Inactivity Timer Dest RRC State" to IDle
        self.__root.send_command("CALL:SERV:PSD:RRC:TRAN:ITIMer:DCH:DEST IDLE")

        # Increase inactivity timer length to make sure fast dormancy is
        # requested by UE via SCRI timer
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:ITIMer:"
                                 "DCH:VALue %s" % str(rrc_inactivity_timer))

        # Triggered from CELL_FACH to Idle where no RLC PDUs have been received
        # Set "Fast Dormancy Dest RRC State for CELL_FACH" to idle
        self.__root.send_command("CALL:SERV:PSD:RRC:TRAN:SCRI:FDOR:FACH:DEST IDLE")
        # Increase inactivity timer length to make sure fast dormancy is
        # requested by UE via SCRI timer
        # Set "CELL_FACH Inactivity Timer Length" to the defined value in test case
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:ITIMer:"
                                 "FACH:VALue %s" % str(rrc_inactivity_timer))

        # Set "SCRI with Fast FDORmancy state" to on
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:SCRI:FDOR ON")

        # set Even 4B off to RRC so DCH directly goes to IDLE at the expiration of inactive timer
        # instead of going to FACH
        self.__root.send_command("CALL:SERVice:PSData:RRC:TRANsition:TVOL:TRIG:STAT:EVent4B OFF")

        # Set UE T323_timer to 30s to be longer than network initiated Fast Dormancy
        # this value isn't taken in account anymore in the release 9
        self.__root.send_command("CALL:BCCH:T323 %s" % str(rrc_inactivity_timer + 10))


    def get_rrc_states(self):
        """
        Gets rrc status.
        :rtype: str
        :return: the rrc status. Possible returned values:
            CELL_FACH, CELL_DCH or Idle
        """
        rrc_states = self.__root.query_command("CALL:STATus:RRC:STATe?")
        return rrc_states

    def set_cqi_test_set_up(self):
        """
        cqi test setup
        """
        cqi_test_set_up_commands = ["CALL:HSDP:SERV:PSD:HSDS:CONF RCQI",
                                    "SET:HBL:COUN 3000",
                                    "SET:HBL:CONT 0"]
        for command in cqi_test_set_up_commands:
            self.__root.send_command(command)

    def get_information_bit_throughput(self):
        """
        Gets reports the data throughput of the HSDPA connection
        :rtype: float
        :return: the information bit throughput in kbps
        """
        self.__root.send_command("INIT:HBL")
        time.sleep(5)
        measurement_fetch_flag = False
        timer = 20
        while timer >= 0:
            output = self.__root.query_command("INIT:DONE?")
            # Check measurements are ready to be fetched
            self.get_logger().info("INIT:DONE? ::%s " % output)
            # The measurement is ready
            if "HBL" in output:
                measurement_fetch_flag = True
                break
            elif "WAIT" in output:
                time.sleep(5)
                timer -= 5
            else:
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "INIT:DONE?"
                                     "command should return HBL or WAIT")

        if measurement_fetch_flag:
            throughput = round(float(self.__root.query_command("FETC:HBL:IBTH?")), 2)
        else:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Measure acquisite"
                                " not finished in 20 seconds, Measure fetching canceled.")

        return throughput

    def set_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed from:
        IPv6 Prefix, Upper/Lower (Hex) and IPv6 Default Router Interface ID, Upper/Lower (Hex)
        """
        # check server address fields because sometimes we might encounter empty address fields i.e: 2844::2345:2345:::4445:3234
        try:
            split_address = server_ipv6_address.split(':')
        except:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Server address fields should not be empty")

        for address in split_address:
            if address == "":
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Server address fields should not be empty")

        ipv6_prefix = split_address[0] + ':' + split_address[1] \
            + ':' + split_address[2] + ':' + split_address[3]
        ipv6_interface_id = split_address[4] + ':' + split_address[5] \
            + ':' + split_address[6] + ':' + split_address[7]

        self.get_logger().info("Set the network parameters for IPv6")
        self.get_logger().info("Set IPv6 Prefix in to %s" % ipv6_prefix)
        self.__root.send_command("SYSTem:COMMunicate:LAN:ADDRess:IP6:PREFix '%s'" % str(ipv6_prefix))
        self.get_logger().info("Set IPv6 Default Router Interface ID to %s" % ipv6_interface_id)
        self.__root.send_command("SYSTem:COMMunicate:LAN:DGATeway:IP6:IID '%s'" % str(ipv6_interface_id))

    def set_dut_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the DUT IP network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed from:
        IPv6 Prefix, Upper/Lower (Hex) and IPv6 Default Router Interface ID, Upper/Lower (Hex)
        """
        # check server address fields because sometimes we might encounter empty address fields
        # i.e: 2844::2345:2345:::4445:3234
        try:
            split_address = server_ipv6_address.split(':')
        except:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Server address fields should not be empty")

        for address in split_address:
            if address == "":
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Server address fields should not be empty")

        ipv6_prefix = split_address[0] + ':' + split_address[1] \
            + ':' + split_address[2] + ':' + split_address[3]

        ipv6_interface_id = split_address[4] + ':' + split_address[5] \
            + ':' + split_address[6] + ':' + split_address[7]

        self.get_logger().info("Set the DUT IPv6 network parameters for IPv6")
        self.get_logger().info("Set DUT IPv6 Prefix to %s" % ipv6_prefix)
        self.__root.send_command("CALL:MS:IP:ADDRess:IP6:PREFix '%s'" % str(ipv6_prefix))
        self.get_logger().info("Set DUT IPv6 DNS to %s" %
                                       (str(ipv6_prefix) + ':' + str(ipv6_interface_id)))
        self.__root.send_command("CALL:MS:DNSServer:PRIMary:IP:ADDRess:IP6 '%s'" %
                                       (str(ipv6_prefix) + ':' + str(ipv6_interface_id)))

    def configure_ims_reject(self, apn_name):
        """
        This command configure callbox to reject IMS PDP context.
        :type apn_name: str
        :param apn_name: name of IMS apn
        """
        self.get_logger().info("Configure callbox to reject IMS PDP context")
        # Set first APN state to 1
        self.__root.send_command("CALL:SERVice:PSData:RAPName:STATe ON, OFF, OFF, OFF, OFF, OFF")
        # Set PDP reject cause to 33 (Requested Service Opt Not Suscribed)
        self.__root.send_command("CALL:SERVice:PSData:RAPName:REJect:CAUSe 33, 33, 33, 33, 33, 33")
        # Set First APN name
        self.__root.send_command("CALL:SERVice:PSData:RAPName1:VAlue CUST")
        self.__root.send_command("CALL:SERVice:PSData:RAPName1:CUSTom '%s'" % apn_name)
