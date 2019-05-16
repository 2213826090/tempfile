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
:summary: virtual interface of 3G data functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IData3G(object):

    """
    IData3G class: virtual interface of 3G data functionalities for cellular
    network simulators.
    """

    def set_dut_ip_address(self, ip_num, ip_addr):
        """
        Sets DUT IP address.
        :type ip_num: integer
        :param ip_num: number of the IP address to set (1 to 4).
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def clear_all_dut_ip_addresses(self):
        """
        Clears all configured DUT IP addresses and sets them to " "
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS IP address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_initial_ps_data_rrc_state(self, state):
        """
        Sets the initial PS data RRC state.
        :type state: str
        :param state: the desired state. Possible values:
            - "DCH"
            - "FACH"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_category_control_mode(self, mode):
        """
        Sets category control mode.
        :type mode: str
        :param mode: the desired mode. Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_macd_pdu_size(self, size):
        """
        Sets ...
        :type size: str
        :param size: the desired mode. Possible values:
            - "BITS336"
            - "BITS656"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_hsdpa_reported_category(self):
        """
        Gets HSDPA reported category
        :rtype: integer
        :return: the HSDPA reported category, an integer from 1 to 64,
        -1 if equipment cannot report the value.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_hsupa_reported_category(self):
        """
        Gets HSUPA reported category.
        :rtype: integer
        :return: the HSUPA reported category, an integer from 1 to 6,
        -1 if the equipment cannot report the value,
        -2 if the functionality isn't supported
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsupa_tti(self, tti):
        """
        Sets HSUPA TTI.
        :type tti: integer
        :param tti: the TTI to set. Possible values:
            - 2:   2ms
            - 10: 10ms
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ps_data_configuration_type(self, cfg_type):
        """
        Sets packet switch data configuration type.
        :type cfg_type: str
        :param cfg_type: the PS data configuration type to set. Possible values:
            - "FIXED"
            - "REPORTED"
            - "USER_DEFINED"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cqi_value(self, cqi):
        """
        Sets CQI value.
        :type cqi: integer
        :param cqi: the CQI value to set, an integer from 5 to 30.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        :rtype: boolean
        :return: True if state is reached, else returns False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_automatic_rrc_state(self, state):
        """
        Sets automatic RRC state transition control state.
        :type state: str
        :param state: automatic RRC state transition control state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_automatic_dl_data_state(self, state):
        """
        Sets the automatic transitions on DL IP data state.
        :type state: str
        :param state: automatic transition on DL IP data state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_dch_inactivity_timer(self, timer):
        """
        Sets the cell DCH inactivity timer length, and sets its state to ON.
        :type timer: double
        :param timer: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_dch_transition_state(self, state):
        """
        Sets the cell DCH inactivity timer destination RRC state.
        :type state: str
        :param state: CELL_DCH inactivity timer destination RRC state:
            - "IDLE"
            - "FACH"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_fach_inactivity_timer(self, timer):
        """
        Sets the cell FACH inactivity timer length, and sets its state to ON.
        :type timer: double
        :param timer: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cpc_state(self, state):
        """
        Sets the CPC state.
        :type state: str
        :param state: CPC state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ccpch_channelization_code(self, code):
        """
        Set the secondary common control physical channel's channelization code
        :type code: integer
        :param code: channelization code to set (1 to 63)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_pich_channelization_code(self, code):
        """
        Set the page indicator channel's channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_aich_channelisation_code(self, code):
        """
        Set the acquisition indicator channel's channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_15ksps_channelization_code(self, code):
        """
        Set the channelization code for the downlink DPCH with a physical
        data rate of 15 ksps
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_30ksps_channelization_code(self, code):
        """
        Set the channelization code for the downlink DPCH with a physical
        data rate of 30 ksps
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_hsdpa_15ksps_channelization_code(self, code):
        """
         Set the HSDPA/HSPA DPCH 15 ksps (OVSF 256) channelization code
        :type code: integer
        :param code: channelization code to set (2 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_hsdpa_30ksps_channelization_code(self, code):
        """
        Set the HSDPA/HSPA DPCH 30 ksps (OVSF 128) channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_eagch_channelization_code(self, code):
        """
        Set the E-AGCH channelization code
        :type code: integer
        :param state: channelization code to set (2 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ehich_channelization_code(self, code):
        """
        Set the E-HICH/E-RGCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_ps_data_first_hs_pdsch_channelization_code(self, code):
        """
        Set PS data first HS-PDSCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 11)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_rb_test_first_hs_pdsch_channelization_code(self, code):
        """
        Set RB test mode first HS-PDSCH channelization code
        :type code: integer
        :param code: channelization code to set (1 to 11)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hs_scch2_channelization_code(self, code):
        """
        Set the HS-SCCH 2 channelization code
        :type code: integer
        :param code: channelization code to set (1 to 127)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hs_scch3_config_state(self, state):
        """
        Set the FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration state
        :type state: str
        :param state: FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hs_scch4_config_state(self, state):
        """
        Set the FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration state
        :type state: str
        :param state: FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_ocns_config_state(self, states):
        """
        Set the HSDPA/HSPA OCNS 1 to 6 channel configuration states
        :type states: array of str
        :param states: each HSDPA/HSPA OCNS 1 to 6 channel configuration
        state is "ON" or "OFF" word
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_ocns_channelization_codes(self, codes):
        """
        Set the channelization codes of HSDPA orthogonal channel noise
        simulator (OCNS) 1 to 6
        :type codes: array of integers
        :param codes: each HSDPA orthogonal channel noise simulator
        channelization code is an integer from 1 to 127
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_cell1_connected_pccpch_sch_level(self, level):
        """
        Set the HSDPA cell 1 connected P-CCPCH/SCH level and set the HSDPA cell 1
        connected P-CCPCH/SCH state to on
        :type level: double
        :param level: HSDPA cell 1 connected P-CCPCH/SCH level (-20.00 dB to 0.00 dB)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_cell1_connected_pich_level(self, level):
        """
        Set the HSDPA cell 1 connected PICH level and set the HSDPA cell 1
        connected PICH state to on
        :type level: double
        :param level: HSDPA cell 1 connected PICH level (-20.00 dB to 0.00 dB)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_network_type(self):
        """
        Returns the expected network type

        :rtype: str
        :return: the expected network type
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_edch_cell_capability(self, state):
        """
        Sets the E-DCH Cell capability to On / Off

        :type state: str
        :param state: E-DCH Cell capability state :
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_hsdpa_cell_capability(self, state):
        """
        Sets the HSDPA Cell capability to On / Off

        :type state: str
        :param state: HSDPA Cell capability state :
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rrc_idle_with_scri(self, state):
        """
        Sets the RRC Idle With SCRI State.
        :type state: str
        :param state: the SCRI state, should be "on" or "off"
        :raise TestEquipmentException: If the SCRI_state parameter is not in his range.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sccpch_prach(self, state):
        """
        Sets SCCPCH/PRACH Configuration
        :type state: str
        :param state: Value that will be set for the SCCPCH_PRACH
            Configuration. Range : signaling|straffic
        :raise TestEquipmentException: If the state parameter is not in his range.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cpc_state(self):
        """
        Get the CPC state (O = ON / 1 = OFF).

        :rtype: str
        :return: CPC state (ON/OFF)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_current_drx_activated_state(self):
        """
        Get the Current DRX Activated State

        :rtype: str
        :return: Current DRX Activated State (ON/OFF)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_current_dtx_activated_state(self):
        """
        Get the Current DTX Activated State

        :rtype: str
        :return: Current DTX Activated State (ON/OFF)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_hsscch_order(self):
        """
        Sends the HS-SCCH order to deactivate CPC feature
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_fast_dormancy_timer(self, rrc_inactivity_timer):
        """
        Set fast dormancy parameters
        :type rrc_inactivity_timer: int
        :param rrc_inactivity_timer: Rrc transition inactivity timer in seconds (0-1800)
        :raise exception: if rrc_inactivity_timer isn't int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rrc_states(self):
        """
        Gets rrc status.
        :rtype: str
        :return: the rrc status. Possible returned values:
            CELL_FACH, CELL_DCH or Idle
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cqi_test_set_up(self):
        """
        cqi test setup
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_information_bit_throughput(self):
        """
        Gets reports the data throughput of the HSDPA connection
        :rtype: float
        :return: the information bit throughput in kbps
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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

        .. warning:: check_ota_throughput set to True will
        test connection by checking OTA throughputs (Tx and Rx),
        whereas check_ota_throughput set to False will test connection
        by checking IP throughputs (Tx and Rx).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed by:
        IPv6 Prefix, Upper/Lower (Hex) and IPv6 Default Router Interface ID, Upper/Lower (Hex)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the DUT IP network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed from:
        IPv6 Prefix, Upper/Lower (Hex) and IPv6 Default Router Interface ID, Upper/Lower (Hex)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_ims_reject(self, apn_name):
        """
        This command configure callbox to reject IMS PDP context.
        :type apn_name: str
        :param apn_name: name of IMS apn
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
