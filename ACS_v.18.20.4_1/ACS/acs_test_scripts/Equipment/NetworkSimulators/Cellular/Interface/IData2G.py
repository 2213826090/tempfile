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
:summary: virtual interface of 2G data functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IData2G(object):

    """
    IData2G class: virtual interface of 2G data functionalities for cellular
    network simulators.
    """

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
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        :rtype: boolean
        :return: True if state is reached, else returns False
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_ip_address(self, ip_num, ip_addr):
        """
        Sets the DUT IP address.
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

    def get_dut_ip_address(self, ip_num):
        """
        Gets the DUT IP address.
        :type ip_num: integer
        :param ip_num: the number of the IP address to return (1 to 4).
        :rtype: str
        :return: the IP address of the DUT.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_dut_primary_dns(self):
        """
        Gets DUT primary DNS address.
        :rtype: str
        :return: the IP address of the primary DNS of the DUT.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_dut_secondary_dns(self):
        """
        Gets DUT secondary DNS address.
        :rtype: str
        :return: the IP address of the device under test secondary DNS
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_initial_ps_data_rrc_state(self, state):
        """
        Sets initial PS data RRC state.
        :type state: str
        :param state: the desired state. Possible values:
            - "DCH"
            - "FACH"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_pdtch_puncturing_scheme(self, scheme):
        """
        Sets the PDTCH puncturing scheme.
        :type scheme: str
        :param scheme: the PDTCH puncturing scheme to set. Possible values:
            - "PS1"
            - "PS2"
            - "PS3"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_pdat_reference_power_level(self, timeslot, power_lvl, gamma=0):
        """
        Sets the I{timeslot offset} so that the obtain a power level corresponding
        to the C{power_lvl} parameter value.
        The actual I{timeslot offset} is computed according to the following formula:
            - I{timeslot offset} = C{power_lvl} - I{reference power}
        The I{reference power} value is automatically retrieved by the equipment
        object.

        .. warning:: slots other than the one indicated by C{timeslot}
        parameter will be disabled.

        .. warning:: this method should not be used in association with
        C{set_custom_multislot_config}.

        :type gamma: int
        :param gamma: the gamma level for the given timeslot (defaults to 0)

        :type timeslot: int
        :param timeslot: index of the slot on which the requested power level
            will be set (0 to 7).

        :type power_lvl: float
        :param power_lvl: the wanted power level, from -137 dBm to -10 dBm

        :rtype: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_pdtch_downlink_coding_scheme(self, scheme):
        """
        Sets packet data traffic channel downlink coding scheme.
        :type scheme: str
        :param scheme: the PDTCH downlink coding scheme to set. Possible values:
            - "CS1"
            - "CS2"
            - "CS3"
            - "CS4"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_data_cell_off(self):
        """
        Sets the data cell off.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_data_cell_on(self):
        """
        Sets the data cell on.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_coding_scheme(self, coding_scheme):
        """
        Sets the data I{coding scheme} to use.
        :type coding_scheme: str
        :param coding_scheme: the I{coding scheme} to use. Possible values are:
            - {"CS1", "CS2", "CS3", "CS4", "MCS1", "MCS2", "MCS3", "MCS4", "MCS5"
               "MCS6", "MCS7", "MCS8", "MCS9")
        Possible values are available as attributes in class C{CodingScheme2G}.
        :rtype: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def data_call(self, call_mode):
        """
        Perform a data call.
        :type call_mode: str
        :param call_mode: the desired call mode. Possible values:
            - TEST_MODE_A
            - TEST_MODE_B
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def data_call_release(self):
        """
        Release a currently running data call.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_multislot_config(self, main_timeslot, dl_slot_enabled,
                                    dl_slot_level, ul_slot_enabled, ul_slot_gamma):
        """
        Sets a custom multislot configuration.
        :type main_timeslot: integer
        :param main_timeslot: The number of the main time slot (0 to 7).
        :type dl_slot_enabled: str
        :param dl_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','.
        :type dl_slot_level: str
        :param dl_slot_level: a str of 8 levels in dB separated by ','
        (each level is a double from -127.0 to +127.0).
        :type ul_slot_enabled: str
        :param ul_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','.
        :type ul_slot_gamma: str
        :param ul_slot_gamma: a str of 8 gamma power control separated by ','
        (each gamma is an integer from 0 to 31).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_data_call_connected(self, call_setup_timeout, expected_state):
        """
        Checks that the current data call is I{connected} until
        the call setup timeout has been reached.
        If the data call is not I{connected} raises an TestEquipmentException.
        :type call_setup_timeout: integer
        :param call_setup_timeout: the timeout before which we expect the call
        to be connected.
        :type expected_state: str
        :param expected_state: expected data call state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def data_register_dut(self, dut_imsi, timeout):
        """
        Registers the DUT on the data cell.
        If C{dut_imsi} is null no registration check will be performed.
        :type dut_imsi: str
        :param dut_imsi: imsi of the DUT (unused)
        :type timeout: integer
        :param timeout: maximum authorized time for DUT registration
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_data_channel(self, ul_channel):
        """
        Sets the uplink channel to the given value.
        :type ul_channel: integer
        :param ul_channel: the uplink channel value
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_pdat_sensitivity_measurement(
        self,
        start_power,
        ber_limit,
        maximum_block_count,
        steps,
        main_timeslot,
            gamma):
        """
        Returns the measured sensitivity as described by the margin research algorithm
        in packet data.

        :type start_power: integer
        :param start_power: the initial power value (in dBm)

        :type ber_limit: float
        :param ber_limit: the upper limit of error (percentage).
        This value corresponds to a bit error rate.

        :type maximum_block_count: integer
        :param maximum_block_count: the maximum block number on which the measurements
        will be performed.

        :type steps: tuple
        :param steps: a tuple containing all decrements (in descending order).
        Those values will be used in order to set the downLink power level
        when computing error rates.

        :type main_timeslot: integer
        :param main_timeslot: main timeslot that will used to set the power level.

        :type gamma: int
        :type gamma: the gamma level (defaults to 0)

        :rtype: float
        :return: the calculated sensitivity (in dBm).
        :raise TestEquipmentException: when an algorithm problem occurs.
        :raise TestEquipmentException: corresponds to a network simulator error.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_data_connection_transferring(self, timeout, check_ota_throughput=False, blocking=True):
        """
        Returns the measured sensitivity as described by the margin research algorithm
        in packet data.

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

        :param timeout: allowed time in seconds to reach the expected state
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_network_type(self):
        """
        Returns the expected network type

        :rtype: str
        :return: the expected network type
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_data_connection_transferring_time(self, transferring_timeout, transferring_time, check_ota_throughput = False, blocking = True):
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
