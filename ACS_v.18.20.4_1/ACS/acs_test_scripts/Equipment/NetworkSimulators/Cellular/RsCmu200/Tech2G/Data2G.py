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
:summary: data 2G implementation for RS CMU200 cellular network simulator
:since: 05/04/2011
:author: ymorel
"""

import time
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData2G import IData2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WData2G as W
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WCell2G as WC2G
from ErrorHandling.TestEquipmentException import TestEquipmentException


class IndependantValue(object):
    CONF = 0
    """
    The value used to set up a connection
    (in Signal Off, Signal On and Registered states)
    """

    PROC = 1
    """
    The value to use during the connection
    (signaling state TBF Established)
    """

    VI_NULL = 2
    """
    The C{VI_NULL} value.
    """

    RS_CMU_DEF = 0
    """
    The default value for some RS CMU parameters.
    """


class CodingScheme2G(object):

    GPRS_VALUES = ("CS1", "CS2", "CS3", "CS4")
    """
    I{Coding scheme} string values for I{GPRS}.
    """

    EGPRS_VALUES = (
        "MCS1", "MCS2", "MCS3", "MCS4", "MCS5", "MCS6", "MCS7", "MCS8", "MCS9")
    """
    I{Coding scheme} string values for I{EGPRS}.
    """


class GprsServiceState(object):

    """
    This class contains str values for GPRS states on RS CMU 200.
    """
    RSCMU_GPRS_STATE_IDLE = "IDLE"
    RSCMU_GPRS_STATE_ATT = "ATT"
    RSCMU_GPRS_STATE_RAUP = "RAUP"
    RSCMU_GPRS_STATE_AIPR = "AIPR"
    RSCMU_GPRS_STATE_DIPR = "DIPR"
    RSCMU_GPRS_STATE_OFF = "OFF"
    RSCMU_GPRS_STATE_CTBF = "CTBF"
    RSCMU_GPRS_STATE_TEST = "TEST"
    RSCMU_GPRS_STATE_TED = "TED"
    RSCMU_GPRS_STATE_CBUL = "CBUL"

    GPRS_SERVICE_STATE_VALUES = (
        "IDLE",
        "ATT",
        "RAUP",
        "AIPR",
        "DIPR",
        "OFF",
        "CTBF",
        "TEST",
        "TED",
        "CBUL")


class DataCallStates(object):

    """
    A class holding I{data call} states values as class attributes.
    """

    DATA_CALL_STATES = {
        "TEST_MODE_A": "CTMA",
        "TEST_MODE_B": "CTMB"}
    """
    The values to use when initiating a I{data call}.
    """

    DATA_CALL_CONNECTED_STATES = {
        "TEST_MODE_A": "TEST",
        "TEST_MODE_B": "TEST",
        "TEST": "TEST"}
    """
    The matching between I{data call} values and values
    returned when retrieving I{call control status}.
    """


class Data2G(IData2G):

    """
    Data 2G implementation for RS CMU200
    """

    MAX_LAC_VALUE = 65535

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (RsCMU)
        """
        IData2G.__init__(self)
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
            self.get_logger().warning(msg)

    def get_root(self):
        """
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def set_data_cell_off(self):
        """
        Sets the data cell off
        """
        self.get_logger().info("Set data cell OFF")
        (err, msg) = \
            W.ProcessSignalingPacketDataActivation(self.get_root(), "OFF")
        self.__error_check(err, msg)

    def set_data_cell_on(self):
        """
        Sets the data cell on
        """
        self.get_logger().info("Set data cell ON")
        (err, msg) = \
            W.ProcessSignalingPacketDataActivation(self.get_root(), "ON")
        self.__error_check(err, msg)

    def set_coding_scheme(self, coding_scheme):
        """
        Sets the data coding scheme to use.
        :type coding_scheme: str
        :param coding_scheme: the coding scheme to use. Possible values are:
            - {"CS1", "CS2", "CS3", "CS4", "MCS1", "MCS2", "MCS3", "MCS4",
               "MCS5", "MCS6", "MCS7", "MCS8", "MCS9"
        :rtype: None
        """
        if coding_scheme in CodingScheme2G.GPRS_VALUES:
            self.get_logger().info(
                "Configure GPRS data coding scheme to %s",
                coding_scheme)
            (err, msg) = W.SetNetworkPacketDataGprsCodingScheme(
                self.get_root(),
                coding_scheme)
        elif coding_scheme in CodingScheme2G.EGPRS_VALUES:
            self.get_logger().info(
                "Configure EGPRS data coding scheme to %s",
                coding_scheme)
            (err, msg) = W.SetNetworkPacketDataEgprsCodingScheme(
                self.get_root(),
                "NOT_CONNECTED",
                coding_scheme)
        self.__error_check(err, msg)

    def data_call(self, call_mode):
        """
        Perform a data call
        :type call_mode: str
        :param call_mode: the desired call mode. Possible values:
            - TEST_MODE_A
            - TEST_MODE_B
        """
        self.get_logger().info("Perform data call")
        if not call_mode in DataCallStates.DATA_CALL_STATES.keys():
            msg = "Invalid parameter value %s for call mode!" % (str(call_mode))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        call_mode = DataCallStates.DATA_CALL_STATES[call_mode]
        (err, msg) = W.ProcessSignalingPacketDataActivation(
            self.get_root(),
            call_mode)
        self.__error_check(err, msg)

    def data_call_release(self):
        """
        Releases a currently running data call
        """
        self.get_logger().info("Release data call")
        (err, msg) = W.ProcessSignalingPacketDataActivation(
            self.get_root(),
            "OFF")
        self.__error_check(err, msg)

    def _get_reference_level(self):
        """
        Returns the reference level
        :rtype: float
        :return: the reference level
        """
        # Retrieve the refence power level
        (err, level, msg) = W.GetPacketDataReferenceLevel(
            self.get_root())
        self.__error_check(err, msg)
        return level

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
        :param gamma: the gamma level for the given timeslot
            (defaults to 0; 0 equivalent to I{max power} on CMU)

        :type timeslot: int
        :param timeslot: index of the slot on which the requested power level
            will be set (0 to 7).

        :type power_lvl: float
        :param power_lvl: the wanted power level, from -137 dBm to -10 dBm

        :rtype: None
        """
        # Retrieve the refence power level
        (err, level, msg) = W.GetPacketDataReferenceLevel(self.get_root())
        self.__error_check(err, msg)
        # Compute the timeslot offset
        computed_level = power_lvl - level
        # Compute the parameters for slot configuration
        default_slots_enabled = \
            ["OFF", "OFF", "OFF", "OFF", "OFF", "OFF", "OFF", "OFF"]
        default_levels = ["0", "0", "0", "0", "0", "0", "0", "0"]
        dl_slot_enabled = []
        dl_slot_enabled.extend(default_slots_enabled)
        dl_slot_enabled[timeslot] = "ON"
        dl_slot_enabled = ",".join(dl_slot_enabled)
        dl_levels = []
        dl_levels.extend(default_levels)
        dl_levels[timeslot] = str(computed_level)
        dl_levels = ",".join(dl_levels)
        ul_slot_enable = dl_slot_enabled
        ul_levels = []
        ul_levels.extend(default_levels)
        ul_levels[timeslot] = str(gamma)
        ul_levels = ",".join(ul_levels)
        # Configure the computed offset for the timeslot
        # at the given index
        self.set_custom_multislot_config(
            timeslot,
            dl_slot_enabled,
            dl_levels,
            ul_slot_enable,
            ul_levels)

        # Retrieve the refence power level
        test_mode2G = self.get_root().get_cell_2g().get_test_mode()
        ber_reference_level = test_mode2G.get_pdat_ber_reference_level()
        # Compute the timeslot offset
        computed_level = power_lvl - ber_reference_level
        # Configure the computed offset for the main timeslot
        # at the given index
        slot_levels = [0, 0, 0, 0, 0, 0, 0, 0]
        slot_levels[timeslot] = computed_level
        test_mode2G.configure_pdat_ber_slots(slot_levels)

    def set_custom_multislot_config(self,
                                    main_timeslot,
                                    dl_slot_enabled,
                                    dl_slot_level,
                                    ul_slot_enabled,
                                    ul_slot_gamma):
        """
        Sets a custom multislot configuration
        :type main_timeslot: integer
        :param main_timeslot: the main timeslot (0 to 7)
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
        (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
        self.__error_check(err, msg)
        # Check whether we are in call mode
        connected = state in "TEST"
        # If we are connected
        if connected:
            # Use the 'connected' version of the slot configuration
            (err, msg) = W.SetCustomMultislotConfig(
                self.get_root(),
                main_timeslot,
                dl_slot_enabled,
                dl_slot_level,
                ul_slot_enabled,
                ul_slot_gamma)
        else:
            # Use the 'disconnected' version of the slot configuration
            (err, msg) = self._set_custom_multislot_disconnected(
                main_timeslot,
                dl_slot_enabled,
                dl_slot_level,
                ul_slot_enabled,
                ul_slot_gamma)
        self.__error_check(err, msg)

    def _set_custom_multislot_disconnected(self,
                                           main_timeslot,
                                           dl_slot_enabled,
                                           dl_slot_level,
                                           ul_slot_enabled,
                                           ul_slot_gamma):
        """
        This method  performs the custom multislot configuration
        while not in call mode.
        :type mainTimeslot: integer
        :param mainTimeslot: the main timeslot (0 to 7)
        :type dl_slot_enabled: str
        :param dl_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','
        :type dl_slot_level: str
        :param dl_slot_level: a str of 8 levels in dB separated by ','
        (each level is a double from -127.0 to +127.0)
        :type ul_slot_enabled: str
        :param ul_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','
        :type ul_slot_gamma: str
        :param ul_slot_gamma: a str of 8 gamma power control separated by ','
        (each gamma is an integer from 0 to 31)
        """
        # Configure the main timeslot
        (err, msg) = W.SetBSSignalPacketDataMainTimeslot(
            self.get_root(),
            main_timeslot)
        self.__error_check(err, msg)
        # We do not have to retrieve the reference level (absolute)
        # because all levels are relative to the reference level
        # (cf GPIB command reference: CONFigure:BSSignal:PDATa[:TCH]:MSLot:SCONfig)

        # Disable the automatic slot configuration
        (err, msg) = W.SetPacketDataAutomatedSlotConfiguration(
            self.get_root(),
            "OFF")
        self.__error_check(err, msg)
        # Configure slots in uplink
        (err, msg) = W.ConfigurePacketDataMultislotPowerControl(
            self.get_root(),
            ul_slot_enabled,
            ul_slot_gamma)
        self.__error_check(err, msg)
        # Compute DL list and enable slot list
        dl_enabled_list = dl_slot_enabled.split(",")
        db_level_list = dl_slot_level.split(",")
        # Configure slots in downlink
        (err, msg) = W.ConfigurePacketDataMultislotConfig(
            self.get_root(),
            dl_enabled_list,
            db_level_list)
        self.__error_check(err, msg)
        return err, msg

    def check_data_call_connected(self, call_setup_timeout, expected_state="TEST"):
        """
        Checks that the current data call is connected until
        the call setup timeout has been reached
        If the data call is not connected raises an TestEquipmentException
        :type call_setup_timeout: integer
        :param call_setup_timeout: the timeout before which we expect the call
        to be connected
        :type expected_state: str
        :param expected_state: expected data call state
        """
        # Initialize local variables
        elapsed_time = 0
        # Change the expected state
        self.get_logger().info(
            "Check data call is connected before %s seconds",
            call_setup_timeout)
        expected_state = \
            DataCallStates.DATA_CALL_CONNECTED_STATES[expected_state]
        # We check the current status
        (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
        # Loop
        while (elapsed_time < call_setup_timeout) and (state != expected_state):
            time.sleep(1)
            (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
            self.__error_check(err, msg)
            elapsed_time += 1
        if state == expected_state:
            self.get_logger().info("Registration success!")
        else:
            error_msg = "Call connection (mode %s) failure after %d seconds!" % \
                (expected_state, elapsed_time)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, error_msg)

    def data_register_dut(self, dut_imsi, timeout):
        """
        Registers the DUT on the data cell
        If cdk IMSI is null, no registration check will be performed
        :type dut_imsi: str
        :param dut_imsi: imsi of the DUT (unused)
        :type timeout: integer
        :param timeout: maximum authorized time for DUT registration
        """
        self.get_logger().info(
            "Attempting DUT data registration for %d seconds",
            timeout)
        # Set cell OFF
        self.set_data_cell_off()
        # Change LAC
        self.__root.get_cell_2g().set_random_lac()
        # Set cell ON
        self.set_data_cell_on()

        if dut_imsi == "":  # No registration check
            return

        # Get the data call status
        elapsed_time = 0
        (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
        self.__error_check(err, msg)

        # Loop
        while (elapsed_time < timeout) and (state != "ATT"):
            time.sleep(1)
            (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
            self.__error_check(err, msg)
            elapsed_time += 1

        if state == "ATT":
            self.get_logger().info("Registration success!")
        else:  # Registration failure
            msg = "Registration failure after %d seconds!" % elapsed_time
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

    def set_data_channel(self, ul_channel):
        """
        Sets the uplink channel to the given value.
        :type ul_channel: integer
        :param ul_channel: the uplink channel value. Possible values (ranges:
            - GSM400 : 259 ... 293 | 306 ... 340
            - GSM850 : 128 ... 251
            - GSM900 : 0 ... 124 | 955 ... 1023
            - GSM1800: 512 ... 885
            - GSM1900: 512 ... 810
        """
        # We check the current status
        (err, state, msg) = W.GetDataCallControlStatus(self.get_root())
        # We check the connection in the same way it is done
        # in CMU dll function SetTchArfcn2G (cel_parameters_2G.cpp)
        connected = state == "ATT" or state == "AIPR" or state == "TEST"
        # If we are not connected
        if not connected:
            # Change the channel to use once it is in test mode
            (err, msg) = WC2G.SetPdtchArfcn(self.get_root(), ul_channel)
        else:
            # Change the channel to use while it is in test mode
            (err, msg) = W.SetDataTrafficChannel(self.get_root(), ul_channel)
        # Check for any error
        self.__error_check(err, msg)

    def get_network_type(self):
        """
        Returns the expected network type
        :rtype: str
        :return: the expected network type
        """
        network_type = "UNKNOWN"
        return network_type

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

        (err, current_state, msg) = W.GetDataCallControlStatus(self.get_root())
        self.__error_check(err, msg)

        while (timer > 0) and (current_state != state):
            if state == "ATT" and current_state == "TEST":
                attached = True
                break
            time.sleep(1)
            (err, current_state, msg) = \
                W.GetDataCallControlStatus(self.get_root())
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
