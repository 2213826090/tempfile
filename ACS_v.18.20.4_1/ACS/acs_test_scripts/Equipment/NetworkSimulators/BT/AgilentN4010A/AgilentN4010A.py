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
:summary: implementation of Agilent N4010A bluetooth network simulator
:since:23/03/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.NetworkSimulators.BT.Interface.IBTNetSim import IBTNetSim
from acs_test_scripts.Equipment.NetworkSimulators.BT.AgilentN4010A.Wrapper import WN4010ABase
from acs_test_scripts.Equipment.NetworkSimulators.BT.AgilentN4010A.Wrapper import WN4010ASetup
from acs_test_scripts.Equipment.NetworkSimulators.BT.AgilentN4010A.Wrapper import WN4010AMeasurement


class AgilentN4010A(IBTNetSim, DllLoader):

    """
    Implementation of Agilent N4010A bluetooth network simulator
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment catalog parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing bench parameters of the equipment
        """
        IBTNetSim.__init__(self)
        DllLoader.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None

    def __del__(self):
        """
        Destructor
        """
        self.release()

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
        if self.get_handle() is None:
            board_id = int(self.__bench_params.get_param_value("GPIBBoardId"))
            gpib_addr = int(self.__bench_params.get_param_value("GPIBAddress"))
            (err, handle, msg) = WN4010ABase.Connect(
                self,
                "GPIB",
                board_id,
                gpib_addr,
                "")
            self.__error_check(err, msg)
            # Update handle value
            self._set_handle(handle)

    def __connect_via_TCPIP(self):
        """
        Connect to equipment via TCP/IP
        """
        if self.get_handle() is None:
            ip_addr = str(self.__bench_params.get_param_value("TcpIpAddress"))
            (err, handle, msg) = WN4010ABase.Connect(
                self,
                "TCPIP",
                0,
                0,
                ip_addr)
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
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("Initialization")

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
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            connect = getattr(self, "_" + self.__class__.__name__ +
                              "__connect_via_" + transport)
            connect()
        else:
            msg = "%s transport is disabled" % (str(transport))
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        """
        Releases all equipment resources and close connection.
        """
        self.get_logger().info("Release")
        if self.get_handle() is not None:
            (err, msg) = WN4010ABase.Disconnect(self)
            self.unload_driver()
            self.__error_check(err, msg)
            # Update handle value
            self._set_handle(None)

    def get_eqt_id(self):
        """
        Gets equipment identification str
        :rtype: str
        :return: the identification str of the connected equipment.
        """
        (err, eqt_id, msg) = WN4010ABase.GetEqId(self)
        self.__error_check(err, msg)
        return eqt_id

    def perform_full_preset(self):
        """
        Resets all parameters to their default values
        """
        (err, msg) = WN4010ABase.PerformFullPreset(self)
        self.__error_check(err, msg)

    def set_operating_mode(self, mode):
        """
        Sets the operating mode
        :type mode: str
        :param mode: the connection type to use:
            - "LINK"
            - "RFA"
            - "RFG"
        """
        (err, msg) = WN4010ASetup.SetOperatingMode(self, mode)
        self.__error_check(err, msg)

    def set_link_type(self, link_type):
        """
        Sets the link type
        :type link_type: str
        :param link_type: the connection type to use:
            - "ACL"
            - "SCO"
            - "TEST"
        """
        (err, msg) = WN4010ASetup.SetLinkType(self, link_type)
        self.__error_check(err, msg)

    def set_test_mode_link_type(self, mode):
        """
        Sets the test mode link type
        :type mode: str
        :param mode: the test mode link to set:
            - "LOOP"
            - "TRAN"
        """
        (err, msg) = WN4010ASetup.SetTestModeLinkType(self, mode)
        self.__error_check(err, msg)

    def select_link_profile(self, profile):
        """
        Selects the link profile
        :type profile: str
        :param profile: the head set profile to set:
            - "NONE"
            - "HSP": head set profile
        """
        (err, msg) = WN4010ASetup.SelectLinkProfile(self, profile)
        self.__error_check(err, msg)

    def set_link_profile_state(self, state):
        """
        Sets the link profile state
        :type state: str
        :param profile: the head set profile state to set:
            - "ON"
            - "OFF"
        """
        (err, msg) = WN4010ASetup.SetLinkProfileState(self, state)
        self.__error_check(err, msg)

    def set_loss_compensation_state(self, state):
        """
        Sets loss compensation state
        :type state: str
        :param profile: the loss compensation state to set:
            - "ON"
            - "OFF"
        """
        (err, msg) = WN4010ASetup.SetLossCompensationState(self, state)
        self.__error_check(err, msg)

    def set_fixed_loss_compensation(self, loss_comp):
        """
        Sets fixed loss compensation
        :type loss_comp: double
        :param loss_comp: loss compensation is in dB. Range -50dB to +40dB.
        """
        (err, msg) = WN4010ASetup.SetFixedLossCompensation(self, loss_comp)
        self.__error_check(err, msg)

    def set_eut_power_class(self, power_class):
        """
        Sets equipment under test power class
        :type power_class: str
        :param power_class: the power class to set:
            - "PC1"
            - "PC2"
            - "PC3"
        """
        (err, msg) = WN4010ASetup.SetEUTPowerClass(self, power_class)
        self.__error_check(err, msg)

    def reset_eut_bd_address(self):
        """
        Resets equipment under test BD addresses
        """
        (err, msg) = WN4010ASetup.ResetEUTBdAddress(self)
        self.__error_check(err, msg)

    def set_link_inquiry_duration(self, duration):
        """
        Sets link inquiry duration
        :type duration: double
        :param duration: duration in seconds: range 1.28 to 61.44. Value is
        rounded to the nearest multiple of 1.28 (example: giving 2.3s sets
        2.56s).
        """
        (err, msg) = WN4010ASetup.SetLinkInquiryDuration(self, duration)
        self.__error_check(err, msg)

    def initiate_inquiry_procedure(self):
        """
        Initiate inquiry procedure
        """
        (err, msg) = WN4010ASetup.InitiateInquiryProcedure(self)
        self.__error_check(err, msg)

    def list_bd_address(self):
        """
        Lists BD addresses
        :rtype: str
        :return: the addresses of the bluetooth devices that have responded to
        the inquiry procedure.
        """
        (err, addresses, msg) = WN4010ASetup.ListBDAddress(self)
        self.__error_check(err, msg)
        return addresses

    def set_ste_tx_power_level(self, power_level):
        """
        Sets Tx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -90.0 to 0.0,
        resolution 0.1.
        """
        (err, msg) = WN4010ASetup.SetSTETxPowerLevel(self, power_level)
        self.__error_check(err, msg)

    def set_ste_rx_power_level(self, power_level):
        """
        Sets Rx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -70.0 to 25,
        resolution 5.
        """
        (err, msg) = WN4010ASetup.SetSTERxPowerLevel(self, power_level)
        self.__error_check(err, msg)

    def set_specific_tx_power_level(self, power_level):
        """
        Sets specific Tx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -95.0 to 0.0,
        resolution 0.1.
        """
        (err, msg) = WN4010ASetup.SetSpecificTxPowerLevel(self, power_level)
        self.__error_check(err, msg)

    def set_specific_rx_power_level(self, power_level):
        """
        Sets specific Rx power level
        :type power_level: double
        :param power_level: the power level in dBm. Range -70 to 25,
        resolution 5.
        """
        (err, msg) = WN4010ASetup.SetSpecificRxPowerLevel(self, power_level)
        self.__error_check(err, msg)

    def reset_link_configure_parameters(self):
        """
        Resets link configure parameters
        """
        (err, msg) = WN4010ASetup.ResetLinkConfigureParameters(self)
        self.__error_check(err, msg)

    def set_frequency_hopping_state(self, state):
        """
        Sets frequency hopping state
        :type state: str
        :param profile: the frequency hopping state to set:
            - "ON"
            - "OFF"
        """
        (err, msg) = WN4010ASetup.SetFrequencyHoppingState(self, state)
        self.__error_check(err, msg)

    def reset_test_sequence(self):
        """
        Resets test sequence
        """
        (err, msg) = WN4010ASetup.ResetTestSequence(self)
        self.__error_check(err, msg)

    def add_test(self, name):
        """
        Adds a test to the test sequence
        :type name: str
        :param name: the test name to set:
            - {"CFDR"; "ICFT"; "MCH"; "MIL"; "MSEN"; "OPOW"; "PCON"; "SSEN";
               "BFP"; "DPEN"; "EMIL"; "ESEN"; "FSM"; "GTIM"; "RPOW"}
        """
        (err, msg) = WN4010ASetup.AddTest(self, name)
        self.__error_check(err, msg)

    def select_test(self, name, occurrence):
        """
        Selects a test in the test sequence
        :type name: str
        :param name: the test name to set. Possible values:
            - {"CFDR"; "ICFT"; "MCH"; "MIL"; "MSEN"; "OPOW"; "PCON"; "SSEN";
               "BFP"; "DPEN"; "EMIL"; "ESEN"; "FSM"; "GTIM"; "RPOW"}
        :type occurrence: integer
        :param occurrence: the occurrence of the test name to select
        """
        (err, msg) = WN4010ASetup.SelectTest(self, name, occurrence)
        self.__error_check(err, msg)

    def set_test_sequence_loop_mode(self, mode):
        """
        Sets test sequence loop mode
        :type mode: str
        :param mode: the test name to set:
            - "SING"
            - "CONT"
            - "FIX"
        """
        (err, msg) = WN4010ASetup.SetTestSequenceLoopMode(self, mode)
        self.__error_check(err, msg)

    def set_test_sequence_loop_number(self, nb_loop):
        """
        Sets test sequence loop number
        :type nb_loop: integer
        :param nb_loop: the number of test loops (1 to 99).
        """
        (err, msg) = WN4010ASetup.SetTestSequenceLoopNumber(self, nb_loop)
        self.__error_check(err, msg)

    def run_test_sequence(self):
        """
        Runs test sequence
        """
        (err, msg) = WN4010ASetup.RunTestSequence(self)
        self.__error_check(err, msg)

    def auto_disconnect_test_set(self, state):
        """
        Auto disconnects from test set
        :type state: str
        :param profile: the auto disconnect state to set:
            - "ON"
            - "OFF"
        """
        (err, msg) = WN4010ASetup.AutoDisconnectTestSet(self, state)
        self.__error_check(err, msg)

    def disconnect_test_set(self):
        """
        Disconnects from test set
        """
        (err, msg) = WN4010ASetup.DisconnectTestSet(self)
        self.__error_check(err, msg)

    def set_impairment_state(self, state):
        """
        Sets impairment state
        :type state: str
        :param profile: the impairment state to set:
            - "ON"
            - "OFF"
        """
        (err, msg) = WN4010ASetup.SetImpairmentState(self, state)
        self.__error_check(err, msg)

    def set_bits_number(self, nb_bits):
        """
        Sets bits number
        :type nb_bits: unsigned long
        :param nb_bits: the number of returned payload bits to use for the test
        """
        (err, msg) = WN4010ASetup.SetBitsNumber(self, nb_bits)
        self.__error_check(err, msg)

    def get_output_power_average(self, channel="SUMMARY", meas_type="MAX",
                                 occurrence=1):
        """
        Gets the average output power
        :type channel: str
        :param channel: the channel to measure
            - "SUMMARY": measure all channels
            - "LOW"
            - "MEDIUM"
            - "HIGH"
        :type meas_type: str
        :param meas_type: the type of measurement to operate
            - "MIN"
            - "MAX"
            - "AVERAGE"
        :type occurrence: integer
        :param occurrence: the occurrence of the test in the sequence (1 to 10)
        :rtype: double
        :return: the result of the measurement.
        """
        (err, power, msg) = WN4010AMeasurement.GetOutputPowerAverage(
            self,
            channel,
            meas_type,
            occurrence)
        self.__error_check(err, msg)
        return power

    def check_connectivity(self, target, margin):
        """
        Checks that DUT is correctly connected to the bluetooth network simulator.
        Connectivity is well connected if retrieved power is inferior to
        minimum power
        :type target: integer
        :param target: value of the expected DUT power (in dBm).
        :type margin: integer
        :param margin: acceptable margin.
        :raise TestEquipmentException: an exception is raised if connectivity is
        too bad.
        :raise TestEquipmentException: an internal problem occurs.
        """
        positive_margin = margin
        if margin < 0:
            positive_margin = -margin
        retrieved = self.get_output_power_average("MEDIUM", "AVERAGE", 1)
        power_min = target - positive_margin
        power_max = target + positive_margin
        if (retrieved < power_min) or (retrieved > power_max):
            msg = "Connectivity is too bad."
            raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR, msg)
