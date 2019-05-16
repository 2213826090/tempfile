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
:summary: implementation of Agilent 66311D power supplies
:since: 31/03/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.PowerSupplies.Interface.IPowerSupply import IPowerSupply
from acs_test_scripts.Equipment.PowerSupplies.Common.Wrapper import WAgilent663xxD as W
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment


class Agilent66311D(IPowerSupply, DllLoader):

    """
    Implementation class of Agilent 66311D power supply
    """

    GPIB_SUCCESS = '+0,"No error"'

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        IPowerSupply.__init__(self)
        # Construct DllLoader object
        DllLoader.__init__(self, name, model, eqt_params)
        # Initialize attributes
        self._bench_params = bench_params
        self.__handle = None
        self._visa = None
        self._name = name
        self._model = model
        self._eqt_params = eqt_params

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def _error_check(self, err, msg):
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
        Connect to the equipment via GPIB interface
        """
        if self.get_handle() is None:
            board_id = int(self._bench_params.get_param_value("GPIBBoardId"))
            gpib_addr = int(self._bench_params.get_param_value("GPIBAddress"))
            (err, handle, msg) = W.Connect(self, board_id, gpib_addr)
            self._error_check(err, msg)
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

    def get_bench_params(self):
        """
        gets the bench parameters of the equipment
        """
        return self._bench_params

    def init(self):
        """
        Initializes the power supply.
        Final state: the power supply is ready to use
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        # Load the equipment driver
        self.load_driver()

        # Get transport mode and try to connect to equipment
        transport = str(self._bench_params.get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s" % str(transport)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            connect = getattr(self, "_" + self.__class__.__name__ +
                              "__connect_via_" + transport)
            connect()
        else:
            msg = "%s transport is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Sets basic parameters of power supply
        self.configure_basic_parameters()
        time.sleep(0.5)

        #check the power supply health and remove protection
        status =self.check_protection_state()
        self.get_logger().info(" The power supply status is :%s" % status)
        if (status != "good") and not ("unknown" in status):
            self.get_logger().warning("Try to remove protection and restart the power supply")
            # Configure power supply parameters
            self.configure_basic_parameters()
            self.reset_protection()

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")
        if self.get_handle() is not None:
            (err, msg) = W.Disconnect(self)
            self.unload_driver()
            self._error_check(err, msg)
            # Update handle value
            self._set_handle(None)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to their default values
        """
        (err, msg) = W.PerformFullPreset(self)
        self._error_check(err, msg)

    def set_max_current(self, max_cur, port):
        """
        This function sets the maximum current allowed of power supply
        :param max_cur: maximum current allowed
        :param port: port number on which the current level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SetMaxCurrent(self, max_cur, port)
        self._error_check(err, msg)

    def set_current_voltage(self, voltage, port):
        """
        This function sets the current voltage of power supply
        :param voltage: current voltage to set
        :param port: port number on which the voltage level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SetVoltageLevel(self, voltage, port)
        self._error_check(err, msg)

    def get_current_meas(self, port, meas_type):
        """
        This function measures the current
        :type port: integer
        :param port: the port on which measurement has to be done
        :type meas_type: str
        :param meas_type: the type of measurement to do. Possible values:
            - "DC"    : dc current
            - "ACDC"  : ac+dc rms current
            - "HIGH"  : high current
            - "LOW"   : low current
            - "MAX"   : maximum current
            - "MIN"   : minimum current
        :rtype: float
        :return: the result of the measurement
        """
        (err, current, msg) = W.GetCurrentMeasurement(self, port, meas_type)
        self._error_check(err, msg)
        return current

    def get_current_meas_average(self, port, meas_type, iteration):
        """
        This function measures the current
        :type port: integer
        :param port: the port on which measurement has to be done
        :type meas_type: str
        :param meas_type: the type of measurement to do. Possible values:
            - "DC"    : dc current
            - "ACDC"  : ac+dc rms current
            - "HIGH"  : high current
            - "LOW"   : low current
            - "MAX"   : maximum current
            - "MIN"   : minimum current
        :type iteration: int
        :param duration: number of iteration for the average computing

        :rtype: float
        :return: the result of the measurement
        """
        i = 0
        average = float(0)
        while i < iteration:
            (err, current, msg) = W.GetCurrentMeasurement(self, port, meas_type, log=False)
            self._error_check(err, msg)
            # check if the measure is not an aberration
            if current > 1000:
                iteration -= 1
                continue
            average += current
            i += 1
        if iteration > 0:
            return average / iteration
        else:
            return average

    def power_on(self):
        """
        This function powers on the power supply
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SwitchOn(self)
        self._error_check(err, msg)

    def power_off(self):
        """
        This function powers off the power supply
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SwitchOff(self)
        self._error_check(err, msg)

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        (err, eqt_id, msg) = W.GetEqID(self)
        self._error_check(err, msg)
        return eqt_id

    def power_cycle(self):
        """
        Toggles this power supply:
            - sets the power to I{OFF}
            - wait a few seconds
            - sets the power to I{ON}
        """
        self.power_off()
        time.sleep(5)
        self.power_on()

    def set_over_current_protection_state(self, state):
        """
        Sets overcurrent protection state
        :type state: str
        :param state: the state to set
        """
        (err, msg) = W.SetOverCurrentProtectionState(self, state)
        self._error_check(err, msg)

    def set_sense_protect(self, state):
        """
        Sets the open sense lead detection state
        :type state: str
        :param state: the state to set
        """
        (err, msg) = W.SetSenseProtect(self, state)
        self._error_check(err, msg)

    def set_curr_sense_detector(self, detector):
        """
        Sets the sense detector
        :type detector: str
        :param detector: the detector to set ("ACDC" | "DC")
        """
        (err, msg) = W.SetCurrSenseDetector(self, detector)
        self._error_check(err, msg)

    def set_voltage_protection_level(self, level):
        """
        Sets the overvoltage protection level
        :type level: float
        :param level: the level to set
        """
        (err, msg) = W.SetVoltageProtectionLevel(self, level)
        self._error_check(err, msg)

    def set_resistance(self, level):
        """
        Sets the output resistance of the power supply
        :type level: float
        :param level: the resistance to set
        """
        (err, msg) = W.SetResistance(self, level)
        self._error_check(err, msg)

    def set_output_state(self, state, output):
        """
        Sets the output state
        :type state: str
        :param state: the state to set ("ON" | "OFF")
        :type output: integer
        :param output: the output to set
        """
        (err, msg) = W.SetOutputState(self, state, output)
        self._error_check(err, msg)

    def set_compensation_mode(self, mode):
        """
        Sets the compensation mode
        :type mode: str
        :param mode: the mode to set
        """
        (err, msg) = W.SetCompensationMode(self, mode)
        self._error_check(err, msg)

    def configure_basic_parameters(self):
        """
        Configure basic parameters of the power supply and register power supply
        outputs in equipment catalog dictionary
        """
        self.perform_full_preset()

        ps_params = self._bench_params
        # Sort power supply parameters list
        ps_params_list = sorted(ps_params.get_parameters_name())

        # Set the compensation mode
        if ps_params.has_parameter("CompensationMode"):
            mode = str(ps_params.get_param_value("CompensationMode"))
            self.set_compensation_mode(mode)

        # Set sense protection using
        if ps_params.has_parameter("SenseProtect"):
            protect = str(ps_params.get_param_value("SenseProtect"))
            self.set_sense_protect(protect)

        # Set over current protection state using
        # OverProtectState parameter
        if ps_params.has_parameter("OverProtectState"):
            state = str(ps_params.get_param_value("OverProtectState"))
            self.set_over_current_protection_state(state)

        # Set current source using CurrentSource
        if ps_params.has_parameter("CurrentSource"):
            detector = str(ps_params.get_param_value("CurrentSource"))
            self.set_curr_sense_detector(detector)

        # Set Voltage protection level using VoltageProtectLevel parameter
        if ps_params.has_parameter("VoltageProtectLevel"):
            level = float(ps_params.get_param_value("VoltageProtectLevel"))
            self.set_voltage_protection_level(level)

        # Configure each output of the power supply
        for attr_name in ps_params_list:

            if "OUTPUT" in attr_name:

                output_params = ps_params.get_parameters(attr_name)
                output_type = output_params.get_param_value("Type")

                # Set output properties if type is not NONE
                if output_type == "NONE":
                    continue

                port = int(output_params.get_param_value("PortNumber"))
                max_cur = float(output_params.get_param_value("MaxCurrent"))
                voltage = float(output_params.get_param_value("Voltage"))

                # Disable the output
                self.set_output_state("OFF", port)
                # Set the max current using CurrrentLevel and PortNumber
                self.set_max_current(max_cur, port)
                # Set voltage level using VoltageLevel and PortNumber
                self.set_current_voltage(voltage, port)
                # Enable the output
                self.set_output_state("ON", port)

    def _get_visa_equipment(self):
        """
        Get agilent equipment with pyvisa interface
        """

        # release DllLoader object
        self.release()

        self.get_logger().debug("Init agilent visa equipment")
        # Init agilent equipment with pyvisa interface
        self._visa = VisaEquipment(self._name,
                                    self._model,
                                    self._eqt_params,
                                    self._bench_params)

        self._visa.connect()

    def query_command(self, command):
        """
        query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """
        # Get agilent equipment with pyvisa interface
        self._get_visa_equipment()

        # Query the command
        self.get_logger().info("Query GPIB command : %s" % command)
        response = self._visa.query(command)

        # Check errors in the System Error buffer
        return_msg = self._visa.query("SYST:ERR?")

        if return_msg != Agilent66311D.GPIB_SUCCESS:
            self.get_logger().error(return_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, return_msg)

        # Release agilent equipment with pyvisa interface
        self._visa.disconnect()
        self._visa = None

        # reconnect the power supply with dll
        self.connect_power_supply()

        return response

    def set_protection_delay(self, delay):
        """
        write a GPIB Command in order to set the protection delay.

        :type delay: float
        :param delay: the delay to set

        :return: none
        """

        # Get agilent equipment with pyvisa interface
        self._visa = VisaEquipment(self._name,
                                    self._model,
                                    self._eqt_params,
                                    self._bench_params)

        self._visa.connect()

        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command : *CLS")
        self._visa.write("*CLS")

        # write the command
        self.get_logger().info("set the delay protection to %s" % str(delay))
        self._visa.write("OUTP:PROT:DEL %s" % str(delay))
        time.sleep(0.2)

        # Release agilent equipment with pyvisa interface
        self._visa.disconnect()
        self._visa = None

    def reset_protection(self):
        """
        write a GPIB Command in order to reset the protection state.

        :rtype: str
        :return: The response from agilent for protection state
        """

        # Get agilent equipment with pyvisa interface
        self._visa = VisaEquipment(self._name,
                                    self._model,
                                    self._eqt_params,
                                    self._bench_params)

        self._visa.connect()

        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command : *CLS")
        self._visa.write("*CLS")

        # write the command
        self.get_logger().info("try to reset the output 1 & 2 protection")
        self._visa.write("OUTP1 OFF")
        time.sleep(0.2)
        self._visa.write("OUTP:PROT:CLE")
        time.sleep(0.2)
        self._visa.write("OUTP1 ON")

        # Release agilent equipment with pyvisa interface
        self._visa.disconnect()
        self._visa = None

    def check_protection_state(self):
        """
        query a GPIB Command in order to get alim status.

        :rtype: str
        :return: The response from agilent for protection state
        """
        status = ""
        status_cmd = "STAT:QUES:COND?"
        # check the power supply status
        results = self.query_command(status_cmd)
        self.get_logger().info("Status is : %s" % str(results))
        # process the result
        if int(results) == 0:
            self.get_logger().info("Status is good")
            return "good"
        if (int(results) & 0b0000000000000001) == 1:
            self.get_logger().warning("Alarm : Over voltage protection found on output 1")
            status += "over_voltage_1- "
        if (int(results) & 0b0000000000000010) == 2:
            self.get_logger().warning("Alarm : Over current protection found on output 1")
            status += "over_current_1 - "
        if (int(results) & 0b0000000000010000) == 16:
            self.get_logger().warning("Alarm : Over temperature protection found ")
            status += "over_temperature - "
        if (int(results) & 0b0000000000100000) == 32:
            self.get_logger().warning("Alarm : Sense wire opened found")
            status += "sense_opened - "
        if (int(results) & 0b0000010000000000) == 1024:
            self.get_logger().warning("Alarm : Output 1 unreguled")
            status += "unreguled_1 - "
        if (int(results) & 0b0100000000000000) == 16384:
            self.get_logger().warning("Alarm : Measurement overload")
            status += "overload - "
        if status == "":
            status += "unknown status :%s" % str(results)
            self.get_logger().info("Status is unknown : %s" % str(results))
        return status
