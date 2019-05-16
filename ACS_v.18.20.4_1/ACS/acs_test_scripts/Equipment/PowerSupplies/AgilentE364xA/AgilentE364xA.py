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
:summary: implementation of Agilent E364xA power supply
:since: 10/03/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.PowerSupplies.Interface.IPowerSupply import IPowerSupply
from acs_test_scripts.Equipment.PowerSupplies.AgilentE364xA.Wrapper import WAgilentE364xA as W
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment


class AgilentE364xA(IPowerSupply, DllLoader):

    """
    Class AgilentE364xA: implementation of Agilent E364xA power supply
    """

    __CTS_RANGE = ["LOW", "HIGH"]
    __CTS_LOW_VOLTAGE_RANGE = 8
    __CTS_HIGH_VOLTAGE_RANGE = 20

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
        self.__bench_params = bench_params
        self.__handle = None
        self.__eqt_params = eqt_params
        self.__visa = None
        self.__name = name
        self.__model = model
        self._max_range_value = self.__CTS_LOW_VOLTAGE_RANGE

    def __del__(self):
        """
        Destructor: releases all allocated resources.
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
            (err, handle, msg) = W.Connect(self, board_id, gpib_addr)
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
        Initializes the power supply:
           - connection to the power supply
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
            msg = "Unsupported transport %s for %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            connect = getattr(self, "_" + self.__class__.__name__ +
                              "__connect_via_" + transport)
            connect()
        else:
            msg = "%s transport for %s is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Sets basic parameters of power supply
        self.configure_basic_parameters()

    def release(self):
        """
        Releases all resources allocated to power supply use
        """
        self.get_logger().info("Release")
        if self.get_handle() is not None:
            # Disconnection
            (err, msg) = W.Disconnect(self)
            self.__error_check(err, msg)
            self._set_handle(None)
            # Unload driver
            self.unload_driver()

    def perform_full_preset(self):
        """
        Resets all power supply parameters to their default values
        """
        (err, msg) = W.PerformFullPreset(self)
        self.__error_check(err, msg)

    def set_max_current(self, max_current, port):
        """
        This function sets the maximum current allowed of power supply
        :param max_current: maximum current allowed
        :param port: port number on which the current level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SetMaxCurrent(self, max_current, port)
        self.__error_check(err, msg)

    def set_current_voltage(self, voltage, port):
        """
        This function sets the current voltage of power supply
        :param voltage: current voltage to set
        :param port: port number on which the voltage level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        #check range value and change it when necessary
        if voltage >= self.__CTS_LOW_VOLTAGE_RANGE and self._max_range_value != self.__CTS_HIGH_VOLTAGE_RANGE:
            self.set_voltage_range("HIGH")
        elif voltage <= self.__CTS_LOW_VOLTAGE_RANGE and self._max_range_value != self.__CTS_LOW_VOLTAGE_RANGE:
            self.set_voltage_range("LOW")
        elif voltage > self.__CTS_HIGH_VOLTAGE_RANGE:
            msg = "expected voltage %s V is too high for this power supply model" % (str(voltage))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        time.sleep(0.5)
        (err, msg) = W.SetVoltageLevel(self, voltage, port)
        self.__error_check(err, msg)

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
        self.__error_check(err, msg)
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
            self.__error_check(err, msg)
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
        self.__error_check(err, msg)

    def power_off(self):
        """
        This function powers off the power supply
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        (err, msg) = W.SwitchOff(self)
        self.__error_check(err, msg)

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        (err, eqt_id, msg) = W.GetEqID(self)
        self.__error_check(err, msg)
        eqt_id.replace('"', '')  # pylint: disable=E1101
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

    def configure_basic_parameters(self):
        """
        Configure basic parameters of the power supply
        :type name: str
        :param name : the bench name of the power supply
        """
        self.perform_full_preset()

        ps_params = self.__bench_params
        # Sort power supply parameters list
        ps_params_list = ps_params.get_parameters_name()
        ps_params_list.sort()

        for attr_name in ps_params_list:

            if "OUTPUT" in attr_name:

                output_params = ps_params.get_parameters(attr_name)
                output_type = output_params.get_param_value("Type")

                # Set output properties if type is not NONE
                if output_type == "NONE":
                    continue

                port = int(output_params.get_param_value("PortNumber"))
                max_current = float(output_params.get_param_value("MaxCurrent"))
                voltage = float(output_params.get_param_value("Voltage"))

                # Set the max current using CurrrentLevel and PortNumber
                self.set_max_current(max_current, port)
                # Set voltage level using VoltageLevel and PortNumber
                self.set_current_voltage(voltage, port)
                #set the voltage range to low as default config
                self.set_voltage_range("LOW")

    def set_voltage_protection_level(self, level):
        """
        Sets the overvoltage protection level
        :type level: float
        :param level: the level to set
        """

        # Get agilent equipment with pyvisa interface
        self.__visa = VisaEquipment(self.__name,
                                    self.__model,
                                    self.__eqt_params,
                                    self.__bench_params)

        self.__visa.connect()

        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command : *CLS")
        self.__visa.write("*CLS")

        # write the command
        self.get_logger().info("set the voltage protection to %s" % str(level))
        self.__visa.write("VOLT:PROT:LEV %s" % str(level))
        time.sleep(0.2)
        self.__visa.write("VOLT:PROT:STAT ON")
        time.sleep(0.2)
        self.__visa.write("VOLT:PROT:CLE")
        time.sleep(0.2)

        # Release agilent equipment with pyvisa interface
        self.__visa.disconnect()
        self.__visa = None

    def set_voltage_range(self, rang):
        """
        Sets the voltage range to LOW (8V-5A) or HIGH (20V-2.5A)

        :type rang: string
        :param rang: can be low or LOW or high or HIGH
        """

        if not rang.upper() in self.__CTS_RANGE:
            msg = "unknown range type %s" % (str(rang))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Get agilent equipment with pyvisa interface
        self.__visa = VisaEquipment(self.__name,
                                    self.__model,
                                    self.__eqt_params,
                                    self.__bench_params)

        self.__visa.connect()

        # Send command *CLS to the equipment to clear the Error Queue
        self.get_logger().debug("Send GPIB command : *CLS")
        self.__visa.write("*CLS")

        # write the command
        self.get_logger().info("set the voltage range to %s" % str(rang))
        self.__visa.write("VOLT:RANG %s" % str(rang.upper()))
        time.sleep(0.2)

        #update range status
        if rang.upper() == "HIGH":
            self._max_range_value = self.__CTS_HIGH_VOLTAGE_RANGE
        elif rang.upper == "LOW":
            self._max_range_value = self.__CTS_LOW_VOLTAGE_RANGE


        # Release agilent equipment with pyvisa interface
        self.__visa.disconnect()
        self.__visa = None
