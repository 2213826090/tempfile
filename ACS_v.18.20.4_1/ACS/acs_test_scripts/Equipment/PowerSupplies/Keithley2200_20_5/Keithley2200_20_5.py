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
:summary: implementation of Keithley2200_20_5 power supplies
:since: 18/01/2016
:author: droseyx
"""

from acs_test_scripts.Equipment.PowerSupplies.Interface.IPowerSupply import IPowerSupply
from Core.Report.ACSLogging import LOGGER_EQT
import time
import re

class Keithley2200_20_5(IPowerSupply):

    """
    Implementation class of Keithley2200 power supply
    """

    # Logger
    __logger = LOGGER_EQT

    def get_logger(self):
        """
        Gets the equipment manager logger
        """
        return self.__logger

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
        # Initialize attributes
        self._bench_params = bench_params
        self.__handle = None
        self._visa = None
        self._idn = ""
        self._name = name
        self._model = model
        self._eqt_params = eqt_params
        self._rm = None

    def __del__(self):
        """
        Destructor
        """
        self._disconnect()

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
        if self._visa is not None:
            self.get_logger().info("Keithley2200 already initialized")
            return

        self.get_logger().info("Initialization of Keithley2200 power supply")

        # Sets basic parameters of power supply
        self.configure_basic_parameters()
        time.sleep(0.5)

    def _connect(self):
        """
        Connect to power supply
        """
        resources_list = []
        self._rm = None
        self._visa = None

        try:
            import visa
            self._rm = visa.ResourceManager()
            resources_list = list(self._rm.list_resources())
        except:
            self.get_logger().warning("Cannot import visa, so we will not use power supply")

        if resources_list and self._rm:
            vendor_id = str(self._bench_params.get_param_value("USBVendorID"))
            device_model = str(self._bench_params.get_param_value("USBDeviceModel"))
            pattern = "%s::%s::" % (vendor_id, device_model)
            for resource in resources_list:
                try:
                    if re.search(pattern, resource):
                        self._idn = resource
                        self._visa = self._rm.open_resource(resource)
                        self.get_logger().info("[Keithley2200_20_5] Power supply connected")
                        break
                finally:
                    pass

    def _disconnect(self):
        """
        Disconnect power supply
        """
        self._write("SYST:LOC")
        if self._visa is not None:
            self._visa.close()
            self._visa = None
        if self._rm is not None:
            self._rm.close()
            self._rm = None
        self.get_logger().info("[Keithley2200_20_5] Power supply disconnected")

    def set_output_state(self, state, output):
        """
        Sets the output state
        :type state: str
        :param state: the state to set ("ON" | "OFF")
        :type output: integer
        :param output: the output to set
        """
        if state == "ON":
            self.power_on()
        elif state == "OFF":
            self.power_off()

    def power_on(self):
        """
        This function powers on the power supply
        :raise: raises TestEquipmentException in case of failure
        """
        self._connect()
        self.get_logger().info("[Keithley2200_20_5] Enabling output")
        self._write("OUTPut 1")
        self._write("OUTPut?")
        self._disconnect()

    def power_off(self):
        """
        This function powers off the power supply
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        self._connect()
        self.get_logger().info("[Keithley2200_20_5] Disabling output")
        self._write("OUTPut 0")
        self._write("OUTPut?")
        self._disconnect()

    def set_current_voltage(self, voltage, port):
        """
        This function sets the current voltage of power supply
        :param voltage: current voltage to set
        :param port: port number on which the voltage level has to be set
        :rtype: none
        :raise: raises TestEquipmentException in case of failure
        """
        # Set voltage level using VoltageLevel
        self._connect()
        self.get_logger().info("[Keithley2200_20_5] Setting voltage to %s" % str(voltage))
        self._write("VOLT:LEV %s" % str(voltage))
        self._write("MEAS:VOLT?")
        self._disconnect()

    def configure_basic_parameters(self):
        """
        Configuration of basic parameters of the power supply
        """
        voltage = float(self._bench_params.get_param_value("Voltage"))
        # Disable the output
        self.set_output_state("OFF", 1)
        # Set voltage level using VoltageLevel
        self.set_current_voltage(voltage, 1)
        # Enable the output
        self.set_output_state("ON", 1)

    def _write(self, command):
        """
        Send command to the power supply
        :type command: str
        :param command: the command
        """
        if self._visa is not None:
            try:
                self._visa.write(str(command))
            except:
                self.get_logger().warning("Cannot send command %s to power supply" % str(command))

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        return self._idn

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

    def query_command(self, command):
        """
        query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """
        ret = ""
        self._connect()
        if self._visa:
            try:
                self.get_logger().info("Query command : %s" % command)
                response = self._visa.query(command)
                ret = response
            except:
                self.get_logger().error("Could not send command : %s" % command)
        self._disconnect()
        return ret
