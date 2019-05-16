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
:summary: implementation of Agilent 66319D power supplies
:since: 31/03/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.PowerSupplies.Agilent66311D.Agilent66311D import Agilent66311D
from acs_test_scripts.Equipment.PowerSupplies.Common.Wrapper import WAgilent663xxD as W
from acs_test_scripts.Equipment.VisaEquipment import VisaEquipment
import time


class Agilent66319D(Agilent66311D):

    """
    Implementation class of Agilent 66319D power supply
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
        Agilent66311D.__init__(self, name, model, eqt_params, bench_params)

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def init(self):
        """
        Initializes the power supply.
        Final state: the power supply is ready to use
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        # Load the equipment driver
        # Get transport mode and try to connect to equipment
        self.connect_power_supply()

        # Sets basic parameters of power supply
        self.configure_basic_parameters()
        time.sleep(5)

    def set_coupling_mode(self, mode):
        """
        Sets the coupling mode
        :type mode: str
        :param mode: the mode to set ("ALL" | "NONE")
        """
        (err, msg) = W.SetCouplingMode(self, mode)
        self._error_check(err, msg)

    def configure_basic_parameters(self):
        """
        Configure basic parameters of the power supply and register power supply
        outputs in equipment catalog dictionary
        """
        self.perform_full_preset()

        ps_params = self.get_bench_params()
        # Sort power supply parameters list
        ps_params_list = ps_params.get_parameters_name()
        ps_params_list.sort()

        # The 66319  power supply has two output that can be coupled.
        # For safety matter, it is better not to couple them.
        if ps_params.has_parameter("CouplingMode"):
            mode = str(ps_params.get_param_value("CouplingMode"))
            self.set_coupling_mode(mode)

        # Set the compensation mode
        if ps_params.has_parameter("CompensationMode"):
            mode = str(ps_params.get_param_value("CompensationMode"))
            self.set_compensation_mode(mode)

        # Set sense protection using
        if ps_params.has_parameter("SenseProtect"):
            protect = str(ps_params.get_param_value("SenseProtect"))
            W.SetSenseProtect(self, protect)
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

        # Set resistance using Resistance parameter
        if ps_params.has_parameter("Resistance"):
            resistance = float(ps_params.get_param_value("Resistance"))
            self.set_resistance(resistance)
        # Set the protection delay using delay parameter
        if ps_params.has_parameter("ProtectDelay"):
            delay = float(ps_params.get_param_value("ProtectDelay"))
            self.set_protection_delay(delay)

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

    def connect_power_supply(self):
        """
        connect the power supply
        :return: none
        """
        # reinit DllLoader object
        # Load the equipment driver
        self.load_driver()
        # Get transport mode and try to connect to equipment
        transport = str(self.get_bench_params().get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s for %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            connect = getattr(self, "_Agilent66311D__connect_via_" + transport)
            connect()
        else:
            msg = "%s transport for %s is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

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
        if (int(results) & 0b0000000000100000) == 256:
            self.get_logger().warning("Alarm : Output 2 unreguled")
            status += "unreguled_2 - "
        if (int(results) & 0b0000010000000000) == 1024:
            self.get_logger().warning("Alarm : Output 1 unreguled")
            status += "unreguled_1 - "
        if (int(results) & 0b0001000000000000) == 4096:
            self.get_logger().warning("Alarm : Over current protection found on output 2")
            status += "over_current_2 - "
        if (int(results) & 0b0100000000000000) == 16384:
            self.get_logger().warning("Alarm : Measurement overload")
            status += "overload - "
        if status == "":
            status += "unknown status :%s" % str(results)
            self.get_logger().info("Status is unknown : %s" % str(results))
        return status

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
        self._visa.write("OUTP2 OFF")
        time.sleep(0.2)
        self._visa.write("OUTP:PROT:CLE")
        time.sleep(0.2)
        self._visa.write("OUTP1 ON")
        time.sleep(0.2)
        self._visa.write("OUTP2 ON")

        # Release agilent equipment with pyvisa interface
        self._visa.disconnect()
        self._visa = None
