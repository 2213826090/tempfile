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
:summary: implementation of Agilent 66321D power supplies
:since: 30/09/2013
:author: dbatut
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.PowerSupplies.Agilent66311D.Agilent66311D import Agilent66311D
from acs_test_scripts.Equipment.PowerSupplies.Common.Wrapper import WAgilent663xxD as W
import time


class Agilent66321D(Agilent66311D):

    """
    Implementation class of Agilent 66321D power supply
    """

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
