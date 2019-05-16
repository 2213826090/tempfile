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
:summary: implementation of Dli_WebPowerSwitch5 power supplies
:since: 29/05/2013
:author: lbavois
"""

import os
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.PowerSupplies.Interface.IPowerSupply import IPowerSupply
import dlipower as dli
from acs_test_scripts.Equipment.IEquipment import EquipmentBase


class Dli_WebPowerSwitch5(EquipmentBase, IPowerSupply):

    """
    Implementation class of WEB POWER SWITCH power supply
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
        IPowerSupply.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)

        # Initialize attributes
        self._model = model
        self._name = name
        self._bench_params = bench_params

        # Initialize all necessary Dli_WebPowerSwitch5 equipment parameters
        self._ip = ""
        self._username = ""
        self._password = ""
        self._portNumber = 0
        self._switch = None

    def _connect_via_TCPIP(self):
        """
        Connect to equipment via TCPIP
        """

        # Disable proxy to access to the fixed IP address of the power supply.
        no_proxy_var = os.getenv("no_proxy", "")
        os.putenv("no_proxy", no_proxy_var + ',' + self._ip)

        # Set up the PowerSwitch with ip default parameters present in default config file path
        self._switch = dli.PowerSwitch(self._username, self._password, self._ip)

        # Verify we can talk to the switch
        if not self._switch.verify():
            self.get_logger().error("Unable to connect to Dli_WebPowerSwitch5 equipment")
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, "Unable to connect to Dli_WebPowerSwitch5 equipment")
        else:
            self.get_logger().info("Successful connection to Dli_WebPowerSwitch5 equipment")

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def init(self):
        """
        Initializes the power supply:
           - connection to the power supply
        """
        self.get_logger().info("Initialization")

        # Get transport mode and try to connect to equipment
        transport = str(self._bench_params.get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s for %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            # Sets basic parameters of power supply
            self.configure_basic_parameters()

            connect = getattr(self, "_connect_via_" + transport)
            connect()
        else:
            msg = "%s transport for %s is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        # for Dli_WebPowerSwitch5, no parameter reset needed
        pass

    def perform_full_preset(self):
        # for Dli_WebPowerSwitch5, no parameter reset needed
        pass

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        return "Dli_WebPowerSwitch5"

    def configure_basic_parameters(self):
        """
        Configure basic parameters of the power supply
        :type name: str
        :param name : the bench name of the power supply
        """

        output_presence_flag = False

        # fixed device's ip used to create connection
        self._ip = str(self._bench_params.get_param_value("IP"))
        self._username = str(self._bench_params.get_param_value("username"))
        self._password = str(self._bench_params.get_param_value("password"))

        if self._ip == "" or self._username == "" or self._password == "":
            error_msg = "Dli_WebPowerSwitch5 Power Supply init issue: Check IP, username, password parameter values in Bench Config file"
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

        # Sort power supply parameters list
        ps_params_list = sorted(self._bench_params.get_parameters_name())

        for attr_name in ps_params_list:

            if "OUTPUT" in attr_name:
                if not output_presence_flag:

                    output_params = self._bench_params.get_parameters(attr_name)

                    output_type = output_params.get_param_value("Type")

                    # Set output properties if type is not NONE
                    if "ACCHG" in output_type:
                        # The power supply type is correctly configured => continue

                        # the reference number [1..8] of the outlet controlled over Dli_WebPowerSwitch5
                        self._portNumber = int(output_params.get_param_value("PortNumber"))

                        if self._portNumber not in range(0, 9):
                            error_msg = "Dli_WebPowerSwitch5 Power Supply init issue: Wrong PortNumber (=%d) parameter values in Bench Config file" % self._portNumber
                            self.get_logger().error(error_msg)
                            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)
                        else:
                            # The Dli_WebPowerSwitch5 equipment implementation supports only one OUTPUT
                            output_presence_flag = True
                    else:
                        error_msg = "Dli_WebPowerSwitch5 Power Supply init issue: Wrong Output type (=%s) parameter in Bench Config file. Output type should contain ACCHG" % output_type
                        self.get_logger().error(error_msg)
                        raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)
                else:
                    # Inform the user that this power supply equipment implementation supports only one OUTPUT
                    error_msg = "Dli_WebPowerSwitch5 Power Supply init issue: only one output is supported for this equipment. To use another PortNumber on the same equipment create a new Power Supply equipment with one output in Bench Config file"
                    self.get_logger().error(error_msg)
                    raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

    def power_on(self):
        # Switch on plug Number of the Web Power Switch
        if self._switch.on(self._portNumber):
            # if return value is True, we fail to turn on the Port Number
            self.get_logger().error("Dli_WebPowerSwitch5 switch on failure for PortNumber %s" % str(self._portNumber))
            TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, "Dli_WebPowerSwitch5 switch on failure for PortNumber %s" % str(self._portNumber))

    def power_off(self):
        # Switch off plug Number of the Web Power Switch
        if self._switch.off(self._portNumber):
            # if return value is True, we fail to turn off the Port Number
            self.get_logger().error("Dli_WebPowerSwitch5 switch off failure for PortNumber %s" % str(self._portNumber))
            TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, "Dli_WebPowerSwitch5 switch off failure for PortNumber %s" % str(self._portNumber))

    def power_cycle(self):
        self.power_off()
        self.power_on()
