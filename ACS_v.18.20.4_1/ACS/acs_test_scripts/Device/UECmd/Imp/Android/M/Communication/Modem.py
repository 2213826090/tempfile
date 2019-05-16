"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG
:summary: This file implements the Modem UEcmd for Android devices
:since: 2015-07-28
:author: mariussx

"""

from acs_test_scripts.Device.UECmd.Imp.Android.LLP.Communication.Modem import Modem as ModemLLP
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class Modem(ModemLLP):
    """
    :summary: Modem UEcommands for Android platform
    """

    @need('modem')
    def __init__(self, device):
        ModemLLP.__init__(self, device)
        self._ims_module = "acscmd.telephony.ImsModule"


    def set_ims_modem_configuration(self, ims_param_keys, params_values):
        """
        Configure the modem with IMS parameters
        :param ims_param_keys: Contains the keys which needs to be set
        :type ims_param_keys: tuple
        :param param_values: Values for the parameters
        :type param_values: tuple
        """

        ims_params_string = ""
        params_values_string = ""
        if (ims_param_keys is None) or (params_values is None):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Parameter mode is not valid !")

        # Concatenate the parameters and values to be sent in the intent
        for param_key in ims_param_keys:
            ims_params_string += str(param_key) + "-"
        for value in params_values:
            params_values_string += str(value) + "-"

        # Remove the last "-" character from the strings computed
        ims_params_string = ims_params_string[:-1]
        params_values_string = params_values_string[:-1]

        function = "setModemImsConfig"
        function_args = "--es ims_params %s --es param_value %s" % (ims_params_string,
                                                                    params_values_string)

        self._internal_exec_v2(self._ims_module,
                               function,
                               function_args,
                               is_system=True)


    def set_single_ims_param(self, ims_param_key, param_value):
        """
        Configure the modem with a single IMS parameter

        :param ims_param_key: Contains the key which needs to be set in modem side
        :type ims_param_key: int
        :param param_value: Value for the parameter
        :type param_value: str or int
        """

        if isinstance(param_value, str):
            function = "setImsConfigStringValueCommand"
            function_args = "--ei param_key " + ims_param_key + " --es value " + param_value
        else:
            function = "setImsConfigIntValueCommand"
            function_args = "--ei param_key " + ims_param_key + " --ei value " + param_value

        self._internal_exec_v2(self._ims_module,
                               function,
                               function_args,
                               is_system=True)


    def turn_ims(self, mode):
        """
        Turn ON or OFF the IMS services
        :param mode: Mode to set the IMS services
        :type mode: str or int

        :return: None
        :raise: AcsConfigException if wrong value for input parameter
        """

        function = "turnIms"

        if mode in ("ON", "on", "1", 1):
            mode_str = "on"
        elif mode in ("OFF", "off", "0", 0):
            mode_str = "off"
        else:
            self._logger.error("Parameter mode %s is not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Parameter mode is not valid !")

        function_args = "--es mode " + mode_str
        self._logger.info("Turn " + mode_str + " IMS services.")

        self._internal_exec_v2(self._ims_module,
                               function,
                               function_args,
                               is_system=True)
