"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements Test Step for WiFi AP Base
:since 11/07/2014
:author: jfranchx
"""
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException


class APBase(EquipmentTestStepBase):
    """
    Implements WiFi AP Base class
    """

    DEFAULT_CONFIGURABLE_AP = "CONFIGURABLE_AP1"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._configurable_ap = None
        self._configurable_ap_params = global_conf.benchConfig.get_parameters(self._pars.eqt)
        self._default_configurable_ap_params = global_conf.benchConfig.get_parameters(self.DEFAULT_CONFIGURABLE_AP)

#------------------------------------------------------------------------------

    def run(self, context):
        """
        Run the step
        """
        EquipmentTestStepBase.run(self, context)

        self._configurable_ap = self._equipment_manager.get_configurable_ap(self._pars.eqt)

        if self._configurable_ap is None:
            msg = "EQT %s not found - Check your bench config" % self._pars.eqt
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._default_configurable_ap_params is None:
            msg = "Default configurable AP %s not found in bench config - no default parameters available" % self.DEFAULT_CONFIGURABLE_AP
            self._logger.debug(msg)

#------------------------------------------------------------------------------

    def _get_bc_ap_param(self, param):
        """
        Get the AP parameter value from the bench config.
        In case of not using the default AP, and absence of parameter,
        it returns the value for the default AP.

        :type param: str
        :param param: Bench config AP parameter name
        :rtype: String
        :return: Value of the parameter in the bench configuration file
        """
        if not self._configurable_ap_params:
            # This error should not occurs in production
            msg = "TestStep coding error. Please instantiate the AP BenchConfig parameters object"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INSTANTIATION_ERROR, msg)

        # Retrieve the default value if exists
        default_value = None
        if self._default_configurable_ap_params and self._default_configurable_ap_params.has_parameter(param):
            default_value = self._default_configurable_ap_params.get_param_value(param)

        # Retrieve the bench config value
        value = self._configurable_ap_params.get_param_value(param, default_value)

        return value