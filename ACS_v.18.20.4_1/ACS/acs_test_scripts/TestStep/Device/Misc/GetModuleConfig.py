#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

@organization: INTEL MCG PSI
@summary: This module implements a step to read a device module configuration
@since: 13/10/14
@author: kturban
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Parsers.ParserUtil import get_dict_value_from_path

class GetModuleConfig(DeviceTestStepBase):
    """
    Get a module config value
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        module_config_key = self._pars.key
        module_name = self._pars.module
        # get all configuration of this type of module
        module_configs = [module.configuration for module in self._device.get_device_modules(module_name)]
        if not module_configs:
            error_msg = "Device does not have module : '{0:s}' !".format(module_name)
            raise DeviceException(DeviceException.INVALID_PARAMETER, error_msg)
        else:
            for module_config in module_configs:
                param_ref_value = get_dict_value_from_path(module_config, module_config_key, None)
                if param_ref_value is not None:
                    # Update the parameter value using the module config
                    context.set_info(self._pars.module_value, param_ref_value)
                    verdict_msg = "Device module config key found for {0} : '{1}'='{2}'".format(module_name,
                                                                                                module_config_key,
                                                                                                param_ref_value)

                    self._logger.debug(verdict_msg)
                    self.ts_verdict_msg = verdict_msg
                    break
            else:
                raise DeviceException(DeviceException.INVALID_PARAMETER,
                                         "'{0:s}' module config key does not exist for module {1}!".format(
                                             module_config_key, module_name))

