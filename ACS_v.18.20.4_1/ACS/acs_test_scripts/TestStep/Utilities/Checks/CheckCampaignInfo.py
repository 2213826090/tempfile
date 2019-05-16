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

:organization: INTEL PEG-SVE-DSV
:summary: This file implements a Test Step to check if a KEY was in the campaign
            configuration file and store the value found into PARAM_VALUE as a string.
            Use DEFAULT_VALUE for PARAM_VALUE if KEY was not found in the configuration file.
:since:13/10/2014
:author: sasmith2
"""
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
import UtilitiesFWK.Utilities as Utils
from Device.DeviceManager import DeviceManager


class CheckCampaignInfo(TestStepBase):
    """
    Check that a key is present in the configuration file and store found value as a context parameter.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Looks for the key in the campaign configuration file.
        value = Utils.get_config_value(DeviceManager().get_global_config().campaignConfig, "Campaign Config", self._pars.key)
        if value is None:
            self._logger.info("{0}:  Value found for {1} in campaign config was None, storing the default value of {2} instead".format(self._pars.id, self._pars.key, self._pars.default_value))
            value = self._pars.default_value
        # Write value to the context so that it can be used by other TestSteps.
        context.set_info(self._pars.param_value, value)
