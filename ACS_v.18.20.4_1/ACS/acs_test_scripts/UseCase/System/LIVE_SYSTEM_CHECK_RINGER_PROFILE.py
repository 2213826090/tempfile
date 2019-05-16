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
:summary: This file is the process of LiveSystemCheckRingerProfile
:since: 11/14/2011
:author: xzhao24
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveSystemCheckRingerProfile(UseCaseBase):

    """
    Live system ringer profile check.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._target_silent_mode = self._tc_parameters.get_param_value("SILENT_MODE")
        self._target_vibra_mode = self._tc_parameters.get_param_value("VIBRA_MODE")

        # Get UECmdLayer
        self._ringer_profile_api = self._device.get_uecmd("PhoneSystem")

        # Save previous mode
        self._previous_mode = self._ringer_profile_api.get_silent_vibrate()

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run function
        UseCaseBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Ringer profile checking...")

        self._ringer_profile_api.set_silent_vibrate(self._target_silent_mode, self._target_vibra_mode)
        result = self._ringer_profile_api.get_silent_vibrate()
        if result == self._target_silent_mode + "+" + self._target_vibra_mode:
            self._logger.info("Ringer profile check succeeded")
            testResult = "Target ringer profile check succeeded, silent mode: %s, vibra mode: %s" % \
                (self._target_silent_mode, self._target_vibra_mode)
        else:
            self._logger.info("Ringer profile check failed")
            testResult = "Target ringer profile check failed"

        return Global.SUCCESS, testResult

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Quit ringer profile check, recover default setting...")

        self._previous_mode_split = self._previous_mode.split("+")
        self._previous_silent_mode = self._previous_mode_split[0]
        self._previous_vibra_mode = self._previous_mode_split[1]

        self._ringer_profile_api.set_silent_vibrate(self._previous_silent_mode, self._previous_vibra_mode)

        return self._error.Code, "No errors"
