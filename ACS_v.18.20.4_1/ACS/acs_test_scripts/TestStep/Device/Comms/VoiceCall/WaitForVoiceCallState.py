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
:summary: This file implements a Test Step to wait for state of voice call
:since: 23/06/2014
:author: jfranchx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE


class WaitForVoiceCallState(DeviceTestStepBase):
    """
    Implements the test step to wait for state of voice call
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._voice_call_api = self._device.get_uecmd("VoiceCall")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)

        assert hasattr(VOICE_CALL_STATE, self._pars.vc_state)

        if self._pars.vc_state is None:
            msg = "Expected state %s is invalid" % self._pars.state
            self._logger.error(msg)
            self._raise_config_exception(msg, AcsConfigException.INVALID_PARAMETER)

        self._voice_call_api.wait_for_state(getattr(VOICE_CALL_STATE,self._pars.vc_state), self._pars.timeout)
