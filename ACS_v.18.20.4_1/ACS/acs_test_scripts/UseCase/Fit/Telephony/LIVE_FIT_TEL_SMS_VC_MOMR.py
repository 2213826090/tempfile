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
:summary: test the emission and reception of a SMS
          while voice call is active on the LIVE network
:since: 09/01/2012
:author: jdurand
"""

import time

from acs_test_scripts.UseCase.Communication.Messaging.LIVE_SMS_LOOPBACK import LiveSmsLoopback
from UtilitiesFWK.Utilities import Global


class LiveFitTelSmsVcMoMr(LiveSmsLoopback):

    """
    Live SMS while Voice Call MO/MR.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveSmsLoopback.__init__(self, tc_name, global_config)

        # Voice call variables initialization
        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        # Get call number
        self._call_number = \
            str(self._tc_parameters.get_param_value("CALL_NUMBER"))

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the environment
        """
        LiveSmsLoopback.set_up(self)

        # end potential active voice call
        self._voicecall_api.release()
        print "call %s" % self._call_number

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # initiate a voice call and ensure it is active
        self._voicecall_api.dial(self._call_number)
        time.sleep(self._wait_btwn_cmd)
        self._voicecall_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)  # pylint: disable=E1101

        # execute the sms loopback test
        (sms_loopback_verdict, sms_loopback_message) = \
            LiveSmsLoopback.run_test(self)

        # check whether voice call is still active
        self._voicecall_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)  # pylint: disable=E1101

        # end the active voice call
        self._voicecall_api.release()

        return sms_loopback_verdict, sms_loopback_message
