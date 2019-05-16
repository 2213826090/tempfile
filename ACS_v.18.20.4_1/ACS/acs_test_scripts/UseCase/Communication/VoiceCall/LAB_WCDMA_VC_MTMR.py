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
:summary: This file implements the WCDMA Voice Call Mobile Terminated / Mobile release UC
:since: 19/08/2010
:author: ssa
"""

from UtilitiesFWK.Utilities import Global
from LAB_WCDMA_VC_BASE import LabWcdmaVcBase


class LabWcdmaVcMtMr(LabWcdmaVcBase):

    """
    Lab Wcdma Voice Call MT/MR.
    """
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call WCDMA VoiceCall base run_test function
        LabWcdmaVcBase.run_test(self)

        # Release any previous call (Robustness)
        self._voice_call_api.release()

        # Mobile Terminated originate call
        self._ns_voice_call_3g.mt_originate_call()

        # pylint: disable=E1101
        # Wait for state "incoming" before callSetupTimeout seconds
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        # Answer call
        self._voice_call_api.answer()

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_3g.check_call_connected(self._call_setup_time,
                                                    blocking=False)

        # Wait for state "active" before callSetupTimeout seconds
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # Check call is connected for x seconds
        # (x is CALL_DURATION value in seconds)
        self._ns_voice_call_3g.is_voice_call_connected(self._call_duration)

        # Mobile Release call
        self._voice_call_api.release()

        # Check voice call state is "IDLE"
        self._ns_voice_call_3g.check_call_state("IDLE",
                                                self._call_setup_time,
                                                blocking=False)

        # Check voice call state is "no_call"
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        # pylint: enable=E1101
        return Global.SUCCESS, "No errors"
