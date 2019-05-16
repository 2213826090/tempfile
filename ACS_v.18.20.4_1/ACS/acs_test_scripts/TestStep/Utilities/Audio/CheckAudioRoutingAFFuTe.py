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

@summary: This TestStep implements checking audio routing using AFFuTe python framework
@since: 7/30/2015
@author: fbelvezx
@organization: CCG-CRD-OPM PC&WTE
"""


from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Utilities.Audio.AFFuTeBase import AFFuTeBase


class CheckAudioRoutingAFFuTe(AFFuTeBase):
    """
    Check audio routing with AFFuTe python framework
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        AFFuTeBase.run(self, context)

        # Start audio routing check on the DUT
        [verdict, self.ts_verdict_msg] = self._audio_fwk_api.start_audio_routing("DUT",
                                                                                 self._pars.audio_file_path,
                                                                                 int(self._pars.audio_verdict_timeout),
                                                                                 self._pars.wait_for_timeout,
                                                                                 start_playback=False,
                                                                                 stop_playback=False)

        if verdict < 0:
            raise DeviceException(DeviceException.OPERATION_FAILED, self.ts_verdict_msg)

        # Release connection with AFFuTe
        self._audio_fwk_api.release_fwk()