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

@summary: This TestStep implements audio playback stop with AFFuTe python framework
@since: 09/21/2015
@author: fbelvezx
@organization: CCG-CRD-OPM PC&WTE
"""

from acs_test_scripts.TestStep.Utilities.Audio.AFFuTeBase import AFFuTeBase


class StopAudioPlaybackAFFuTe(AFFuTeBase):
    """
    Stop audio playback with AFFuTe python framework
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        AFFuTeBase.run(self, context)

        self._audio_fwk_api.stop("WAV_PLAY", "DUT")

        # Release connection with AFFuTe
        self._audio_fwk_api.release_fwk()