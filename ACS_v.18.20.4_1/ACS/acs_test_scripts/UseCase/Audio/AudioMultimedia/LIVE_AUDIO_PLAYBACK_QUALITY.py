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
:summary: This file is the process of LiveAudioPlayback
:since: 30/01/14
:author: jcoutox
"""

import acs_test_scripts.Lib.AudioCheck.AudioCheck as AudioCheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Multimedia.LIVE_AUDIO_PLAYBACK import LiveAudioPlayback
import time


class LiveAudioPlaybackQuality(LiveAudioPlayback):
    """
    Class Live Audio playback quality.
    """

    def __init__(self, tc_name, global_config):

        """
        Constructor
        """

        # Call UseCase base Init function
        LiveAudioPlayback.__init__(self, tc_name, global_config)

        #Read sequence and volume value from teh testcase
        self._sequence = self._tc_parameters.get_param_value("SEQUENCE")
        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))

        # Get UECmdLayer
        self._system_api = self._device.get_uecmd("System")

    def set_up(self):

        """
        Initialize the test
        """

        # Call UseCase base Setup function
        LiveAudioPlayback.set_up(self)

        #Adjust media volume
        self._system_api.adjust_specified_stream_volume("Media", self._volume)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

    def run_test(self):

        """
        Execute the test
        """

        seq_string = ""

        # Call UseCase base Run function
        LiveAudioPlayback.run_test(self)

        self._logger.info("Apply FFT on WAV file for extract frequency.")
        freq = AudioCheck.fft_on_wav_file(self._host_record_file)

        self._logger.info("Replace remarkable frequency by note and extract sequence.")
        seq = AudioCheck.extract_sequence(freq)

        #Create str with sequence
        for i in seq:
            seq_string = seq_string + i + " "

        if self._sequence in seq_string:
            verdict = Global.SUCCESS
            msg = "No errors"
        else:
            verdict = Global.FAILURE
            msg = "Original and record music are not identical"

        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        LiveAudioPlayback.tear_down(self)

        #Adjust media volume at 50%
        self._system_api.adjust_specified_stream_volume("Media", 50)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
