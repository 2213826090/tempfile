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

@summary: This TestStep implements playing an audio file on host machine.
@since: 6/30/2015
@author: yun.wei@intel.com
@organization: PEG-SVE-SEA


"""

import os
import time
import sys
import acs_test_scripts.Utilities.AudioUtilities as AudioUtils
from Core.TestStep.EquipmentTestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException


class PlayAudioOnHost(TestStepBase):
    """
    Plays audio on host computer
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._audio_host_api = AudioUtils.AudioPlayer(self._logger);

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._logger.info("PlayAudioOnHost: Run")

        audio_file_path = self._pars.audio_file_path
        duration = self._pars.duration
        self._logger.info(duration);

        # Check if audio file path is valid
        if not os.path.isfile(audio_file_path):
            msg = "Audio file not found: " + audio_file_path
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        # Check if wav file
        if not audio_file_path.lower().endswith('.wav'):
            msg = "Please use a PCM encoded WAV file"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        self._audio_host_api.pc_audio_playback_start(audio_file_path, None, True);
        # Play duration in seconds
        time.sleep(duration)
        self._audio_host_api.pc_audio_playback_stop()

        self._logger.info("PlayAudioOnHost: Done")
