"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
:organization: INTEL MCG PSI
:summary: This file implements the Test Steps to deal with User mode image capture
:since: 10/07/2015
:author: tchourrx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class CloseAudioPlayer(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step
        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._audio_api = self._device.get_uecmd("Audio")

        self._logger.debug("Stop audio")
        if self._pars.music_player and self._pars.music_player.lower() == "native":
            self._audio_api.stop_native()
        else:
            self._audio_api.stop()
