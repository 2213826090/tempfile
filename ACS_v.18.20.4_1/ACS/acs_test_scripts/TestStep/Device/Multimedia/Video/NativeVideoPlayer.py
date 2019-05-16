"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL PEG SVE DSV
:summary: This file implements a Test Step to play video files.
:since: 05/02/15
:author: pblunie
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class NativePlayerBase(DeviceTestStepBase):
    """
    Base class for all steps with native player
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf,
                                    ts_conf, factory)

        # Load Multimedia ue command.
        self._video_api = self._device.get_uecmd("Video")
        self._system_api = self._device.get_uecmd("System")

    def _run(self):
        """
        Specific code to override in test step implementation
        """

    def run(self, context):
        """
        Run the step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._run()


class PlayNativeVideo(NativePlayerBase):
    """
    Play a video with the native player
    """
    def _run(self):
        """
        Specific code of the Play step for native player
        """

        # Get Video Playback parameters
        video_file = self._pars.video_file
        volume = self._pars.volume
        delay_time_s = int(self._pars.delay_s)

        self._system_api.adjust_specified_stream_volume("Media", volume)
        self._video_api.play_native(video_file, delay_time_s)
        start_date = time.time() + delay_time_s

        self._context.set_info(self._pars.save_as, start_date)


class CloseNativePlayer(NativePlayerBase):
    """
    Close the native player
    """
    def _run(self):
        """
        Specific code of the Clode step for native player

        """
        self._video_api.close_native()
