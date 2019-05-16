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

@organization: UMG PSI Validation
@summary: This file implements the step to manage with a host side
          VideoLan player

@author: pblunie
"""
from Core.TestStep.TestStepBase import TestStepBase
from Device.DeviceManager import DeviceManager
import UtilitiesFWK.Utilities as Util


class VideoLANClientBase(TestStepBase):
    """
    Manage MediaPlayer on host
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._media_player = None

    def _run(self):
        """
        Abstract method to run specific code
        """

    def run(self, context):
        """
        Generic method to run MediaPlayer test steps
        """
        TestStepBase.run(self, context)

        host_screen = Util.str_to_bool(DeviceManager().get_device_config("PHONE1").get("DisplayScreenAvailable", "False"))
        if host_screen is not None and host_screen:
            from acs_test_scripts.Lib.EM.VideoPlayback import MediaPlayer as MediaPlayer

            self._media_player = context.get_info(self._pars.instance_name)
            if self._media_player is None:
                self._media_player = MediaPlayer()
                context.set_info(self._pars.instance_name, self._media_player)

            self._run()
        else:
            self._logger.info("Host screen not available, nothing will be displayed")


class PlayVideoWithVLC(VideoLANClientBase):
    """
    Start the video on host with VLC.
    """
    def _run(self):
        """
        Launch video on the host screen
        """
        payload = self._pars.payload
        screen = self._pars.screen
        transform = self._pars.video_transform
        self._media_player.play(payload, screen, transform)


class StopVideoWithVLC(VideoLANClientBase):
    """
    Stop the VLC player
    """
    def _run(self):
        """
        Stop the VLC instance
        """
        self._media_player.stop()
