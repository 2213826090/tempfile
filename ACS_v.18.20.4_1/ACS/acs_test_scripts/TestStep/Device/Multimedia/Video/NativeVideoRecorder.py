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

:organization: INTEL NDG SW
:summary: This file contains the test steps on video recorder
:since 25/03/2015
:author: pblunie
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class NativeVideoRecorderBase(DeviceTestStepBase):
    """
    Implements the base test step for Video recording
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._recorder_api = self._device.get_uecmd("VideoRecorder")

    def _run(self):
        """
        Launch the specific code for the inner test step
        """
        pass

    def run(self, context):
        """
        Run the test step
        """
        DeviceTestStepBase.run(self, context)

        self._run()


class StartNativeVideoRecorder(NativeVideoRecorderBase):
    """
    Start the video recorder
    """

    def _run(self):
        """
        Run the inner code of video recording
        """
        self._recorder_api.kill_camera("camera_google")
        filename = self._recorder_api.native_video_record(self._pars.video_save_path,
                                                          "camera_google",
                                                          self._pars.camera,
                                                          self._pars.quality,
                                                          self._pars.flash_mode,
                                                          self._pars.color_effect,
                                                          self._pars.white_balance,
                                                          self._pars.dvs,
                                                          self._pars.noise_reduction)

        self._context.set_info(self._pars.save_as, filename)


class StopNativeVideoRecorder(NativeVideoRecorderBase):
    """
    Stop the video recording
    """
    def _run(self):
        """
        Stop the native video recorder
        """
        self._recorder_api.stop_video_record(self._pars.video_save_path,
                                             self._pars.filename)
