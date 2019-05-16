"""
@summary: This file implements a Test Step to run youtube_camera_gfx.apk on the device.  This
    application was created by the DSV FV (functional validation) team, and can be
    found in Artifactory under acs_test_artifacts/CONCURRENCY/TESTS/youtube_camera_gfx.
    It creates 2 SurfaceViews.  One contains the YouTube video, and the other contains
    a the camera preview.  A spinning cube is rendered into an overlay on top of the 2 views.
    The graphics stress can be disabled by setting the ENABLE_GFX parameter to '0'.
@since 15 May 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
"""

import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class RunSplitScreenYoutubeCameraGfx(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.system_api = self._device.get_uecmd("System")
        self.app_api = self._device.get_uecmd("AppMgmt")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info("YoutubeCameraGfx: Run")

        package = self._pars.app_package
        activity = self._pars.activity
        duration = self._pars.duration
        enable_gfx = self._pars.enable_gfx
        device_logger = self._device.get_device_logger()

        system_api = self._device.get_uecmd("System")

        #Adding triglog messages to capture the initialization of the following modules:
        #- Youtube_camera_gfx application
        #- OpenGLRender
        #- OMX Codec
        #- Camera HAL. The Camera HAL which talks directly to the Camera driver enables the camera preview
        #- Audio Streamming, this triglog verifies that the Audio start streaming through Audio Intel HAL
        conc_start_messages = self._device.get_app_start_messages(package_name="com.example.youtube_camera_gfx", app_name="Youtube_Camera_Gfx")
        # [12/17/2014 JONG] : Commented out out dated start messages (WW50 Android releases)
        #conc_start_messages = [
        #    self._device.get_app_start_messages(package_name="com.example.youtube_camera_gfx", app_name="Youtube_Camera_Gfx"),
        #    self._device.get_gfx_start_messages(),
        #    self._device.get_video_decoder_start_messages(),
        #    self._device.get_camera_start_messages(),
        #    self._device.get_audio_playback_start_messages()
        #]

        # Add triglogs for applications and drivers
        system_api.start_log_monitoring(device_logger=device_logger, target_messages=conc_start_messages)

        pkg_activity = package+"/."+ activity

        self.app_api.launch_app(intent=pkg_activity, extras="-e time %d -e gfxactive %d"%(duration, enable_gfx))

        time.sleep(5)

        #Check logs for messages that indicate successful start of the activity
        system_api.check_logged_start_msgs(device_logger=device_logger, target_messages=conc_start_messages)

        #Remove trigger messages for start
        system_api.stop_log_monitoring(device_logger=device_logger, target_messages=conc_start_messages)

        #Make sure we're still on top, in case the test case launched other apps (e.g. audio playback) that started after us.
        self.app_api.launch_app(intent=pkg_activity, extras="-e time %d -e gfxactive %d"%(duration, enable_gfx))

        self._logger.info("YoutubeCameraGfx: Waiting %d minutes before starting verification"%duration)
        time.sleep(duration * 60)

        self._logger.info("YoutubeCameraGfx: Done")

        #TODO: Check that app was still running just before the time was done.
