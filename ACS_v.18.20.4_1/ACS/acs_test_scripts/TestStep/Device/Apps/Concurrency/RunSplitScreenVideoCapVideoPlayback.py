"""
@summary: This file implements a Test Step to run VideoCapture_VideoPlayback.apk on the device.  This
    application was created by the DSV FV (functional validation) team, and can be
    found in Artifactory under acs_test_artifacts/CONCURRENCY/TESTS/vplay_vcap_hdmi.
    It creates 2 SurfaceViews.  One contains the video playback, and the other contains
    a the video recording. Make sure to connect HDMI cable.
    *** This test runs only on android and cannot be implemented on windows.
@since 5 October 2014
@author: Santhosh Reddy D
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

class RunSplitScreenVideoCapVideoPlayback(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.system_api = self._device.get_uecmd("System")
        self.app_api = self._device.get_uecmd("AppMgmt")
        self.display_api =self._device.get_uecmd("Display")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")


    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info(self._pars.id +" : Test Run")

        device_logger = self._device.get_device_logger()

        #Adding triglog messages to capture the initialization of the following modules:
        #- Video Playback and Video caapture application
        #- OMX Codec
        #- Camera HAL. The Camera HAL which talks directly to the Camera driver enables the camera preview
        #- Audio Streamming, this triglog verifies that the Audio start streaming through Audio Intel HAL
        # [12/12/14 Jong: Some of these messages are gone as of r5_ww50. So Commenting out]
        #triglogApplist = [ ["PackageActivity " + 'com.cv.cv_videocapture_videoplayback', "ActivityManager: Start proc com.cv.cv_videocapture_videoplayback for activity com.cv.cv_videocapture_videoplayback/.CVVideoCaptureVideoPlaybackActivity"],
        #           ["OMXCodec ","OMXCodec: [OMX.Intel.VideoDecoder.AVC] video dimensions are"],
        #           ["Camera driver atomisp loaded ", "Camera_HAL: atom_preview_enabled"],
        #           ["Audio Streamming ","AudioIntelHAL: startStream: output stream"]]
        triglogApplist = self._device.get_app_start_messages(package_name="com.cv.cv_videocapture_videoplayback", app_name="CVVideoCaptureVideoPlaybackActivity")

        # Add triglogs for applications and drivers
        for triglog in triglogApplist:
            device_logger.add_trigger_message(triglog[1])

        # Check for system UI
        systemui_ok =self._phone_system_api.check_process("systemui")
        if systemui_ok :
            self._logger.info(self._pars.id +" : Systemui is active")

        else:
            self._logger.error(self._pars.id +" : Systemui is not active.")
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id +" : Systemui is not active.")

        pkg_activity = self._pars.app_package+"/."+ self._pars.activity

        self.app_api.launch_app(intent=pkg_activity, extras="-e vpath {0} -e time {1} ".format(self._pars.video_file, self._pars.duration))
        #Check logs for messages that indicate successful start of the activity
        for triglog in triglogApplist:
            time.sleep(1)
            messages= device_logger.get_message_triggered_status(triglog[1])
            if len(messages) == 0:
                msg = "{2} :{0} failed to start properly. Could not find this message in the log: {1} ".format(triglog[0],triglog[1],self._pars.id)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #Remove application start-up messages.
        for triglog in triglogApplist:
            device_logger.remove_trigger_message(triglog[1])

        self._logger.info("%s : Waiting %d minutes to complete the test."%(self._pars.id,self._pars.duration))
        time.sleep(self._pars.duration * 60)

        self._logger.info(self._pars.id +" : Done")

