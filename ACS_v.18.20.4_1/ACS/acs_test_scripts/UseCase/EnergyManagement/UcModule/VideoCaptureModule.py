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
:summary: This module represent video capture module allowing you to record a video with special option.
:author: vgomberx
:since: 17/12/2014
"""
from time import sleep
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class VideoCaptureModule():
    """
    init.
    """
    __LOG_TAG = "[VIDEOCAPTURE_MODULE]\t"

    def __init__(self):
        """
        parameter to initialize this module.
        Different way to capture the video should be added in a generic way.
        :type mode: str
        """
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        device = overmind.get_instance(overmind.DEVICE)
        tc_parameters = overmind.get_instance(overmind.TC_PARAMETERS)

        #-----------------------------------------------------------------------
        # Get TC Parameters
        self.__camera_name = tc_parameters.get_param_value("CAMERA_NAME")
        self.__quality = tc_parameters.get_param_value("QUALITY", None)

        # Get UECmdLayer
        self.__video_record_api = device.get_uecmd("VideoRecorder", True)
        # var
        self.file_path = None

    def setup(self):
        """
        setup the camera
        """
        self.__logger.info(self.__LOG_TAG + "Setup recording according to your testcase parameter")
        self.__video_record_api.setup_camera(camera=self.__camera_name,
                        back_quality=self.__quality,
                        skip_wizard="ON")

    def start_recording(self):
        """
        start video recording

        @rtype: str
        @return: return the video name
        """
        self.__logger.info(self.__LOG_TAG + "Start recording")
        # Kill camera package to make next upload video profile can work
        self.__video_record_api.force_stop(self.__camera_name)
        sleep(5)
        # Starting the video recording
        a, b = self.__video_record_api.record(self.__camera_name)
        self.file_path = b
        return a, b

    def stop_recording(self):
        """
        stop the video recording
        """
        self.__logger.info(self.__LOG_TAG + "Stop recording")
        self.__video_record_api.stop_record(self.__camera_name)
        self.file_path = None

    def start_recording_load(self):
        """
        start video recording for load

        @rtype: str
        @return: return the video name
        """
        self.__logger.info(self.__LOG_TAG + "Start recording")
        # Kill camera package to make next upload video profile can work
        self.__video_record_api.force_stop(self.__camera_name)
        sleep(5)
        self.__video_record_api.clean_video_storage(self.__camera_name)
        # Starting the video recording
        a, b = self.__video_record_api.record(self.__camera_name)
        self.file_path = b
        return a, b

    def check_recording_state(self):
        """
        start video recording

        @rtype: str
        @return: return the video name
        """
        self.__logger.info(self.__LOG_TAG + "Start recording")
        return self.__video_record_api.check_video_state(self.file_path)
