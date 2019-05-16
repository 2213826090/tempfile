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

:organization: INTEL AMPS
:summary: This file implements use case to record video after setting flash mode to torch
:since: 22/01/2014
:author: mmorchex
"""

import os
import time
import datetime
import acs_test_scripts.Lib.VideoCheck.VideoCheck as VideoCheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraVideoFlashModeTorch(UseCaseBase):

    """
    Class Live Camera video flash mode torch.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get RECORD_DURATION from TC parameters
        self.__record_duration = self._tc_parameters.get_param_value("RECORD_DURATION", default_cast_type=int)

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'), "")

        # Camera name : BACK cause of flash usage
        self.__camera = "BACK"

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check record duration
        if self.__record_duration <= 0:
            error_msg = "Please update TC and set a record duration that is > 0 milliseconds"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check save directory
        if self.__save_directory in (None, ""):
            error_msg = "Please update TC and set a saving directory"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Open camera
        self.__camera_api.launch_camera(self.__camera)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        msg = "No errors"

        # Start video record
        video_uri = self.__camera_api.record_video(self.__record_duration, self.__save_directory, 0)
        time.sleep(self._wait_btwn_cmd)

       # Get video from DUT to host
        local_video_uri = self.__camera_api.download_media_file(video_uri, self.__save_folder)

       # Get video's brightness
        local_video_brightness = VideoCheck.check_video_brightness(local_video_uri)

       # Set camera flash mode to torch
        self.__camera_api.set_flash_mode('torch')
        time.sleep(self._wait_btwn_cmd)

        # Start video record with flash mode
        video_with_flash_uri = self.__camera_api.record_video(self.__record_duration, self.__save_directory, 0)
        time.sleep(self._wait_btwn_cmd)

        # Get video from DUT to host
        local_video_with_flash_uri = self.__camera_api.download_media_file(video_with_flash_uri, self.__save_folder)

        # Get video with flash mode torch's brightness
        local_video_with_flash_brightness = VideoCheck.check_video_brightness(local_video_with_flash_uri)

        if local_video_with_flash_brightness <= local_video_brightness:
            verdict = Global.FAILURE
            msg = "Flash mode was not set during video record"

        return (verdict, msg)

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # close camera
        self.__camera_api.shutdown()
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")
