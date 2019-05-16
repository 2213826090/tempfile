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
:summary: This file implements use case to take picture while camera is recording a video
:since: 22/01/2014
:author: mmorchex
"""

import os
import time
import datetime
import acs_test_scripts.Lib.VideoCheck.VideoCheck as VideoCheck
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraTakePictureWhileVideoRecord(UseCaseBase):

    """
    Class Live Camera take picture while video record.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get RECORD_DURATION from TC parameters
        self.__record_duration = self._tc_parameters.get_param_value("RECORD_DURATION", default_cast_type=int)

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'), "")

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
        msg = ""

        # Start video record
        video_uri, picture_uri = self.__camera_api.record_video(self.__record_duration, self.__save_directory, 1)
        time.sleep(self._wait_btwn_cmd)

        # Download video recorded from the DUT
        local_video_uri = self.__camera_api.download_media_file(video_uri, self.__save_folder)

        # Download picture taken from the DUT
        local_picture_uri = self.__camera_api.download_media_file(picture_uri, self.__save_folder)

        # Check if video recorded is corrupted
        self._logger.info("Checking if video recorded is corrupted ...")
        video_state = VideoCheck.check_video_corrupt(local_video_uri)

        # Check if picture taken is corrupted
        self._logger.info("Checking if picture taken is corrupted ...")
        picture_state = Imagecheck.check_image_corrupt(local_picture_uri)

        if not video_state:
            verdict = Global.FAILURE
            msg = " Video recorded is corrupted, "

        if picture_state:
            verdict = Global.FAILURE
            msg += " Picture taken is corrupted, "

        if msg == "":
            msg = "No errors"

        return (verdict, msg)

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Release camera
        self.__camera_api.shutdown()
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")
