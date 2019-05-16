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
:summary: This file implements use case to record video in HD
:since: 22/01/2014
:author: mmorchex
"""

import os
import time
import datetime
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
import acs_test_scripts.Lib.VideoCheck.VideoCheck as VideoCheck


class LiveCameraVideoRecordHd(UseCaseBase):

    """
    Class Live Camera video HD record.
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

        self.__hd_resolution = (1920, 1080)


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
        msg = "No error"

        # Start video record, quality is set to lowest resolution of the camera
        # supported by camera
        self._logger.info("Start recording video in low quality ...")
        video_uri = self.__camera_api.record_video(self.__record_duration, self.__save_directory, 0,
                                                   VideoCheck.VIDEO_QUALITY["QUALITY_LOW"])
        time.sleep(self._wait_btwn_cmd)

        # Get video from DUT
        local_video_uri = self.__camera_api.download_media_file(video_uri, self.__save_folder)

        # Start video record, quality is set to 1080p
        self._logger.info("Start recording video in HD quality ...")
        video_hd_uri = self.__camera_api.record_video(self.__record_duration, self.__save_directory, 0,
                                                      VideoCheck.VIDEO_QUALITY["QUALITY_1080P"])
        time.sleep(self._wait_btwn_cmd)

        # Get video from DUT
        local_video_hd_uri = self.__camera_api.download_media_file(video_hd_uri, self.__save_folder)

        # Check video recorded in HD resolution
        video_hd_resolution, operation_status = VideoCheck.check_video_resolution(local_video_hd_uri)

        self._logger.info("Checking video resolution ...")
        if not operation_status or video_hd_resolution != self.__hd_resolution:
            self._logger.error("Video resolution is not as expected")
            verdict = Global.FAILURE
            msg = "Bad video resolution %s, required resolution is : %s" % (str(video_hd_resolution),
                                                                      str(self.__hd_resolution))
        else:
            self._logger.info("Video resolution is as expected")

        self._logger.info("Checking video size ...")
        # Check that video recorded in HD resolution's size is bigger than the video recorded in low quality's size
        size_result = VideoCheck.compare_videos_size(local_video_hd_uri, local_video_uri)

        if not size_result:
            self._logger.error("Video size is not as expected")
            verdict = Global.FAILURE
            msg = " Failed to record video in HD"
        else:
            self._logger.info("Video size is as expected")

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
