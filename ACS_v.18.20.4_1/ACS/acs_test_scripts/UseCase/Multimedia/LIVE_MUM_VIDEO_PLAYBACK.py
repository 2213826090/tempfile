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
:summary: This file implements use case to execute a video playback
:since: 23/01/2013
:author: ssavrimoutou
"""

import time
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global, is_number
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveMumVideoPlayback(UseCaseBase):

    """
    Class LiveMumVideoPlayback.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._global_config = global_config

        # Initialize variables
        self._video_filename = str(self._tc_parameters.get_param_value("FILENAME", ""))

        self._multimedia_volume = self._tc_parameters.get_param_value("VOLUME", -1, int)

        self._screen_orientation = self._tc_parameters.get_param_value("SCREEN_ORIENTATION", "")

        self._video_duration = self._tc_parameters.get_param_value("DURATION")
        if self._video_duration in [None, ""]:
            self._video_duration = ""
        elif not str(self._video_duration).isdigit():
            self._video_duration = -1
        else:
            self._video_duration = float(self._video_duration)

        self._repeat_playback = False
        self._play_whole_file = False
        self._settledown_duration = 0

        # Get UECmdLayer
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._video_api = self._device.get_uecmd("Video")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check value of FILENAME parameter
        if self._video_filename == "" or is_number(self._video_filename):
            error_msg = "The parameter FILENAME must a string containing the local video file !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check value of VOLUME parameter
        if self._multimedia_volume < 0 or self._multimedia_volume > 100:
            error_msg = "The parameter VOLUME must be an integer between 0 and 100 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check value of SCREEN_ORIENTATION
        valid_screen_orientation = ["portrait", "landscape", "reverse_portrait", "reverse_landscape"]
        if self._screen_orientation not in (valid_screen_orientation + [None, ""]):
            error_msg = "The parameter SCREEN_ORIENTATION must be a string with following " \
                        "values: %s or empty value!" % ("".join(valid_screen_orientation))
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Get video file duration
        video_file_duration = self._multimedia_api.get_media_file_duration(self._video_filename)

        # Check value of DURATION parameter
        if not self._video_duration:
            self.get_logger().warning("The parameter DURATION is empty! The whole video file will be played.")
            self._video_duration = video_file_duration
            self._play_whole_file = True
            # when playing video until end of file, there is a delay on DUT side before considering playback complete
            self._settledown_duration = 5

        elif self._video_duration <= 0:
            error_msg = "The parameter DURATION must be an integer strictly upper than 0 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        elif self._video_duration > video_file_duration:
            self.get_logger().warning("The parameter DURATION is greater than video file duration ! The "
                                      "whole video file will be repeated until duration is reached.")
            self._repeat_playback = True

        # Disable device lock screen
        self._phonesystem_api.disable_lockscreen(True)
        # Set phone to keep the device display on and wake it up
        self._phonesystem_api.display_on()
        self._phonesystem_api.wake_screen()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        # Set volume using VOLUME parameter
        time.sleep(self._wait_btwn_cmd)
        self._system_api.adjust_specified_stream_volume("Media", self._multimedia_volume)

        # Launch the video file using FILENAME parameter
        time.sleep(self._wait_btwn_cmd)
        self._video_api.play(self._video_filename, self._repeat_playback, self._screen_orientation)

        # check every second that video is still playing
        is_playing = True
        duration = 0
        while is_playing and duration < self._video_duration + self._settledown_duration:
            time.sleep(1)
            is_playing, duration = self._video_api.is_playing()

        if is_playing:
            # when no duration is defined, playback should stop automatically at the end of the playback
            if self._play_whole_file:
                error_msg = "Video playback should have stopped once complete, but it has not"
                self.get_logger().error(error_msg)
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, error_msg)

        elif duration < (self._video_duration - 0.5):  # subtract 0.5 second for possible timer deviation
            error_msg = "Video playback was interrupted"
            self.get_logger().error(error_msg)
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, error_msg)


        # Stop the video file and quit player
        self._video_api.stop()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        if self._phonesystem_api.get_screen_status():
            # Set phone screen off
            self._phonesystem_api.display_off()

        return Global.SUCCESS, "No errors"
