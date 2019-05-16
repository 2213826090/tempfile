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
:summary: This file implements use case to execute a video playback with unplug during video playback
:since: 23/01/2013
:author: jcoutox
"""

import time
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global, is_number
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveMumVideoPlaybackUnplugPlug(UseCaseBase):

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

        self._player = self._tc_parameters.get_param_value("PLAYER")

        self._playback_duration = int(self._tc_parameters.get_param_value("PLAYBACK_DURATION"))

        if self._player == "NATIVE":
            self._video_filename = "/storage/emulated/0/acs_files/" + self._video_filename

        if self._video_duration in [None, ""]:
            self._video_duration = ""
        elif not str(self._video_duration).isdigit():
            self._video_duration = -1
        else:
            self._video_duration = float(self._video_duration)

        self._repeat_playback = False
        self._play_whole_file = False
        self._settledown_duration = 0
        self._mediaStoreName = "external"

        # Get UECmdLayer
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._display_api = self._device.get_uecmd("Display")
        self._video_api = self._device.get_uecmd("Video")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Launch media scanner
        self._multimedia_api.launch_media_scanner("sdcard")

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

        verdict = Global.FAILURE
        msg = ""

        # Set volume using VOLUME parameter
        time.sleep(self._wait_btwn_cmd)
        self._system_api.adjust_specified_stream_volume("Media", self._multimedia_volume)

        if self._player == "ACS_AGENT":

            # Launch the video file using FILENAME parameter
            time.sleep(self._wait_btwn_cmd)
            self._video_api.play(self._video_filename, False, self._screen_orientation)
            time.sleep(self._wait_btwn_cmd)

            if not self._video_api.is_playing():
                return Global.FAILURE, "Failed to launch video playback!"

        elif self._player == "NATIVE":
            # Get the video ID from the real video path
            video_id = self._multimedia_api.get_video_id_from_path(self._video_filename, self._mediaStoreName)
            time.sleep(self._wait_btwn_cmd)

            # Set orientation
            self._display_api.set_display_orientation(self._screen_orientation.lower())
            time.sleep(self._wait_btwn_cmd)

            # Launch video from is ID
            self._multimedia_api.play_video_from_ID_with_Google_Photo(video_id)
            time.sleep(self._wait_btwn_cmd)

            if not self._multimedia_api.Google_Photo_is_playing_video():
                return Global.FAILURE, "Failed to launch video playback!"

        else:
            return Global.FAILURE, "Wrong PLAYER parameter!"

        # disconnect the board
        self._device.disconnect_board()
        time.sleep(self._wait_btwn_cmd)

        # UNPLUG
        self._io_card.usb_connector(False)

        self._logger.info("Wait video palyback duration = " + str(self._playback_duration) + " s.")

        # Wait for PLUG
        time.sleep(self._playback_duration)

        self._logger.info("Connecting the DUT : PLUG")

        # PLUG
        self._io_card.usb_connector(True)
        time.sleep(self._wait_btwn_cmd)

        # connect the board
        self._device.connect_board()
        time.sleep(self._wait_btwn_cmd)

        if self._player == "ACS_AGENT":
            is_playing, duration = self._video_api.is_playing()
        elif self._player == "NATIVE":
            is_playing = self._multimedia_api.Google_Photo_is_playing_video()
            duration = self._video_duration
        else:
            verdict = Global.FAILURE
            msg = "Wrong PLAYER parameter!"

        if is_playing and duration >= self._playback_duration:
            verdict = Global.SUCCESS
            msg = "No errors"
            # when no duration is defined, playback should stop automatically at the end of the playback
            if self._play_whole_file:
                error_msg = "Video playback should have stopped once complete, but it has not"
                self.get_logger().error(error_msg)
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, error_msg)

        elif duration < (self._video_duration - 0.5):  # subtract 0.5 second for possible timer deviation
            error_msg = "Video playback was interrupted"
            self.get_logger().error(error_msg)
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, error_msg)

        else:
            verdict = Global.FAILURE
            msg = "Video playback failed. Play status = " + str(is_playing) + ", Playback duration = " + str(duration)

        # Stop the video file and quit player
        if self._player == "ACS_AGENT":
            self._video_api.stop()
        elif self._player == "NATIVE":
            self._keyevent_api.back()
            self._keyevent_api.back()
            self._keyevent_api.back()
            self._keyevent_api.home()

        return verdict, msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)
        if self._player == "NATIVE":
            # Set auto
            self._display_api.set_display_orientation("auto")
            time.sleep(self._wait_btwn_cmd)

        if self._phonesystem_api.get_screen_status():
            # Set phone screen off
            self._phonesystem_api.display_off()

        return Global.SUCCESS, "No errors"
