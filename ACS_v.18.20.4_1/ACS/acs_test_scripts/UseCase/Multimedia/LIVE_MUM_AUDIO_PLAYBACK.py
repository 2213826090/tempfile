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
:summary: This file implements use case to execute an audio playback
:since: 21/01/2013
:author: ssavrimoutou
"""

import time
from UtilitiesFWK.Utilities import Global, is_number
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveMumAudioPlayback(UseCaseBase):

    """
    Class Lab Audio playback.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._global_config = global_config

        # Initialize variables
        self._audio_filename = str(self._tc_parameters.get_param_value("FILENAME", ""))

        self._multimedia_volume = self._tc_parameters.get_param_value("VOLUME", -1, int)

        self._audio_duration = self._tc_parameters.get_param_value("DURATION")
        if self._audio_duration in [None, ""]:
            self._audio_duration = ""
        elif not str(self._audio_duration).isdigit():
            self._audio_duration = -1
        else:
            self._audio_duration = float(self._audio_duration)

        # Initialize variable for data connection
        self._use_data_connection = False

        # Get UECmdLayer
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._audio_api = self._device.get_uecmd("Audio")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")

#------------------------------------------------------------------------------
    def _enable_data_connection(self):
        """
        Enable data connection
        """
        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Active pdp context
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context()

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check value of FILENAME parameter
        if self._audio_filename == "" or is_number(self._audio_filename):
            error_msg = "The parameter FILENAME must a string containing the local audio file or an url !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check if needed to enable data for audio streaming
        if self._audio_filename.startswith("http://"):
            self._use_data_connection = True
        else:
            # To prevent b2b iterations
            self._use_data_connection = False

        # Check value of VOLUME parameter
        if self._multimedia_volume < 0 or self._multimedia_volume > 100:
            error_msg = "The parameter VOLUME must be an integer between 0 and 100 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # FILENAME is an url, enable data connection
        if self._use_data_connection:
            self._enable_data_connection()

        # Get audio file duration
        audio_file_duration = self._multimedia_api.get_media_file_duration(self._audio_filename)

        # Check value of DURATION parameter
        if self._audio_duration == "":
            self.get_logger().warning("The parameter DURATION is empty! The whole audio file will be played.")
            self._audio_duration = audio_file_duration
            # Add a delay to make sure the use case play whole the file
            self._audio_duration += 5

        elif self._audio_duration <= 0:
            error_msg = "The parameter DURATION must be an integer strictly upper than 0 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        else:
            if self._audio_duration > audio_file_duration:
                self.get_logger().warning("The parameter DURATION is upper than audio file duration ! "
                                          "The whole audio file will be played.")
                self._audio_duration = audio_file_duration
                # Add a delay to make sure the use case play whole the file
                self._audio_duration += 5

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

        # Launch the audio file using FILENAME parameter
        time.sleep(self._wait_btwn_cmd)
        self._audio_api.play(self._audio_filename, loop=False)

        self.get_logger().info("Wait for %.1f second(s) before stopping audio playback"
                               % self._audio_duration)
        time.sleep(self._audio_duration)

        # Stop the audio file
        self._audio_api.stop()

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

        if self._use_data_connection:
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.deactivate_pdp_context()

        return Global.SUCCESS, "No errors"
