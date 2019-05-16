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

:organization: INTEL OTC
:summary: This file is the process of LiveVideoQuality
:since: 109/05/2014
:author: mmaracix
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import Lib.VideoCheck.VideoCheck as VideoCheck
import os
import shutil
import datetime


class LiveVideoQuality(UseCaseBase):
    """
    Class Live Video Playback Quality.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._video_filename = str(self._tc_parameters.get_param_value("FILENAME", ""))

        self._screen_orientation = self._tc_parameters.get_param_value("SCREEN_ORIENTATION", "")

        self._repeat_playback = False
        self._play_whole_file = True
        self._settledown_duration = 0

        self._video_file = self._tc_parameters.get_param_value("FILENAME")
        self._video_reference = self._tc_parameters.get_param_value("VIDEO_REFERENCE")
        self._wait_before_record = int(self._tc_parameters.get_param_value("WAIT_BEFORE_RECORD"))
        self._duration = int(self._tc_parameters.get_param_value("DURATION"))
        self._disp_brightness = self._tc_parameters.get_param_value("DISP_BRIGHTNESS")
        self._screenrecord_bit_rate = int(self._tc_parameters.get_param_value("BIT_RATE"))
        self._screenrecord_time_limit = int(self._tc_parameters.get_param_value("TIME_LIMIT"))
        self._save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")
        self._save_directory_host = self._tc_parameters.get_param_value("SAVE_DIRECTORY_HOST")
        self._video_orientation = self._tc_parameters.get_param_value("SCREEN_ORIENTATION")

        #Set variable
        self._mediaStoreName = "external"
        self._save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), "")

        #Create screenrecord name.
        screenrecord_name = self._video_file.split("/")
        screenrecord_name = screenrecord_name[-1].split(".")
        self._screenrecord_name = screenrecord_name[-2] + "_screenrecord_" + \
                                  datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"

        # Get UECmdLayer
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._video_api = self._device.get_uecmd("Video")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._camera_api = self._device.get_uecmd("Camera")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")

    #------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Set display brightness to %s%%..." % self._disp_brightness)
        self._phone_system_api.set_display_brightness(self._disp_brightness)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Wake up the screen...")
        self._phone_system_api.wake_screen()

        self._logger.info("Turn on the display")
        self._phone_system_api.display_on()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Unlock the screen...")
        self._phone_system_api.set_phone_lock(0)

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        UseCaseBase.run_test(self)

        # Launch the video file using FILENAME parameter
        time.sleep(self._wait_btwn_cmd)
        self._video_api.play(self._video_filename, self._repeat_playback, self._screen_orientation)

        #Take screenrecord
        self._logger.info("Take screenrecord starts here")
        self._multimedia_api.take_screenrecord(self._screenrecord_name, save_directory=self._save_directory,
                                               bit_rate=self._screenrecord_bit_rate,
                                               time_limit=self._screenrecord_time_limit, verbose=True)

        #Wait during duration
        time.sleep(self._duration + self._wait_btwn_cmd)

        #Stop video playback
        self._logger.info("Stop playing video file...")
        self._keyevent_api.back()
        time.sleep(self._wait_btwn_cmd)

        #Pull screenrecord in report file
        local_video_file = self._camera_api.download_media_file(self._save_directory + self._screenrecord_name,
                                                                self._save_folder)

        #Compare screenrecord with reference video
        self._logger.info("Compare recorded and reference file.")
        verdict_bool, msg = VideoCheck.compare_two_video_frame_by_frame(local_video_file, self._video_reference,
                                                                        self._screenrecord_time_limit)

        #Concatenate result
        if verdict_bool:
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        return verdict, msg

    #------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call LiveVideoPlayback tear_down function
        UseCaseBase.tear_down(self)

        if self._save_directory_host != None:
            self._logger.info("Move screenrecord to save directory on host.")
            report_path = self._device.get_report_tree().get_report_path()
            shutil.move(os.path.join(report_path, self._save_folder), self._save_directory_host)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Sets brightness to automatic mode")
        self._phone_system_api.set_brightness_mode("automatic")

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Turn off the display")
        self._phone_system_api.display_off()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Lock the screen after image display...")
        self._phone_system_api.set_phone_lock(1)

        return self._error.Code, "No errors"
