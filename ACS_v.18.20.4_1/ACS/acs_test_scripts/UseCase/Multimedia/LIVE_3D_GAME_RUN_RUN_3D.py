"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements use case to play Run Run 3D game
:since: 10/04/2014
:author: mmorchex
"""

import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
import Lib.VideoCheck.VideoCheck as VideoCheck
import time
import os


class Live3dGameRunRun3d(UseCaseBase):

    """
    Class Live 3d Game Run Run.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get IMAGE_URI from TC parameters
        self.__image_to_display_uri = self._tc_parameters.get_param_value("IMAGE_URI")

        # get LIBRARY_IMAGE_PATH from TC parameters
        self.__library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        # get GALLERY3D_ICON from TC parameters
        self.__icons = self._tc_parameters.get_param_value("ICONS")

        # get VIDEO_REFERENCE
        self.__video_reference = self._tc_parameters.get_param_value("VIDEO_REFERENCE")

        # get DEFAULT_ORIENTATION from TC parameters
        self.__default_orientation = self._tc_parameters.get_param_value("DEFAULT_ORIENTATION")

        # Get UECmdLayer.
        self.__image_api = self._device.get_uecmd("Image")
        self.__phone_system_api = self._device.get_uecmd("PhoneSystem")
        self.__multimedia_api = self._device.get_uecmd("Multimedia")
        self.__networking_api = self._device.get_uecmd("Networking")
        self.__camera_api = self._device.get_uecmd("Camera")

        # Image library
        self.__dic_image = Imagecheck.generate_dictionary_image_path(self.__library_image_path)

        self.__icons_paths = []
        self.__local_video_file = None
        self.__initial_pdp_context_status = None
        self.__initial_wifi_status = None
        self.__rotate_screen_shot = False
        self.__screen_record = "run_run_3d.mp4"
        self.__dut_save_directory = "/sdcard/"
        self.__video_duration = 3
        self.__bit_rate = 16000000

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check DEFAULT_ORIENTATION
        if self.__default_orientation is None or self.__default_orientation.lower() not in ['portrait',
                                                                                                 'reverse_landscape']:
            error_msg = "Bad value for default orientation, please update it in the TC "
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        elif self.__default_orientation.lower() == 'reverse_landscape':
            self.__rotate_screen_shot = True

        # Get path for icons set in TC
        for icon in self.__icons.split(';'):
            self.__icons_paths.append(self.__dic_image[icon])

        # Store the initial PDP context status
        self.__initial_pdp_context_status = self.__networking_api._get_pdp_context_status()
        time.sleep(self._wait_btwn_cmd)

        # Store the initial wifi state
        self.__initial_wifi_status = self.__networking_api.get_wifi_power_status()
        time.sleep(self._wait_btwn_cmd)

        # Deactivate wifi
        self.__networking_api.set_wifi_power(0)
        time.sleep(self._wait_btwn_cmd)

        # Deactivate PDP context status
        self.__networking_api.deactivate_pdp_context(check=False)
        time.sleep(self._wait_btwn_cmd)

        # Wake the screen
        self._logger.info("Turn on the display")
        self.__phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        # unlock the screen
        self._logger.info("unlock the screen")
        self.__phone_system_api.set_phone_lock(0)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test

        verdict = Global.SUCCESS
        msg = "No errors"

        # Starts Run Run 3D
        self._logger.info("Starting Run Run 3D ...")
        self.__multimedia_api.start_run_run_3d()
        time.sleep(self._wait_btwn_cmd)

        # Starts playing
        self.__multimedia_api.execute_icons_sequence(self.__icons_paths, os.getcwd(), 5, self.__rotate_screen_shot)
        time.sleep(self._wait_btwn_cmd)

        # Start Recording
        self.__multimedia_api.take_screenrecord(self.__screen_record, save_directory=self.__dut_save_directory,
                                                bit_rate=self.__bit_rate, time_limit=self.__video_duration)
        time.sleep(self.__video_duration + self._wait_btwn_cmd)

        # Get video recorded from DUT to Host
        self.__local_video_file = self.__camera_api.download_media_file(self.__dut_save_directory+self.__screen_record
                                                                        , os.getcwd())
        time.sleep(self._wait_btwn_cmd)

        # Compare screenrecord with reference video
        self._logger.info("Compare recorded and reference file.")
        verdict_bool, msg = VideoCheck.compare_two_video_frame_by_frame(self.__local_video_file, self.__video_reference,
                                                                        self.__video_duration, percent_threshold=0.9,
                                                                        match_template_threshold=0.60)

        # Concatenate result
        if verdict_bool:


            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Stop Run Run 3d
        self._logger.info("Stopping Run Run 3D ...")
        self.__multimedia_api.stop_run_run_3d()
        time.sleep(self._wait_btwn_cmd)

        # Put the PDP_CONTEXT on its original state
        if self.__initial_pdp_context_status not in ("0", "2"):
            # Then activate PDP context status
            self.__networking_api.activate_pdp_context()
            time.sleep(self._wait_btwn_cmd)

        # Put wifi on its original state
        self.__networking_api.set_wifi_power(self.__initial_wifi_status)
        time.sleep(self._wait_btwn_cmd)

        # Remove video recorded
        self._logger.info("Removing : %s", self.__local_video_file)
        os.remove(self.__local_video_file)

        return Global.SUCCESS, "No errors"
