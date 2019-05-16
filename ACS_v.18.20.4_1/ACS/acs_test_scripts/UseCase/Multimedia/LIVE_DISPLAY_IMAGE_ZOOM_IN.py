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
:summary: This file implements use case to display image and zoom in
:since: 12/02/2014
:author: mmorchex
"""

import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
import time
import os


class LiveDisplayImageZoomIn(UseCaseBase):

    """
    Class Display Image Zoom In.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get IMAGE_URI from TC parameters
        self.__image_to_display_uri = self._tc_parameters.get_param_value("IMAGE_URI")

        # get SCRIPT_PATH from TC parameters
        self.__script = \
            str(self._tc_parameters.get_param_value("SCRIPT_PATH"))

        # get ERROR_MANAGEMENT_POLICY from TC parameters
        self._error_policy = str(self._tc_parameters.get_param_value("ERROR_MANAGEMENT_POLICY"))

        # get ENABLE_UI_AUTOMATOR_EVENT from TC parameters
        self._ui_automator_event = self._tc_parameters.get_param_value("ENABLE_UI_AUTOMATOR_EVENT",
                                                                       default_value=False,
                                                                       default_cast_type="str_to_bool")

        # get LIBRARY_IMAGE_PATH from TC parameters
        self.__library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        # get REFERENCE_IMAGE_ZOOM_IN from TC parameters
        self.__ref_image_zoom_in = self._tc_parameters.get_param_value("REFERENCE_IMAGE_ZOOM_IN")

        # get PHOTOS_ICON from TC parameters
        self.__photos_icon = self._tc_parameters.get_param_value("PHOTOS_ICON")

        # get JUST_ONCE_ICON from TC parameters
        self.__just_once_icon = self._tc_parameters.get_param_value("JUST_ONCE_ICON")

        # get DEFAULT_ORIENTATION from TC parameters
        self.__default_orientation = self._tc_parameters.get_param_value("DEFAULT_ORIENTATION")

        # Get UECmdLayer.
        self.__image_api = self._device.get_uecmd("Image")
        self.__phone_system_api = self._device.get_uecmd("PhoneSystem")
        self.__display_api = self._device.get_uecmd("Display")
        self.__multimedia_api = self._device.get_uecmd("Multimedia")
        self.__key_event_api = self._device.get_uecmd("KeyEvent")
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)

        #Image library
        self.__dic_image = Imagecheck.generate_dictionary_image_path(self.__library_image_path)

        self.__pictures_uri = []
        self.__wait_for_icons = 5
        self.__time_for_display = 3

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check IMAGE_URI
        if self.__image_to_display_uri in (None, ""):
            error_msg = "Image uri is not set, please update the TC"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        self.__image_uri = "sdcard/acs_files/" + self.__image_to_display_uri

        # Check DEFAULT_ORIENTATION
        if self.__default_orientation is None or self.__default_orientation.lower() not in ['portrait',
                                                                                                 'reverse_landscape']:
            error_msg = "Bad value for default orientation, please update it in the TC "
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Initialize UI object
        self._ui_api.init(self._error_policy, self._ui_automator_event)

        # wake the screen
        self._logger.info("Turn on the display")
        self.__phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        # Set orientation
        self.__display_api.set_display_orientation(self.__default_orientation.lower())
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

        # Open the image to display
        self._logger.info("Displaying image ...")
        self.__multimedia_api.display_image_photos(self.__image_uri,
                                                      self.__dic_image[self.__photos_icon],
                                                      self.__dic_image[self.__just_once_icon],
                                                      os.getcwd())
        time.sleep(self.__time_for_display)

        # Zoom In
        self._logger.info("Zoom in ...")
        self._ui_api.run_action_from_script(self.__script)
        time.sleep(self.__time_for_display)

        # wait for icons
        self._logger.info("Waiting for icons ...")
        time.sleep(self.__wait_for_icons)

        # Take screen shot
        self._logger.info("Taking screen shot ...")
        zoom_in_screen_shot_uri = self.__image_api.take_screenshot_and_pull_on_host("zoom_in_screen_shot", os.getcwd())

        self.__pictures_uri.append(zoom_in_screen_shot_uri)

        # Check that the zoom in is correctly displayed
        result = Imagecheck.match_template_in_image(self.__dic_image[self.__ref_image_zoom_in], zoom_in_screen_shot_uri,
                                                    0.9)
        if not result:
            verdict = Global.FAILURE
            msg = "Zoom in did not match"

        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Remove pictures created and downloaded on host
        self._logger.info("Removing  pictures created or downloaded on host ...")
        for picture_uri in self.__pictures_uri:
            self._logger.info("Removing : %s", picture_uri)
            os.remove(picture_uri)
        self.__pictures_uri = []

        # Set orientation to auto
        self.__display_api.set_display_orientation('auto')
        time.sleep(self._wait_btwn_cmd)

        # Go back to home screen
        self.__key_event_api.home()
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
