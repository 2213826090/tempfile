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
:summary: This file implements use case to display image when there is a screen rotation
:since: 12/02/2014
:author: mmorchex
"""

import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
import time
import os


class LiveDisplayImageRotate(UseCaseBase):
    """
    Class Display Image Rotate.
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

        # Image library
        self.__dic_image = Imagecheck.generate_dictionary_image_path(self.__library_image_path)

        self.__pictures_uri = []
        self.__time_for_display = 2
        self.__orientations = None
        self.__rotate_generated_pictures = False
        self.__wait_for_icons = 5

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
        elif self.__default_orientation.lower() == 'portrait':
            self.__orientations = ['landscape', 'reverse_landscape']
        else:
            self.__orientations = ['reverse_portrait', 'portrait']

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

        self._logger.info("Downloading the picture to display ...")
        # Get the image to display from DUT
        local_image_to_display = self.__image_api.pull_on_host(self.__image_uri, os.getcwd())
        time.sleep(self._wait_btwn_cmd)

        if local_image_to_display:

            self.__pictures_uri.append(local_image_to_display)
            self._logger.info("Opening the picture to display on Gallery...")

            # Open the image to display
            self.__multimedia_api.display_image_photos(self.__image_uri,
                                                          self.__dic_image[self.__photos_icon],
                                                          self.__dic_image[self.__just_once_icon],
                                                          os.getcwd())
            time.sleep(self.__time_for_display)

            # Take a screen shot of the image displayed to check its
            # resolution and tap on the center of the screen to display off icons
            screen_shot_with_icons_uri = self.__image_api.take_screenshot_and_pull_on_host("screen_shot_with_icons",
                                                                                           os.getcwd())
            self.__pictures_uri.append(screen_shot_with_icons_uri)

            # get screen size
            (screen_shot_width, screen_shot_height) = Imagecheck.get_picture_size(screen_shot_with_icons_uri)

            # tap on the center of the screen
            self.__key_event_api.tap_on_screen(screen_shot_width / 2, screen_shot_height / 2)

            # wait for icons
            self._logger.info("Waiting for icons ...")
            time.sleep(self.__wait_for_icons)

            # Change orientation
            self.__display_api.set_display_orientation(self.__orientations[0].lower())
            time.sleep(self.__time_for_display)

            # Take screen shot
            landscape_screen_shot_uri = self.__image_api.take_screenshot_and_pull_on_host("landscape_screen_shot",
                                                                                          os.getcwd())

            self.__pictures_uri.append(landscape_screen_shot_uri)

            # Change orientation
            self.__display_api.set_display_orientation(self.__orientations[1].lower())
            time.sleep(self.__time_for_display)

            # Take screen shot
            reverse_landscape_screen_shot_uri = self.__image_api. \
                take_screenshot_and_pull_on_host("reverse_landscape_screen_shot", os.getcwd())

            self.__pictures_uri.append(reverse_landscape_screen_shot_uri)

            if landscape_screen_shot_uri and reverse_landscape_screen_shot_uri:

                # Get screen shots's resolutions

                (landscape_screen_shot_width, landscape_screen_shot_height) = Imagecheck. \
                    get_picture_size(landscape_screen_shot_uri)

                (reverse_landscape_screen_shot_width, reverse_landscape_screen_shot_height) = Imagecheck. \
                    get_picture_size(reverse_landscape_screen_shot_uri)

                # Create local landscape picture using the original one
                local_landscape_picture = Imagecheck.set_orientation(local_image_to_display, "landscape", os.getcwd())
                self.__pictures_uri.append(local_landscape_picture)

                local_landscape_picture_resized = Imagecheck.resize_picture(local_landscape_picture,
                                                                            landscape_screen_shot_width,
                                                                            landscape_screen_shot_height,
                                                                            os.getcwd(),
                                                                            self.__rotate_generated_pictures, True)
                self.__pictures_uri.append(local_landscape_picture_resized)

                # Create local reverse landscape picture using the original one
                local_reverse_landscape_picture = Imagecheck. \
                    set_orientation(local_image_to_display, "reverse_landscape", os.getcwd())
                self.__pictures_uri.append(local_reverse_landscape_picture)

                local_reverse_landscape_picture_resized = Imagecheck.resize_picture(local_reverse_landscape_picture,
                                                                                    reverse_landscape_screen_shot_width,
                                                                                    reverse_landscape_screen_shot_height
                                                                                    , os.getcwd(),
                                                                                    self.__rotate_generated_pictures,
                                                                                    True)
                self.__pictures_uri.append(local_reverse_landscape_picture_resized)

                match_landscape = Imagecheck.match_template_in_image(landscape_screen_shot_uri,
                                                                     local_landscape_picture_resized, 0.9)

                match_reverse_landscape = Imagecheck.match_template_in_image(reverse_landscape_screen_shot_uri,
                                                                             local_reverse_landscape_picture_resized,
                                                                             0.9)

                if not match_landscape[0] or not match_reverse_landscape[0]:
                    verdict = Global.FAILURE
                    msg = "Fail to correctly display picture"
            else:
                verdict = Global.FAILURE
                msg = "Can't get Screen Shots"

        else:
            verdict = Global.FAILURE
            msg = "Can't get the picture to display from DUT"

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
