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
:summary: This module implements check of icon in camera application
:since: 21/01/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import os
import time


class LiveCheckIconCamera(UseCaseBase):
    """
    Class check icon in camera application.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._image_filename = "DUTScreenshot"
        self._screenshot_path = ""
        self._screenshot_state = False

        #Read the path of image library from testcase
        self._library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        #Read four orientation from testcase
        self._first_orientation = self._tc_parameters.get_param_value("FIRST_ORIENTATION")
        self._second_orientation = self._tc_parameters.get_param_value("SECOND_ORIENTATION")
        self._third_orientation = self._tc_parameters.get_param_value("THIRD_ORIENTATION")
        self._final_orientation = self._tc_parameters.get_param_value("FINAL_ORIENTATION")

        #Read value of different image for first orientation from testcase
        self._camera_mode_first_orientation = self._tc_parameters.get_param_value("CAMERA_MODE_FIRST_ORIENTATION")
        self._take_photo_first_orientation = self._tc_parameters.get_param_value("TAKE_PHOTO_FIRST_ORIENTATION")
        self._option_camera_first_orientation = self._tc_parameters.get_param_value("OPTION_CAMERA_FIRST_ORIENTATION")


        #Read value of different image for second orientation from testcase
        self._camera_mode_second_orientation = self._tc_parameters.get_param_value("CAMERA_MODE_SECOND_ORIENTATION")
        self._take_photo_second_orientation = self._tc_parameters.get_param_value("TAKE_PHOTO_SECOND_ORIENTATION")
        self._option_camera_second_orientation = self._tc_parameters.get_param_value("OPTION_CAMERA_SECOND_ORIENTATION")

        #Read value of different image for third orientation from testcase
        self._camera_mode_third_orientation = self._tc_parameters.get_param_value("CAMERA_MODE_THIRD_ORIENTATION")
        self._take_photo_third_orientation = self._tc_parameters.get_param_value("TAKE_PHOTO_THIRD_ORIENTATION")
        self._option_camera_third_orientation = self._tc_parameters.get_param_value("OPTION_CAMERA_THIRD_ORIENTATION")

        #Image library
        self._dic_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._display_api = self._device.get_uecmd("Display")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._camera_api = self._device.get_uecmd("Camera")

        # store initial Sleep Timeout value
        self._initial_sleep_timeout_value = self._phonesystem_api.get_screen_timeout()
        time.sleep(self._wait_btwn_cmd)


    def set_up(self):
        """
        set up
        """
        UseCaseBase.set_up(self)

        # wake the screen
        self._phonesystem_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        # unlock the screen
        self._phonesystem_api.set_phone_lock(0)
        time.sleep(self._wait_btwn_cmd)

        # set sleep timeout to it's maximum value
        self._phonesystem_api.set_screen_timeout(1800)
        time.sleep(self._wait_btwn_cmd)

        #Launch app camera2
        self._camera_api.launch_system_camera_application()
        time.sleep(self._wait_btwn_cmd)

        #Close first geo location popup
        self._keyevent_api.scenario(["move_home", "tab", "tab", "enter"])

        return self._error.Code, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function.
        UseCaseBase.run_test(self)

        #First step: check icon of camera in first orientation
        self._display_api.set_display_orientation(self._first_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("First step: Check camera icons in " + self._first_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        first_verdict, first_msg = self.match_template_camera(self._first_orientation, self._screenshot_path,
                                                                                 self._camera_mode_first_orientation,
                                                                                 self._take_photo_first_orientation,
                                                                                 self._option_camera_first_orientation)

        #Second step: check icon of camera in second orientation
        self._display_api.set_display_orientation(self._second_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Second step: Check camera icons in " + self._second_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        second_verdict, second_msg = self.match_template_camera(self._second_orientation, self._screenshot_path,
                                                                                self._camera_mode_second_orientation,
                                                                                self._take_photo_second_orientation,
                                                                                self._option_camera_second_orientation)

        #Third step: check icon of camera in third orientation
        self._display_api.set_display_orientation(self._third_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Third step: Check camera icons in " + self._third_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        third_verdict, third_msg = self.match_template_camera(self._third_orientation, self._screenshot_path,
                                                                                self._camera_mode_third_orientation,
                                                                                self._take_photo_third_orientation,
                                                                                self._option_camera_third_orientation)

        #Concatenate result
        if first_verdict and second_verdict and third_verdict:
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE
            #Concatenate message
        msg = first_msg + second_msg + third_msg

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)
        #Set the orientation in final orientation
        self._display_api.set_display_orientation(self._final_orientation)

        if self._screenshot_state:
            #Delete screenshot on the host
            self._logger.info("Delete the screenshot on host.")
            os.remove(self._screenshot_path)

        # set initial sleep timeout
        self._phonesystem_api.set_screen_timeout(self._initial_sleep_timeout_value)
        time.sleep(self._wait_btwn_cmd)

        # Force home screen
        self._keyevent_api.back()
        self._keyevent_api.home()

        # lock the screen
        self._phonesystem_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)
        self._keyevent_api.power_button()


        return Global.SUCCESS, "No errors"

    def match_template_camera(self, orientation, screenshot, camera_mode_icon, take_photo_icon, option_camera_icon):
        """
            Check if icon of camera application is correctly display.

            :type orientation: str
            :param orientation: set the screen orientation (portrait, landscape or reverse landscape.
            :type screenshot: str
            :param screenshot: path of the DUT screenshot
            :type camera_mode_icon: str
            :param camera_mode_icon: name of camera_mode_icon image in images library.
            :type take_photo_icon: str
            :param take_photo_icon: name of take_photo_icon image in images library.
            :type option_camera_icon: str
            :param option_camera_icon: name of option_camera_icon image in images library.
            :rtype: tuple
            :return: (str, str) verdict : True if all icon is displayed correctly or false if one or more doesn't
            and msg : Output message
            """

        msg = ""
        verdict = True

        self._logger.info("Match the template in screenshot.")
        #Ckeck camera mode icon
        self._logger.debug("Check camera mode icon in " + orientation + " orientation.")
        template = self._dic_image[camera_mode_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match camera mode icon in " + orientation + " orientation, "

        #Ckeck take photo icon
        self._logger.debug("Check take photo icon  in " + orientation + " orientation.")
        template = self._dic_image[take_photo_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match take photo icon in " + orientation + " orientation, "

        #Ckeck camera option icon
        self._logger.debug("Check camera option icon in " + orientation + " orientation.")
        template = self._dic_image[option_camera_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match camera option icon in " + orientation + " orientation, "

        #Concatenate result
        if verdict:
            msg = "No error in " + orientation + " orientation, "

        return verdict, msg

