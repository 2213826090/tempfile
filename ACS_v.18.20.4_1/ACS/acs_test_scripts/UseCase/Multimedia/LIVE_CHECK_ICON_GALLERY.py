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
:summary: This module implements check of icon in galley application
:since: 21/01/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import os
import time


class LiveCheckIconGallery(UseCaseBase):
    """
    Class check icon in gallery application
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
        self._album_icon_first_orientation = self._tc_parameters.get_param_value("ALBUM_ICON_FIRST_ORIENTATION")
        self._album_text_first_orientation = self._tc_parameters.get_param_value("ALBUM_TEXT_FIRST_ORIENTATION")
        self._camera_icon_first_orientation = self._tc_parameters.get_param_value("CAMERA_ICON_FIRST_ORIENTATION")
        self._option_icon_first_orientation = self._tc_parameters.get_param_value("OPTION_ICON_FIRST_ORIENTATION")
        self._preview_first_orientation = self._tc_parameters.get_param_value("PREVIEW_FIRST_ORIENTATION")

        #Read value of different image for second orientation from testcase
        self._album_icon_second_orientation = self._tc_parameters.get_param_value("ALBUM_ICON_SECOND_ORIENTATION")
        self._album_text_second_orientation = self._tc_parameters.get_param_value("ALBUM_TEXT_SECOND_ORIENTATION")
        self._camera_icon_second_orientation = self._tc_parameters.get_param_value("CAMERA_ICON_SECOND_ORIENTATION")
        self._option_icon_second_orientation = self._tc_parameters.get_param_value("OPTION_ICON_SECOND_ORIENTATION")
        self._preview_second_orientation = self._tc_parameters.get_param_value("PREVIEW_SECOND_ORIENTATION")

        #Read value of different image for third orientation from testcase
        self._album_icon_third_orientation = self._tc_parameters.get_param_value("ALBUM_ICON_THIRD_ORIENTATION")
        self._album_text_third_orientation = self._tc_parameters.get_param_value("ALBUM_TEXT_THIRD_ORIENTATION")
        self._camera_icon_third_orientation = self._tc_parameters.get_param_value("CAMERA_ICON_THIRD_ORIENTATION")
        self._option_icon_third_orientation = self._tc_parameters.get_param_value("OPTION_ICON_THIRD_ORIENTATION")
        self._preview_third_orientation = self._tc_parameters.get_param_value("PREVIEW_THIRD_ORIENTATION")

        #Image library
        self._dic_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        # Get UECmdLayers
        self._image_api = self._device.get_uecmd("Image")
        self._display_api = self._device.get_uecmd("Display")
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._system_api = self._device.get_uecmd("System")

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

        #Launch app Gallery3d
        self._multimedia_api.launch_system_gallery_application()
        time.sleep(self._wait_btwn_cmd)

        #Check if gallery is correctly open
        if not self._system_api.check_Activity("GalleryActivity"):
            verdict = Global.BLOCKED
            msg = "Gallery application is not open."
        else:
            verdict = Global.SUCCESS
            msg = " No errors"

        #Check if account window is open
        if self._system_api.check_Activity("AccountIntroUIActivity"):
            #Close account popup
            self._keyevent_api.scenario(["move_home", "tab", "tab", "enter"])
            time.sleep(self._wait_btwn_cmd)

        return verdict, msg

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function.
        UseCaseBase.run_test(self)

        #First step: check icon of gallery in first orientation
        self._display_api.set_display_orientation(self._first_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("First step: Check gallery icons in " + self._first_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        verdict_portrait, msg_portrait = self.match_template_gallery(self._first_orientation, self._screenshot_path,
                                                                                 self._album_icon_first_orientation,
                                                                                 self._album_text_first_orientation,
                                                                                 self._camera_icon_first_orientation,
                                                                                 self._option_icon_first_orientation,
                                                                                 self._preview_first_orientation)

        #Second step: check icon of gallery in second orientation
        self._display_api.set_display_orientation(self._second_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Second step: Check gallery icons in " + self._second_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        verdict_landscape, msg_landscape = self.match_template_gallery(self._second_orientation, self._screenshot_path,
                                                                                    self._album_icon_second_orientation,
                                                                                    self._album_text_second_orientation,
                                                                                    self._camera_icon_second_orientation,
                                                                                    self._option_icon_second_orientation,
                                                                                    self._preview_second_orientation)

        #Third step: check icon of gallery in third orientation
        self._display_api.set_display_orientation(self._third_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Third step: Check gallery icons in " + self._third_orientation + " orientation")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        verdict_reverse_landscape, msg_reverse_landscape = self.match_template_gallery(self._third_orientation,
                                                                                    self._screenshot_path,
                                                                                    self._album_icon_third_orientation,
                                                                                    self._album_text_third_orientation,
                                                                                    self._camera_icon_third_orientation,
                                                                                    self._option_icon_third_orientation,
                                                                                    self._preview_third_orientation)

        #Concatenate result
        if verdict_portrait and verdict_landscape and verdict_reverse_landscape:
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE
            #Concatenate message
        msg = msg_portrait + msg_landscape + msg_reverse_landscape

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)
        #Set the orientation in final orientation
        self._display_api.set_display_orientation(self._final_orientation)
        time.sleep(self._wait_btwn_cmd)

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

    def match_template_gallery(self, orientation, screenshot, album_icon, album_text, camera_icon, option_icon,
                               preview):
        """
            Check if icon of gallery application is correctly display.

            :type orientation: str
            :param orientation: set the screen orientation (portrait, landscape or reverse landscape
            :type screenshot: str
            :param screenshot: path of the DUT screenshot
            :type album_icon: str
            :param album_icon: name of album_icon image in images library.
            :type album_text: str
            :param album_text: name of album_text image in images library.
            :type camera_icon: str
            :param camera_icon: name of camera_icon image in images library.
            :type option_icon: str
            :param option_icon: name of option_icon image in images library.
            :type preview: str
            :param preview: name of preview image in images library.
            :rtype: tuple
            :return: (str, str) verdict : True if all icon is displayed correctly or false if one or more doesn't
            and msg : Output message
            """

        msg = ""
        verdict = True

        self._logger.info("Match the template in screenshot.")
        #Ckeck album icon
        self._logger.debug("Check album icon in " + orientation + " orientation.")
        template = self._dic_image[album_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match album icon in " + orientation + " orientation, "

        #Ckeck album text
        self._logger.debug("Check album  in " + orientation + " orientation.")
        template = self._dic_image[album_text]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match album text in " + orientation + " orientation, "

        #Ckeck camera icon
        self._logger.debug("Check camera icon in " + orientation + " orientation.")
        template = self._dic_image[camera_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match camera icon in " + orientation + " orientation, "

        #Ckeck option icon
        self._logger.debug("Check option icon in " + orientation + " orientation.")
        template = self._dic_image[option_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match option icon in " + orientation + " orientation, "

        #Ckeck preview
        self._logger.debug("Check preview image in " + orientation + " orientation.")
        template = self._dic_image[preview]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = False
            msg = msg + "Doesn't match preview image in " + orientation + " orientation, "

        #Concatenate result
        if verdict:
            msg = "No error in " + orientation + " orientation, "

        return verdict, msg
