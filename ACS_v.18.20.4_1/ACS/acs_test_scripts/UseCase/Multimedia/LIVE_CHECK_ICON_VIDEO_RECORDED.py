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
:summary: This module implements check of icon in video application with video recorded
:since: 21/01/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import os
import time


class LiveCheckIconVideoRecorded(UseCaseBase):
    """
    Class check icon in video application.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._image_filename = "DUTScreenshot"
        self._screenshot_path = ""
        self._mediaStoreName = "external"
        self._screenshot_state = False
        self._camera_released = False
        self._camera_state = False
        self._save_directory = "/storage/emulated/0/"

        #Read the path of image library from testcase
        self._library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        #Read value of different image from testcase
        self._album_icon = self._tc_parameters.get_param_value("ALBUM_VIDEO")
        self._pause_icon = self._tc_parameters.get_param_value("PAUSE_VIDEO")
        self._share_icon = self._tc_parameters.get_param_value("SHARE_VIDEO")

        #Read value of video orientation from testcase
        self._video_orientation = self._tc_parameters.get_param_value("VIDEO_ORIENTATION")

        #Image library
        self._dic_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._multimedia_api = self._device.get_uecmd("Multimedia")
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

        #Launch Camera
        self._camera_api.launch_camera("BACK")
        time.sleep(self._wait_btwn_cmd)

        self._camera_state = True

        #Record video
        self._video_recorded = self._camera_api.record_video(10000, self._save_directory)
        time.sleep(self._wait_btwn_cmd)

        #Shutdown camera
        self._camera_api.shutdown()
        time.sleep(self._wait_btwn_cmd)

        self._camera_released = True
        self._camera_state = False

        #Launch media scanner
        self._camera_api.launch_media_scanner(self._save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Get the video from the real video path
        video_id = self._multimedia_api.get_video_id_from_path(self._video_recorded, self._mediaStoreName)
        time.sleep(self._wait_btwn_cmd)

        #Launch video from is ID
        self._multimedia_api.play_video_from_ID(video_id, self._video_orientation)
        time.sleep(self._wait_btwn_cmd)

        #Pause the video
        self._keyevent_api.media_play_pause()
        time.sleep(self._wait_btwn_cmd)

        return self._error.Code, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function.
        UseCaseBase.run_test(self)

        self._logger.info("Check video icons in pause.")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        verdict, msg = self.match_template_video(self._screenshot_path, self._album_icon, self._pause_icon,
                                                 self._share_icon)

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)

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

        if not self._camera_released and self._camera_state:
            # Release camera
            self._camera_api.shutdown()
            time.sleep(self._wait_btwn_cmd)

        # lock the screen
        self._phonesystem_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)
        self._keyevent_api.power_button()


        return Global.SUCCESS, "No errors"

    def match_template_video(self, screenshot, album_icon, pause_icon, share_icon):
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
        verdict = Global.SUCCESS

        self._logger.info("Match the template in screenshot.")
        #Ckeck album icon
        self._logger.debug("Check album icon in video pause.")
        template = self._dic_image[album_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = Global.FAILURE
            msg = msg + "Doesn't match album icon, "

        #Ckeck pause icon
        self._logger.debug("Check pause icon  in video pause.")
        template = self._dic_image[pause_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = Global.FAILURE
            msg = msg + "Doesn't match pause icon, "

        #Ckeck share icon
        self._logger.debug("Check share icon in video pause.")
        template = self._dic_image[share_icon]
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if not matching:
            verdict = Global.FAILURE
            msg = msg + "Doesn't match share icon. "

        #Concatenate result
        if verdict == Global.SUCCESS:
            msg = "No errors"

        return verdict, msg

