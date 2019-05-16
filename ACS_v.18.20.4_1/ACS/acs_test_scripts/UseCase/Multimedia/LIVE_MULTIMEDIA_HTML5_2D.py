"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This module implements Live Multimedia Html5 2D test.
@since: 01/04/14
@author: jcoutox
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
import Lib.VideoCheck.VideoCheck as VideoCheck
import datetime
import os
import time
import shutil


class LiveMultimediaHtml52d(UseCaseBase):
    """
    Class Live Multimedia Html5 2D.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        #Set variable
        self._webpage_loading_timeout = 30
        self._box_crop_bottom = 360

        # Get TC Parameters
        self._local_website_url = self._tc_parameters.get_param_value("LOCAL_WEBSITE_URL")
        self._browser_type = self._tc_parameters.get_param_value("BROWSER_TYPE").lower()
        self._video_reference = self._tc_parameters.get_param_value("VIDEO_REFERENCE")
        self._wait_before_record = int(self._tc_parameters.get_param_value("WAIT_BEFORE_RECORD"))
        self._disp_brightness = self._tc_parameters.get_param_value("DISP_BRIGHTNESS")
        self._screenrecord_bit_rate = int(self._tc_parameters.get_param_value("BIT_RATE"))
        self._screenrecord_time_limit = int(self._tc_parameters.get_param_value("TIME_LIMIT"))
        self._save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")
        self._save_directory_host = self._tc_parameters.get_param_value("SAVE_DIRECTORY_HOST")

        #Create save folder name.
        self._save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), "")

        #Create screenrecord name.
        screenrecord_name = self._local_website_url.split("/")
        screenrecord_name = screenrecord_name[-1].split(".")
        self._screenrecord_name = screenrecord_name[-2] + "_screenrecord_" + \
                                  datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
        # Get UECmdLayer
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._camera_api = self._device.get_uecmd("Camera")
        self._system_api = self._device.get_uecmd("System")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")

    #------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        #Get screen resolution for determine box crop
        screen_resolution = self._phone_system_api.get_screen_resolution().split("x")
        #Check if one parameter of screen resolution lower than 1000.
        if int(screen_resolution[0]) < 1000 or int(screen_resolution[1]) < 1000:
            self._box_crop_top = 160
        else:
            self._box_crop_top = 300

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

        # Open the browser and load the url before timeout
        (self._error.Code, self._error.Msg) = self._networking_api.open_web_browser(self._local_website_url,
                                                                                    self._browser_type,
                                                                                    self._webpage_loading_timeout)
        time.sleep(self._wait_btwn_cmd)

        if self._browser_type == "CHROME" or self._browser_type == "chrome":
            #Check if first chrome utilisation is displayed.
            if self._system_api.check_Activity("FirstRunExperienceActivityPhone"):
                #Close first page
                self._logger.debug("Close first Chrome First Run Experience Phone page.")
                self._keyevent_api.scenario(["move_home", "tab", "tab", "enter"])
                time.sleep(self._wait_btwn_cmd)
                #Check if second page of first chrome utilisation is displayed.
                if self._system_api.check_Activity("FirstRunExperienceActivityPhone"):
                #Close second page
                    self._logger.debug("Close second Chrome First Run Experience Phone page.")
                    self._keyevent_api.scenario(["move_home", "tab", "tab", "tab", "enter"])
                    time.sleep(self._wait_btwn_cmd)
            elif self._system_api.check_Activity("FirstRunExperienceActivityTablet"):
            #Close first page for tablet.
                self._logger.debug("Close Chrome First Run Experience Tablet page.")
                self._keyevent_api.scenario(["move_home", "tab", "enter"])
                time.sleep(self._wait_btwn_cmd)

            self._networking_api.close_web_browser(self._browser_type.lower())

            # Open the browser and load the url before timeout
            (self._error.Code, self._error.Msg) = self._networking_api.open_web_browser(self._local_website_url,
                                                                                        self._browser_type,
                                                                                        self._webpage_loading_timeout)

        return (Global.SUCCESS, "No errors")

    #------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        #Wait before launch the screenrecord.
        time.sleep(self._wait_before_record)

        #Take screenrecord
        self._multimedia_api.take_screenrecord(self._screenrecord_name, save_directory=self._save_directory,
                                               bit_rate=self._screenrecord_bit_rate,
                                               time_limit=self._screenrecord_time_limit)
        #Wait during screenrecord time limit
        time.sleep(self._screenrecord_time_limit + self._wait_btwn_cmd)

        #Pull screenrecord in report file
        local_video_file = self._camera_api.download_media_file(self._save_directory + self._screenrecord_name,
                                                                self._save_folder)

        #Compare screenrecord with reference video
        self._logger.info("Compare recorded and reference file.")
        verdict_bool, msg = VideoCheck.compare_two_video_frame_by_frame(local_video_file, self._video_reference,
                                                                        self._screenrecord_time_limit, crop=True,
                                                                        box_crop_top=self._box_crop_top,
                                                                        box_crop_bottom=self._box_crop_bottom,
                                                                        percent_threshold=0.80,
                                                                        match_template_threshold=0.80)

        #Concatenate result
        if verdict_bool:
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        return (verdict, msg)

    #------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Close the browser
        self._networking_api.close_web_browser(self._browser_type)
        time.sleep(self._wait_btwn_cmd)

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

        return (Global.SUCCESS, "No errors")
