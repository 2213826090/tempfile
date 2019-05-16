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
:summary: This module implements check of icon in google camera application
:since: 29/09/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import str_to_dict
import os
import time


class LiveCheckIconGoogleCamera(UseCaseBase):
    """
    Class check icon in video application.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get ICON_DICT from TC param
        self.__icon_dict_mode = str_to_dict(self._tc_parameters.get_param_value("ICON_DICT_MODE"))
        self.__icon_dict_option = str_to_dict(self._tc_parameters.get_param_value("ICON_DICT_OPTION"))
        self.__hardware = self._tc_parameters.get_param_value("HARDWARE").lower()

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._system_api = self._device.get_uecmd("System")
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

        # Open google camera
        self._multimedia_api.open_google_camera()
        time.sleep(self._wait_btwn_cmd)

        # Close first page
        self._logger.debug("Close first Google Photos activity page.")
        self._keyevent_api.scenario(["move_home", "dpad_down", "dpad_down", "enter"])
        time.sleep(self._wait_btwn_cmd)

        return self._error.Code, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function.
        UseCaseBase.run_test(self)

        msg = ""

        # Open camera mode
        self._logger.info("Open camera mode")
        self._keyevent_api.scenario(["menu"])
        time.sleep(self._wait_btwn_cmd)

        for icon in self.__icon_dict_mode:
            verdict = self._multimedia_api.check_screen(self.__icon_dict_mode[icon], "text")
            time.sleep(self._wait_btwn_cmd)

            if not verdict:
                msg += "Doesn't match " + str(icon) + " in UI for camera mode,"

        # Close camera mode
        self._logger.info("Close camera mode")
        self._keyevent_api.scenario(["menu"])
        time.sleep(self._wait_btwn_cmd)

        screen_resolution = self._phonesystem_api.get_screen_resolution().split("x")
        if self.__hardware == "phone":
            self._keyevent_api.tap_on_screen(int(int(screen_resolution[0]) * 0.91),
                                             int(int(screen_resolution[1]) * 0.73))
            time.sleep(self._wait_btwn_cmd)
        elif self.__hardware == "tablet":
            self._keyevent_api.tap_on_screen(int(int(screen_resolution[0]) * 0.83),
                                             int(int(screen_resolution[1]) * 0.05))
            time.sleep(self._wait_btwn_cmd)

        for icon in self.__icon_dict_option:
            verdict = self._multimedia_api.check_screen(self.__icon_dict_option[icon], "id")
            time.sleep(self._wait_btwn_cmd)

            if not verdict:
                msg += "Doesn't match " + str(icon) + " in UI for camera option,"

        if len(msg) > 0:
            verdict = Global.FAILURE
        else:
            verdict = Global.SUCCESS
            msg = "No errors"

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)

        # set initial sleep timeout
        self._phonesystem_api.set_screen_timeout(self._initial_sleep_timeout_value)
        time.sleep(self._wait_btwn_cmd)

        # Force home screen
        self._keyevent_api.back()
        self._keyevent_api.back()
        self._keyevent_api.back()
        self._keyevent_api.home()

        # lock the screen
        self._phonesystem_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)
        self._keyevent_api.power_button()

        return Global.SUCCESS, "No errors"
