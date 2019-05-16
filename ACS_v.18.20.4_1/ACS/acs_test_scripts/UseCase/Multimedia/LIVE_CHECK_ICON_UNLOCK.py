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
:summary: This module implements check icon unlock DUT test.
:since: 06/02/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import os
import time


class LiveCheckIconUnlock(UseCaseBase):
    """
    Class check icon in unlock application
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
        self._swipe_timeout = 500

        #Read the path of image library from testcase
        self._library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        #Read value of different image from testcase
        self._unlock_icon = self._tc_parameters.get_param_value("UNLOCK_ICON")

        #Read value of force_orientation form testcase
        self._force_orientation = self._tc_parameters.get_param_value("FORCE_ORIENTATION")

        #Image library
        self._dic_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._system_api = self._device.get_uecmd("System")
        self._display_api = self._device.get_uecmd("Display")



    def set_up(self):
        """
        set up
        """
        UseCaseBase.set_up(self)

        # wake the screen
        self._phonesystem_api.display_on()
        time.sleep(self._wait_btwn_cmd)
        #Check if DUT is lock
        if not self._system_api.check_Activity("Keyguard"):
            #try to lock the phone
            self._logger.info("Device is unlock, trying to lock.")
            self._phonesystem_api.set_phone_lock(1)
            time.sleep(self._wait_btwn_cmd)
            self._keyevent_api.power_button()
            # wake the screen
            self._phonesystem_api.display_on()
            time.sleep(self._wait_btwn_cmd)


        return self._error.Code, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function.
        UseCaseBase.run_test(self)

        #set display orientation
        self._logger.info("Force screen orientation.")
        self._display_api.set_display_orientation(self._force_orientation)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Check if unlock icon is correctly display.")
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        template = self._dic_image[self._unlock_icon]

        matching, x, y = Imagecheck.match_template_in_image(self._screenshot_path, template)
        if matching:
            #try to unlock phone with swipe
            self._logger.debug("Trying to unlock the phone with swipe.")
            self._phonesystem_api.unlock_phone_with_swipe(x,y,self._swipe_timeout)
            #Now verify if DUT is unlock
            if not self._system_api.check_Activity("Keyguard"):
                verdict = Global.SUCCESS
                msg = "No errors"
            else:
                verdict = Global.FAILURE
                msg = "Device is always lock, unlock fail."
        else:
            verdict = Global.FAILURE
            msg = "Doesn't match unlock icon."

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

        #set display orientation in auto
        self._logger.info("Force auto screen orientation.")
        self._display_api.set_display_orientation("auto")
        time.sleep(self._wait_btwn_cmd)

        # lock the screen
        self._phonesystem_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)
        self._keyevent_api.power_button()


        return Global.SUCCESS, "No errors"
