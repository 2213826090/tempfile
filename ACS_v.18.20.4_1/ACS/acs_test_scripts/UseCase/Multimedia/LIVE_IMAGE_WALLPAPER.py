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
:summary: This file is the process of LiveImageReopen
:since: 12/24/2011
:author: szhen11
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveImageWallpaper(UseCaseBase):

    """
    Class set wallpaper, then restore system wallpaper.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._multimedia_path = self._device.multimedia_path
        self._wp_type = \
            self._tc_parameters.get_param_value("WP_TYPE")
        self._wp_name = \
            self._tc_parameters.get_param_value("WP_NAME")

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

    def set_up(self):
        """
        set up
        """
        UseCaseBase.set_up(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Wake up the screen...")
        self._phone_system_api.wake_screen()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Turn on the display")
        self._phone_system_api.display_on()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Unlock the screen...")
        self._phone_system_api.set_phone_lock(0)

        return self._error.Code, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        if self._wp_type in "static":
            self._wp_name = self._multimedia_path + self._wp_name

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Trying to set wallpaper...")
        self._image_api.set_wallpaper(self._wp_type, self._wp_name)

        return Global.SUCCESS, "No error"

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Trying to restore original wallpaper...")
        self._image_api.restore_wallpaper()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Turn off the display")
        self._phone_system_api.display_off()

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Lock the screen...")
        self._phone_system_api.set_phone_lock(1)

        return self._error.Code, "No errors"
