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

:organization: INTEL AMPS
:summary: This file implements use case to use camera for taking a picture setting flash on
:since: 23/01/2013
:author: mmorchex
"""

import os
import time
import datetime
import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraFlash(UseCaseBase):

    """
    Class Live Camera Flash On Off.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Init camera name : BACK
        self.__camera = "BACK"

        # get FLASH_MODE from TC parameters
        self.__camera_flash_mode = self._tc_parameters.get_param_value("FLASH_MODE")

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), "")

        self.__local_ref_picture_file = None
        self.__local_picture_file_with_flash = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Open camera
        self.__camera_api.launch_camera(self.__camera)
        time.sleep(self._wait_btwn_cmd)

        # Check save directory
        if self.__save_directory is None:
            error_msg = "Save directory is not set, please update the TC"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # check camera flash mode
        if self.__camera_flash_mode is None:
            error_msg = "Flash mode is not set, please update the TC"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # take picture without flash
        ref_picture_file = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # download picture taken from the DUT
        self.__local_ref_picture_file = self.__camera_api.download_media_file(ref_picture_file, self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        msg = "No errors"

        # set camera flash mode
        self.__camera_api.set_flash_mode(self.__camera_flash_mode)
        time.sleep(self._wait_btwn_cmd)

        # take picture with flash
        picture_file_with_flash = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # download picture taken from the DUT
        self.__local_picture_file_with_flash = self.__camera_api.download_media_file(picture_file_with_flash,
                                                                                     self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        # Check if the picture with flash's brightness is higher than the ref picture's one
        result = Imagecheck.compare_images_brightness(self.__local_ref_picture_file,
                                                      self.__local_picture_file_with_flash)
        if not result:
            msg = "Flash was not fired"
            verdict = Global.FAILURE

        return (verdict, msg)

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Release camera
        self.__camera_api.shutdown()
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")
