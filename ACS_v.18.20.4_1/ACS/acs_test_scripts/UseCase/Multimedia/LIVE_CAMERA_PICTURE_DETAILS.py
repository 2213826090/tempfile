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

:organization: INTEL MCG PSI
:summary: This module implements checking picture details after setting camera flash mode, color effect
and white balance.
:since: 02/01/14
:author: mmorchex
"""

import os
import time
import datetime
import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraPictureDetails(UseCaseBase):

    """
    Class Live camera picture details.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get PICTURE_SIZE from TC parameters
        self.__picture_size = self._tc_parameters.get_param_value("PICTURE_SIZE")

        # get WHITE_BALANCE from TC parameters
        self.__camera_white_balance = self._tc_parameters.get_param_value("WHITE_BALANCE")

        # get FLASH_MODE from TC parameters
        self.__camera_flash_mode = self._tc_parameters.get_param_value("FLASH_MODE")

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), "")

        self.__picture_file = None

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

        # set Picture Size
        if self.__picture_size:
            self.__picture_size = self.__picture_size.upper()
            if not Imagecheck.check_picture_resolution(self.__camera, self.__picture_size):
                error_msg = "Invalid resolution %s for camera %s" % (self.__picture_size, self.__camera)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

            (width, height) = Imagecheck.get_size_from_resolution(self.__camera, self.__picture_size)
            self.__camera_api.set_picture_size(width, height)
            time.sleep(self._wait_btwn_cmd)

        # set camera flash mode
        if self.__camera_flash_mode:
            self.__camera_api.set_flash_mode(self.__camera_flash_mode)
            time.sleep(self._wait_btwn_cmd)

        # set camera white balance mode
        if self.__camera_white_balance:
            self.__camera_api.set_white_balance(self.__camera_white_balance)
            time.sleep(self._wait_btwn_cmd)

        # take picture
        picture_file = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # download picture taken from the DUT
        self.__picture_file = self.__camera_api.download_media_file(picture_file, self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        result_msg = ""
        result = Global.SUCCESS

        # Check Picture size on picture's EXIF data
        if self.__picture_size:
            check_result = Imagecheck.check_picture_size(self.__camera, self.__picture_file, self.__picture_size)
            if not check_result:
                result_msg = "Picture Size was not correctly set,"
                result = Global.FAILURE

        # Check Flash on picture's EXIF data
        if self.__camera_flash_mode:
            check_result = Imagecheck.get_picture_exif_data(self.__picture_file, "Flash")
            if check_result == 0:
                result_msg += " Flash mode was not correctly set in picture details,"
                result = Global.FAILURE

        # Check white balance on picture's EXIF data
        if self.__camera_white_balance:
            check_result = Imagecheck.get_picture_exif_data(self.__picture_file, "WhiteBalance")
            if self.__camera_white_balance.lower() == "auto":
                if check_result == 1:
                    result_msg += " White balance was not correctly set in picture details,"
                    result = Global.FAILURE
            else:
                if check_result == 0:
                    result_msg += " White balance was not correctly set in picture details,"
                    result = Global.FAILURE

        if result_msg == "":
            result_msg = "No errors"

        return (result, result_msg)

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
