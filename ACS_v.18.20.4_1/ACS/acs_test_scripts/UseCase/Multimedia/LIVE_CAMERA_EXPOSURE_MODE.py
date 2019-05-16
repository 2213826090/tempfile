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
:summary: This file implements use case to take picture after updating exposure mode
:since: 22/01/2014
:author: mmorchex
"""

import os
import time
import datetime
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraExposureMode(UseCaseBase):

    """
    Class Live Camera exposure mode.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get HIGHER_EXPOSURE_VALUE from TC parameters
        self.__higher_exposure_value = self._tc_parameters.get_param_value("HIGHER_EXPOSURE_VALUE", \
                                                                           default_cast_type=int)

        # get LOWER_EXPOSURE_VALUE from TC parameters
        self.__lower_exposure_value = self._tc_parameters.get_param_value("LOWER_EXPOSURE_VALUE", \
                                                                          default_cast_type=int)

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'), "")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check exposure mode values
        if self.__higher_exposure_value < self.__lower_exposure_value:
            error_msg = "Exposure higher value should be > Exposure lower value, please update values in the TestCase"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check save directory
        if self.__save_directory in (None, ""):
            error_msg = "Please update TC and set a saving directory"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Open camera
        self.__camera_api.launch_camera(self.__camera)
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
        msg = ""
        pictures_corrupted = 0

        # Take picture with default exposure mode value : exposure = 0
        default_exposure_pictures_uri = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Download picture taken from the DUT
        local_default_exposure_pictures_uri = self.__camera_api.download_media_file(default_exposure_pictures_uri,\
                                                                                    self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        # Take picture with the higher exposure value
        # Set exposure mode to the higher value
        self.__camera_api.set_exposure_mode(self.__higher_exposure_value)
        time.sleep(self._wait_btwn_cmd)

        # Take picture
        high_exposure_value_picture_uri = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Download picture taken from the DUT
        local_high_exposure_value_picture_uri = self.__camera_api.download_media_file(high_exposure_value_picture_uri, \
                                                                                      self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

       # Take picture with the lower exposure value
       # Set exposure mode to the lower value
        self.__camera_api.set_exposure_mode(self.__lower_exposure_value)
        time.sleep(self._wait_btwn_cmd)

        # Take picture
        low_exposure_value_picture_uri = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Download picture taken from the DUT
        local_low_exposure_value_picture_uri = self.__camera_api.download_media_file(low_exposure_value_picture_uri, \
                                                                                      self.__save_folder)

        # Set default exposure value
        self.__camera_api.set_exposure_mode(0)
        time.sleep(self._wait_btwn_cmd)

        # Take picture with default exposure value : exposure = 0
        second_default_exposure_pictures_uri = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Download picture taken from DUT
        local_second_default_exposure_pictures_uri = self.__camera_api.\
            download_media_file(second_default_exposure_pictures_uri, self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        # Check pictures taken are not corrupted
        for picture in (local_default_exposure_pictures_uri, local_high_exposure_value_picture_uri, \
                        local_low_exposure_value_picture_uri, local_second_default_exposure_pictures_uri):
            if Imagecheck.check_image_corrupt(picture):
                pictures_corrupted += 1

        if pictures_corrupted == 0:

            # Check that exposure mode was correctly set to higher value set in TC
            result = Imagecheck.compare_images_brightness(local_default_exposure_pictures_uri, \
                                                          local_high_exposure_value_picture_uri)
            if not result:
                verdict = Global.FAILURE
                msg = "Exposure value was not set to %s ," % str(self.__higher_exposure_value)

            # Check that exposure mode was correctly set to lower value set in TC
            result = Imagecheck.compare_images_brightness(local_low_exposure_value_picture_uri, \
                                                          local_high_exposure_value_picture_uri)
            if not result:
                verdict = Global.FAILURE
                msg += "Exposure value was not set to %s ," % str(self.__lower_exposure_value)

            # Check that exposure mode was correctly set to default value
            result = Imagecheck.compare_images_brightness(local_low_exposure_value_picture_uri, \
                                                          local_second_default_exposure_pictures_uri)

            if not result:
                verdict = Global.FAILURE
                msg += "Exposure value was not set to default : 0"

            if msg == "":
                msg = "No errors"

        else:
            verdict = Global.FAILURE
            msg = " %s picture(s) are/is corrupted " % str(pictures_corrupted)

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
