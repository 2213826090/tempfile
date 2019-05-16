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
:summary: This file implements use case to use camera for taking a picture after setting a color effect
:since: 16/01/2014
:author: mmorchex
"""

import os
import time
import datetime
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException



class LiveCameraColorEffect(UseCaseBase):

    """
    Class Live camera color effect.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get COLOR_EFFECT from TC parameters
        self.__camera_color_effect = self._tc_parameters.get_param_value("COLOR_EFFECT")

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), "")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check color effect
        if self.__camera is None:
            error_msg = "Camera is not set, please update the TC"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Open camera
        self.__camera_api.launch_camera(self.__camera)
        time.sleep(self._wait_btwn_cmd)

        # set camera flash mode to torch
        self.__camera_api.set_flash_mode("torch")
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

        local_ref_picture_file = None
        ref_picture = False

        # Check color effect
        if self.__camera_color_effect is None:
            error_msg = "Color effect is not set, please update the TC"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        for iter in Imagecheck.treatment_methods:
            if iter['effect'] == self.__camera_color_effect:
                if iter['attributes'] > 1:
                    ref_picture = True

        if ref_picture:
            # take picture without color effect
            ref_picture_file = self.__camera_api.take_picture(self.__save_directory)
            time.sleep(self._wait_btwn_cmd)

            # download picture taken from the DUT
            local_ref_picture_file = self.__camera_api.download_media_file(ref_picture_file, self.__save_folder)
            time.sleep(self._wait_btwn_cmd)

        # set color effect
        self.__camera_api.set_color_effect(self.__camera_color_effect)
        time.sleep(self._wait_btwn_cmd)

        # take picture with color effect
        picture_file_with_color_effect = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # download picture taken from the DUT
        local_picture_file_with_color_effect = self.__camera_api.download_media_file(picture_file_with_color_effect,
                                                                                     self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        # Check if color effect has been set
        for iter in Imagecheck.treatment_methods:
            if iter['effect'] == self.__camera_color_effect:
                result = iter['method'](local_picture_file_with_color_effect, local_ref_picture_file)
                if not result:
                    msg = "%s was not set" % self.__camera_color_effect
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
