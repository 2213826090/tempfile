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
:summary: This file implements use case to take picture in burst mode
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


class LiveCameraBurstMode(UseCaseBase):

    """
    Class Live Camera burst mode
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get PICTURES_NUMBER from TC parameters
        self.__pictures_number = self._tc_parameters.get_param_value("PICTURES_NUMBER", default_cast_type=int)

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

        # Check if the number of pictures to take is correct : > 1
        if not self.__pictures_number > 1:
            error_msg = "Number of pictures to take should be > 1"
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
        msg = "No errors"

       # Take pictures in burst mode
        pictures_uri = self.__camera_api.take_picture(self.__save_directory, self.__pictures_number)
        time.sleep(self._wait_btwn_cmd)

        # Check the number of pictures taken
        if len(pictures_uri.split(",")) != self.__pictures_number:
            verdict = Global.FAILURE
            msg = "Number of pictures taken is not as expected. expected %s, obtained : %s"  \
                                              % (str(self.__pictures_number), str(len(pictures_uri.split(","))))
            return verdict, msg

        # Download pictures taken from DUT
        local_pictures_uri = []
        for picture_uri in pictures_uri.split(","):
            local_pictures_uri.append(self.__camera_api.download_media_file(picture_uri, self.__save_folder))

        # Check if pictures are corrupted
        corrupted = False
        pictures_corrupted = 0

        for picture in local_pictures_uri:
            if Imagecheck.check_image_corrupt(picture):
                corrupted = True
                pictures_corrupted += 1

        if corrupted:
            verdict = Global.FAILURE
            msg = " %s pictures are corrupted" % str(pictures_corrupted)

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
