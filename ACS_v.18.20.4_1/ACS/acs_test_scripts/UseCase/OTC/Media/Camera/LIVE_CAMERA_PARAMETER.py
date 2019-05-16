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

:organization: INTEL OTC
:summary: This file implements use case to use camera to take a picture with a certain setting.
:since: 13/05/2014
:author: agdobrex
"""

import os
import time
import datetime
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException



class LiveCameraParameter(UseCaseBase):

    """
    Class that check if the DUT can correctly detect change of camera parameters.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # Get UECmdLayer
        self.__camera_api = self._device.get_uecmd("Camera")


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

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        msg = "No errors"

        before = self.__camera_api.get_camera_current_parameters()

        if before['picture-size'] == '640x480':
            self.__camera_api.set_picture_size('1920', '1080')
        else:
            self.__camera_api.set_picture_size('640', '480')

        time.sleep(self._wait_btwn_cmd)

        after = self.__camera_api.get_camera_current_parameters()

        if before['picture-size'] == after['picture-size']:
            verdict = Global.FAILURE
            msg = "Preview size has not changed. Before: %s, after %s." % (before['picture-size'],
                                                                          after['picture-size'])

        return verdict, msg

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

        return Global.SUCCESS, "No errors"
