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

:organization: INTEL AMPS
:summary: This file implements use case to use camera for taking a picture after setting scene mode
:since: 02/01/2014
:author: mmorchex
"""

import os
import time
import datetime
import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveCameraSceneMode(UseCaseBase):

    """
    Class Live image capture.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # get CAMERA name from TC parameters
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        # get SCENE_MODE from TC parameters
        self.__camera_scene_mode = self._tc_parameters.get_param_value("SCENE_MODE")

        # get SAVE_DIRECTORY from TC parameters
        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        # Get UECmdLayer.
        self.__camera_api = self._device.get_uecmd("Camera")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'), "")

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

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        local_ref_picture_file = None
        ref_picture = False

        verdict = Global.SUCCESS
        msg = "No errors"

        # Check Scene mode
        if self.__camera_scene_mode is None:
            error_msg = "Scene mode is not set, please update the TC"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        for iter in Imagecheck.treatment_methods:
            if iter['effect'] == self.__camera_scene_mode:
                if iter['attributes'] > 1:
                    ref_picture = True

        if ref_picture:
            # take picture without setting scene mode
            ref_picture_file = self.__camera_api.take_picture(self.__save_directory)
            time.sleep(self._wait_btwn_cmd)

            # download picture taken from the DUT
            local_ref_picture_file = self.__camera_api.download_media_file(ref_picture_file, self.__save_folder)
            time.sleep(self._wait_btwn_cmd)

        # set scene mode
        self.__camera_api.set_scene_mode(self.__camera_scene_mode)
        time.sleep(self._wait_btwn_cmd)

        # take picture with scene mode
        picture_file_with_scene_mode = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # download picture taken from the DUT
        local_picture_file_with_scene_mode = self.__camera_api.download_media_file(picture_file_with_scene_mode,
                                                                                    self.__save_folder)
        time.sleep(self._wait_btwn_cmd)

        # Check if scene mode has been set
        for iter in Imagecheck.treatment_methods:
            if iter['effect'] == self.__camera_scene_mode:
                result = iter['method'](local_ref_picture_file, local_picture_file_with_scene_mode)
                if not result:
                    msg = "%s mode was not set" % self.__camera_scene_mode
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
