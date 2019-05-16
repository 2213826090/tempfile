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
:summary: This file implements use case to use camera for face detection
:since: 13/03/2014
:author: mmorchex
"""

import os
import time
import Lib.ImageCheck.Imagecheck as Imagecheck
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class LiveCameraFaceDetection(UseCaseBase):

    """
    Class Live Camera Face Detection.
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
        self.__image_api = self._device.get_uecmd("Image")
        self.__phone_system_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # wake the screen
        self._logger.info("Turn on the display")
        self.__phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)

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

        # Start face detection on DUT
        dut_faces = self.__camera_api.start_face_detection()
        time.sleep(self._wait_btwn_cmd)

        # Take a screen shot for what is displayed by the DUT
        screen_shot = self.__image_api.take_screenshot_and_pull_on_host("screen_shot", os.getcwd())
        time.sleep(self._wait_btwn_cmd)

        # Check the number of faces detected in the screen shot taken
        host_faces = Imagecheck.face_detection(screen_shot)

        # Remove Screen shot taken
        os.remove(screen_shot)

        if host_faces != dut_faces:
            msg = "%s face(s) checked by the Camera, %s face(s) checked by host" % (str(dut_faces), str(host_faces))
            verdict = Global.FAILURE

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
