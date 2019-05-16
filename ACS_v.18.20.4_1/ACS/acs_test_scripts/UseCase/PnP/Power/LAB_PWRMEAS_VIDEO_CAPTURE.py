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
:summary: This file implements Power measurement UC for Video Capture
:author: ssavrimoutou
:since:28/10/2011
"""

import time
from LAB_PWRMEAS_BASE import LabPwrMeasBase
from UtilitiesFWK.Utilities import Global


class LabPwrMeasVideoCapture(LabPwrMeasBase):

    """
    Class Lab Power measurement Video Capture
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        LabPwrMeasBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._video_save_path = self._tc_parameters.get_param_value("VIDEO_PATH")
        self._camera_name = self._tc_parameters.get_param_value("CAMERA_NAME")
        self._camera = self._tc_parameters.get_param_value("CAMERA")
        self._quality = self._tc_parameters.get_param_value("QUALITY")
        self._flash_mode = self._tc_parameters.get_param_value("FLASH_MODE")
        self._color_effect = self._tc_parameters.get_param_value("COLOR_EFFECT")
        self._white_balance = self._tc_parameters.get_param_value("WHITE_BALANCE")
        self._dvs = self._tc_parameters.get_param_value("DVS")
        self._noise_reduction = self._tc_parameters.get_param_value("NOISE_REDUCTION")

        self._video_filename = None

        # Get UECmdLayer
        self._video_record_api = self._device.get_uecmd("VideoRecorder")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LabPwrMeasBase base Setup function
        LabPwrMeasBase.set_up(self)

        # Lock the phone display to ON
        self._phonesystem_api.display_on()

        # Kill camera package to make next upload video profile can work
        self._video_record_api.kill_camera(self._camera_name)
        time.sleep(self._wait_btwn_cmd)

        # Starting the video recording
        self._video_filename = self._video_record_api.native_video_record(
            self._video_save_path,
            self._camera_name,
            self._camera,
            self._quality,
            self._flash_mode,
            self._color_effect,
            self._white_balance,
            self._dvs,
            self._noise_reduction)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _run_test_end(self):
        """
        End test actions
        """
        self._logger.debug("End test actions begin")
        # Checking that video still recording
        self._video_record_api.check_recording()
        time.sleep(self._wait_btwn_cmd)
        self._logger.debug("Video filename: %s" % str(self._video_filename))

        if self._video_filename is not None:
            # Check if the video file exist
            self._logger.debug("Stop video record")
            self._video_record_api.stop_video_record(self._video_save_path, self._video_filename)
            # Check if the video file exist
            self._phonesystem_api.check_file_exist_from_shell(self._video_filename)

        self._logger.debug("End test actions end")

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call Lab PwrMeas base tear down function
        LabPwrMeasBase.tear_down(self)

        #Stop video record as prevention mechanism
        self._logger.debug("Stop video record (prevention mechanism in teardown)")
        self._video_record_api.stop_video_record(self._video_save_path, self._video_filename)

        # Release the phone display
        self._phonesystem_api.display_off()

        return Global.SUCCESS, "No errors"
