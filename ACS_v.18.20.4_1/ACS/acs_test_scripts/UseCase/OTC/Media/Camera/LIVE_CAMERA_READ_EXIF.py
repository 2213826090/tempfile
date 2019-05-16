# pylint: disable=invalid-name
"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

:since: 6/19/14
:author: agdobrex
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import time
import os
import datetime

class LiveCameraReadExif(UseCaseBase):

    """
    Class Live Camera Read Exif.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        if self._device.get_config("DisableTcReboot", False, default_cast_type='str_to_bool'):
            self._pre_reboot_device = self._tc_parameters.get_param_value("PRE_REBOOT", default_value="False",
                                                                          default_cast_type="str_to_bool")
            self._post_reboot_device = self._tc_parameters.get_param_value("POST_REBOOT", default_value="False",
                                                                           default_cast_type="str_to_bool")
            self._post_reboot_nok_device = self._tc_parameters.get_param_value("POST_REBOOT_NOK", default_value="False",
                                                                               default_cast_type="str_to_bool")
        self.__camera = self._tc_parameters.get_param_value("CAMERA")

        self.__save_directory = self._tc_parameters.get_param_value("SAVE_DIRECTORY")

        self.__exif_parameter_to_check = self._tc_parameters.get_param_value("EXIF_PARAMETER")

        self.__exif_parameter_value = self._tc_parameters.get_param_value("EXIF_VALUE")

        self.__save_folder = os.path.join(self._name + "-" + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'), "")

        # get UECmd layer
        self.__camera_api = self._device.get_uecmd("Camera")

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check save directory
        if self.__save_directory in (None, ""):
            error_msg = "Please update TC and set a saving directory"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Open camera
        self.__camera_api.launch_camera(self.__camera)
        time.sleep(self._wait_btwn_cmd)

        try:
            #Set exposure mode
            self.__camera_api.set_exposure_mode(0)
            time.sleep(self._wait_btwn_cmd)
        except AcsConfigException:
            self._logger.warning("UseCase tried to set camera parameter exposure mode on a device that does not "
                                 "support this parameter.")
        except DeviceException:
            self._logger.warning("UseCase tried to set camera parameter exposure mode on a device that does not "
                                 "support this parameter.")

        #Set picture resolution
        self.__camera_api.set_picture_size(640, 480)
        time.sleep(self._wait_btwn_cmd)

        #Set white balance
        self.__camera_api.set_white_balance('auto')
        time.sleep(self._wait_btwn_cmd)

        #Set flash mode
        try:
            #if flash mode is not supported by device, then catch the exception here
            self.__camera_api.set_flash_mode('auto')
            time.sleep(self._wait_btwn_cmd)
        except AcsConfigException:
            self._logger.warning("UseCase tried to set camera parameter flash on a device that does not support this "
                                 "parameter.")
        except DeviceException:
            self._logger.warning("UseCase tried to set camera parameter flash on a device that does not support this "
                                 "parameter.")

        #Set scene mode
        try:
            self.__camera_api.set_scene_mode('auto')
            time.sleep(self._wait_btwn_cmd)
        except AcsConfigException:
            self._logger.warning("UseCase tried to set camera parameter scene mode on a device that does not support "
                                 "this parameter.")
        except DeviceException:
            self._logger.warning("UseCase tried to set camera parameter scene mode on a device that does not support "
                                 "this parameter.")

        result, output = Global.SUCCESS, ""
        return Global.SUCCESS, ""

    def run_test(self):
        UseCaseBase.run_test(self)

        result = Global.SUCCESS
        return_message = ""

        # Take a picture with default settings
        picture_uri = self.__camera_api.take_picture(self.__save_directory)
        time.sleep(self._wait_btwn_cmd)

        # Download picture taken from DUT
        local_picture_uri = self.__camera_api.download_media_file(picture_uri, self.__save_folder)

        # Check if picture is corrupted
        corrupted = False

        if Imagecheck.check_image_corrupt(local_picture_uri):
            corrupted = True

        if corrupted:
            verdict = Global.FAILURE
            msg = "Picture taken is corrupted"
            return verdict, msg

        exif_value = Imagecheck.get_picture_exif_data(local_picture_uri, self.__exif_parameter_to_check)

        self._logger.info("Exif parameter has value " + str(exif_value))
        if self.__exif_parameter_value not in (None, ""):
            if str(exif_value) != str(self.__exif_parameter_value):
                result = Global.FAILURE
                return_message = "EXIF parameter {0} value is different than the one stated in the test case:"\
                    .format(self.__exif_parameter_to_check) + \
                "Expected: {0}, returned: {1}.".format(self.__exif_parameter_value,
                                                       exif_value)
        else:
            self._logger.info("No reference value for EXIF parameter was given. Only checking if it exists.")
            if exif_value in (None, ""):
                result = Global.FAILURE
                return_message = "EXIF parameter does not exist in picture."
            else:
                result = Global.SUCCESS
                return_message = "EXIF value for the given parameter exists: %s" % exif_value

        return result, return_message
