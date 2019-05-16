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
:summary: This file implements the Flash UC
:since: 01/12/2011
:author: ssavrimoutou
"""

from Core.PathManager import Paths
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, Verdict2Global, str_to_bool
import os


class LabSystemFlash(UseCaseBase):

    """
    Lab System Flash UC Class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._handle_power_cycle = False

        # Initialize flash attributes
        self._flash_file_path = None
        self._flash_timeout = None

        # Check if flash is mandatory for this campaign
        self.flash_mandatory = str_to_bool(global_config.campaignConfig.get("FLASH_MANDATORY", "True"))

        # Get the flash file path value either from parameter file or
        # default value from phone catalog
        self._flash_file_path = self._tc_parameters.get_param_value("FLASH_FILE_PATH", "")
        # Getting the flag `CHECK_BOARD_AVAILABILITY`
        self._check_board_availability = self._tc_parameters.get_param_value("CHECK_BOARD_AVAILABILITY", False,
                                                                             default_cast_type="str_to_bool")
        if not self._flash_file_path:
            self._flash_file_path = Paths.FLASH_FILES
            self.get_logger().warning("Flash file is not defined,"
                                      "using value from ACS -f option ({0})".format(self._flash_file_path))

        # Get flash module config if specified
        # it will override the standard module configuration
        self._optional_module_config = self._tc_parameters.get_param_value("FLASH_MODULE_CONFIG", default_value="",
                                                                           default_cast_type=str)
        if self._optional_module_config:
            # could be a full path to xml file
            if not os.path.isfile(self._optional_module_config):
                # could relative path from TC path
                my_path = os.path.dirname(os.path.abspath(self._testcase_file_name))
                relative_path = os.path.join(my_path, self._optional_module_config)
                if os.path.isfile(relative_path):
                    self._optional_module_config = relative_path
                else:
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                             "File {0} has not been found!".format(self._optional_module_config))
        # Get boot_after_flash value from test case XML file
        self._boot_required = self._tc_parameters.get_param_value("BOOT_AFTER_FLASH", default_value=True,
                                                                  default_cast_type="str_to_bool")
        # Get flash module
        self._flash_module = self._device.get_device_module("FlashModule")

        # Get flashTimeout value either from parameter file or
        # default value from module config
        self._flash_timeout = self._tc_parameters.get_param_value("FLASH_TIMEOUT", default_cast_type=int)
        if self._flash_timeout is None:
            self._flash_timeout = self._flash_module.configuration.get_value("flashTimeout", 600, int)
            self.get_logger().warning("Flash timeout is not defined, "
                                      "using value from Module Config ({0})".format(self._flash_timeout))
        elif self._flash_timeout <= 0:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Wrong Flash timeout value specified {0}, "
                                     "it should be an integer > 0".format(self._flash_timeout))

    def _do_override_parameter(self):
        """
        Override flash module configuration if needed
        """
        if self._optional_module_config:
            self._flash_module.configuration_file = self._optional_module_config
            self._flash_module.load_conf()

    def run_test(self):
        """
        Execute the test
        """
        # Run UC base run_test
        return_code, return_msg = UseCaseBase.run_test(self)

        # Call flash sequence
        if return_code == Global.SUCCESS:
            # override module parameters if necessary
            self._do_override_parameter()
            # run the init of the flash module
            # It will check all the flash tools availability and flash file integrity
            return_code = self._flash_module.init(self._flash_file_path)
            if return_code == Global.SUCCESS:
                # run the flash of the flash module
                # Flash the device
                self.get_logger().info("Flashing file %s onto %s target with flash timeout %s...",
                                       self._flash_file_path,
                                       self._device.get_phone_model(), self._flash_timeout)

                # flash only if mandatory or build flashed is different
                need_to_flash = True
                if not self.flash_mandatory:
                    bn = self._flash_module.get_build_name()
                    if bn:
                        ret, _ = self._device.switch_on(boot_timeout=60)
                        if ret == Global.SUCCESS:
                            build_version = self._device.get_property_value("ro.build.version.incremental")
                            if build_version == bn:
                                need_to_flash = False
                if need_to_flash:
                    return_code = self._flash_module.flash(self._flash_timeout)
                else:
                    return_code = Global.SUCCESS
                    return_msg = "build version : %s is already flashed, skipping flash" % str(build_version)
                    self.get_logger().info(return_msg)
            else:
                return_msg = "Cannot initialize the flash on the device"

            # Check if flash failed
            if return_code == Global.SUCCESS and self._boot_required:
                # Flash succeeded, connect the board
                self._device.switch_on()

                if not self._device.is_available():
                    # Connection with the board failure
                    return_code = Global.FAILURE
                    return_msg = "Flash succeeded but connection with the board fails."

            # Is the board available?
            if return_code == Global.SUCCESS and self._check_board_availability:
                return_code = Verdict2Global.map[self._device.get_reporting_device_info()[0]]
                return_msg = "Flash succeeded {}!".format("and board is available"
                                                          if return_code == Global.SUCCESS
                                                          else "but board is not available")

        # Return the verdict of the flash
        return return_code, return_msg
