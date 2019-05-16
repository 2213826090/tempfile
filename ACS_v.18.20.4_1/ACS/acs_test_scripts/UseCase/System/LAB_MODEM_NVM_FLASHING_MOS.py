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
:summary: This file implements the Modem NVM Flash with DUT in MOS Use Case
:since: 06/09/2012
:author: asebbanx
"""

import re
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.System.LAB_MODEM_FLASHING_BASE import LabModemFlashingBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.ModemFlashing \
    import NvmFastbootResult


class LabModemNvmFlashingMos(LabModemFlashingBase):

    """
    Lab Modem NVM Flash with DUT in I{MOS} (Main OS) Use Case Class.
    It is assumed that the DUT is already in I{MOS} when this Use Case is executed.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        LabModemFlashingBase.__init__(self, tc_name, global_config)

        # Initialize attributes with default values
        self._flash_tty = None
        self._flash_app = None

        # Get the TTY to use for the flash operation
        flash_tty = self._tc_parameters.get_param_value("FLASH_TTY")
        if flash_tty is not None and flash_tty != "":
            self._flash_tty = flash_tty

        # Get the application to use for the flash operation
        flash_app = self._tc_parameters.get_param_value("FLASH_APPLICATION")
        if flash_app is not None and flash_app != "":
            self._flash_app = flash_app

        # Initialize the possible values to look for
        self._possible_values = {
            "NVM_CONFIGURATION_APPLIED": "Configuration Applied",
            "NVM_CONF_NOT_APPLIED": "Error while applying configuration",
            "NVM_CONF_COMMAND_FAILED": "Configuration Command Failed"
        }

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base run_test
        LabModemFlashingBase.set_up(self)

        # Check the parameters
        if self._expected_result not in self._possible_values.keys():
            message = "Invalid parameter value: %s for parameter '%s'." % \
                (str(self._expected_result), "EXPECTED_RESULT")
            self._logger.error(message)
            return Global.FAILURE, message
        return Global.SUCCESS, "No error."

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test.
        """

        # Run UC base run_test
        LabModemFlashingBase.run_test(self)

        # Check the TTY attribute value
        if self._flash_tty is None:
            message = "Invalid parameter value for parameter '%s'." % "FLASH_TTY"
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Call the flash sequence with the provided flashing application
        (return_code, return_msg, stdout, stderr) = \
            self._modem_flashing_api.flash_nvm_mos(
                self._flash_file_path,
                self._flash_tty,
                self._flash_app,
                self._target_directory_path)

        # Check the error code on the stdout.
        # Note that the error code is only written
        # when an error occurred.
        if return_code == NvmFastbootResult.NVM_ERR_SUCESS:
            stdout = stdout.translate(None, '\r\n')

            expected_result = self._possible_values[self._expected_result]
            matcher = re.search("%s" % expected_result, stdout)
            if matcher is not None:
                return_code = Global.SUCCESS
                return_msg = "Flash test success. Found expected result: %s (%s)." % \
                    (str(self._expected_result), expected_result)
            else:
                return_code = Global.FAILURE
                return_msg = "Flash test failed. Expected flash result was %s (%s)." % (
                    str(self._expected_result),
                    str(expected_result))
                self._logger.error(return_msg)
        else:
            return_msg = stderr

        # Wait until the modem is up before
        # going any further in the campaign
        self._logger.debug("Waiting for the modem to be up")
        time.sleep(15)

        # Return the verdict of the flash
        return return_code, return_msg
