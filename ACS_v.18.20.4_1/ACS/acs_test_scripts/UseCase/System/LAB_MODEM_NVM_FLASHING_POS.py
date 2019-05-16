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
:summary: This file implements the Modem NVM Flash with DUT in POS Use Case
:since: 06/09/2012
:author: asebbanx
"""

import re

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.System.LAB_MODEM_FLASHING_BASE import LabModemFlashingBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.ModemFlashing \
    import NvmFastbootResult


class LabModemNvmFlashingPos(LabModemFlashingBase):

    """
    Lab Modem NVM Flash with DUT in I{POS} (Provisioning OS) Use Case Class.
    It is assumed that the DUT is already in I{POS} when this Use Case is executed.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        LabModemFlashingBase.__init__(self, tc_name, global_config)

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run UC base run_test
        LabModemFlashingBase.run_test(self)

        # Create a flash result for the expected return value
        cmfwdl_expected_result = self._modem_flashing_api._get_cmfwdl_result()  # pylint: disable=W0212
        cmfwdl_expected_result_code = cmfwdl_expected_result.from_string(self._expected_result)

        # Initialize return message
        return_msg = None

        # Check the expected result code before going any further
        if cmfwdl_expected_result_code is None:
            message = "Invalid parameter value: %s for parameter '%s'." % \
                (str(self._expected_result), "EXPECTED_RESULT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Create a flash result for the expected return value
        cmfwdl_actual_result = self._modem_flashing_api._get_cmfwdl_result()  # pylint: disable=W0212

        # Call flash sequence
        (return_code, stdout_string) = \
            self._modem_flashing_api.flash_nvm_pos(
                self._flash_file_path,
                self._target_directory_path)
        cmfwdl_actual_result_code = return_code

        # Check the error code on the stdout.
        # Note that the error code is only written
        # when an error occurred.
        if return_code != NvmFastbootResult.NVM_ERR_SUCESS:
            stdout_string = stdout_string.translate(None, '\r\n')
            matcher = re.search("Error (\d)", stdout_string)
            if matcher is not None:
                error_value = matcher.group(1)
                cmfwdl_actual_result_code = int(error_value)
            else:
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                    "The application cmfwdl-app did not print any error "
                    "or exit status on the stdout. (%s)" % stdout_string)

        # Check if flash failed
        if cmfwdl_actual_result_code != cmfwdl_expected_result_code:
            return_code = Global.FAILURE
            cmfwdl_actual_result_text = cmfwdl_actual_result.to_string(cmfwdl_actual_result_code)
            message = "Flash test failed. Expected flash result was %s (%s) got %s (%s)." % (
                str(self._expected_result),
                str(cmfwdl_expected_result_code),
                str(cmfwdl_actual_result_text),
                str(cmfwdl_actual_result_code))
            self._logger.error(message)
            return_msg = stdout_string
        else:
            return_code = Global.SUCCESS
            return_msg = "Flash test success. Got expected result: %s (%d)." % \
                (str(self._expected_result), cmfwdl_actual_result_code)

        # Return the verdict of the flash
        return return_code, return_msg
