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
:summary: This file implements the CRM testing Use Case
:since: 04/02/2016
:author:nowelchx
"""


from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveCRM(UseCaseBase):

    """
    LIVE CRM test use case class.
    """
    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize CRM test parameters
        self._cmd_str = None
        self._expected_result = None
        self._test_timeout = None

        # Get the command to execute
        self._cmd_str = self._tc_parameters.get_param_value("RUN_CMD")

        # Get the expect.ed result
        expected_result = \
            self._tc_parameters.get_param_value("EXPECTED_RESULT")
        if expected_result is not None and expected_result != "":
            self._expected_result = expected_result

        # Get the execution timeout
        test_timeout = self._tc_parameters.get_param_value("EXECUTION_TIMEOUT")
        if test_timeout is not None and test_timeout != "":
            self._test_timeout = int(test_timeout)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base run_test
        UseCaseBase.set_up(self)

        # Check the send command before going any further
        if self._cmd_str is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._cmd_str),
                "RUN_CMD")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the expected result str before going any further
        if self._expected_result is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._expected_result),
                "EXPECTED_RESULT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the test execution timeout before going any further
        if self._test_timeout is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._test_timeout),
                "EXECUTION_TIMEOUT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)
        # Return the status
        return Global.SUCCESS,"No error."

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Launch the test scenario main command for performing the test
        (response, output) = self._device.run_cmd(str(self._cmd_str),
                                                  self._test_timeout,
                                                  force_execution=True)
        # Check if the configuration commands is successful or not
        if self._check_adb_response(str(self._expected_result), output):
            return_code = Global.SUCCESS
            return_msg = "The adb command was successfully executed"
        else:
            return_code = Global.FAILURE
            return_msg = "The test case scenario has not the expected response; The command FAILED"
            self._logger.error(return_msg)

        # Return the verdict
        return return_code, return_msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        # Reboot the device to return to original state
        self._device.reboot()

        # Return the verdict
        return Global.SUCCESS, "No error."

#------------------------------------------------------------------------------

    def _check_adb_response(self, expected_key, adb_response):
        """
        Check the if the adb command response is the expected one

        Parameter:
        :param expected_key: a certain word/expression to be looked for in the adb response
        :type expected_key: str
        :param adb_response: adb reponse to the command that is checked
        :type adb_response: str

        Return value:
        :return verdict: if the expression is fapund in the adb response -> TRUE, otherwise -> FALSE
        :rtype verdict: boolean
        """

        # Check if the adb command is successful or not
        if expected_key in adb_response:
            verdict = True
        else:
            verdict = False

        return verdict

    #------------------------------------------------------------------------------
