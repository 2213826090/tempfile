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
:summary: This file implements the IPC testing Use Case
:since: 08/07/2015
:author: amitrofx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveIPC(UseCaseBase):

    """
    LIVE IPC test use case class.
    """
    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize ipc test parameters
        self._cmd_str = None
        self._expected_result = None
        self._test_timeout = None

        # Get the command to execute
        self._cmd_str = self._tc_parameters.get_param_value("RUN_CMD")

        # Get the expected result
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

        # Provide access rights in the folders where
        # IPCbox tool will be installed
        self._provide_ipc_box_acces_rights()

        # Perform the IPC preparation before properly starting each test
        (return_code, return_msg) = self._ipc_preparation()

        # Return the status
        return return_code, return_msg

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

    def _ipc_preparation(self):
        """
        Launch the commands necessary to put the phone in the
        proper state for the tests to start.
        In case one of the preparation commands will be failed an
        exception will be raised.

        Return value:
        :return response_code: the verdict of the test case depending on the pass/fail criteria
        :rtype response_code: str
        :return msg: a message to complete and explain the verdict and if the case the reasons of failure
        :rtype msg: str
        """

        # Activate the desired configuration on the device
        modem_type = ""
        platform_type = "SSIC"
        board_type = ""

        if "7260" in str(self._device.get_property_value("gsm.version.baseband")):
            modem_type = "7260"
        else:
            modem_type = "7360"

        if "cht" in str(self._device.get_property_value("ro.boot.hardware")):
            board_type = "CHT"

        config_command = "adb shell ipcbox configure default -i " + platform_type + " -s " + board_type + " -m " + modem_type + " -t nonreg"

        (response, output) = self._device.run_cmd(config_command,
                                                  self._test_timeout,
                                                  force_execution=True)
        # Check if the configuration commands is successful or not
        if self._check_adb_response("PASSED", output):
            response_code = Global.SUCCESS
            msg = "The adb command was successfully executed"
        else:
            response_code = Global.FAILURE
            msg = "The key PASSED was not found in the adb reponse; The command FAILED"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        # Disable telephony
        (response, output) = self._device.run_cmd("adb shell ipcbox dis_tel",
                                                  self._test_timeout,
                                                  force_execution=True)
        # Check if the disable telephony commands is successful or not
        if self._check_adb_response("PASSED", output):
            response_code = Global.SUCCESS
            msg = "The adb command was successfully executed"
        else:
            response_code = Global.FAILURE
            msg = "The key PASSED was not found in the adb reponse; The command FAILED"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        return response_code, msg

    #------------------------------------------------------------------------------

    def _provide_ipc_box_acces_rights(self):
        """
        Grant access rights in 2 folders of the device under
        test where the IPCbox tool files will be pushed.

        :raises AcsConfigException: In case one
            of the access rights adb commands will
            fail an exception will be raised.
        """

        # Installing IPC tool on the device under test
        # Assign executable rights to /system/bin folder
        (response, output) = self._device.run_cmd("adb shell chmod 777 /system/bin/*",
                                                  self._test_timeout,
                                                  force_execution=True)
        # Check if the adb command was properly sent
        if output <> "":
            self._logger.error("The adb command for guaranteeing access rights failed")
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "The adb command for guaranteeing access rights failed")

        # Assign executable rights to /etc/ipcbox folder
        (response, output) = self._device.run_cmd("adb shell chmod 777 /etc/ipcbox/*",
                                                  self._test_timeout,
                                                  force_execution=True)
        if output <> "":
            self._logger.error("The adb command for guaranteeing access rights failed")
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "The adb command for guaranteeing access rights failed")

    #------------------------------------------------------------------------------
