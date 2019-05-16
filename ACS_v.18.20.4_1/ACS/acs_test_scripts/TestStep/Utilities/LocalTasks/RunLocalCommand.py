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

:organization: INTEL NDG
:summary: This file implements the execution of commands on local computer
:since: 18/09/2014
:author: msouyrix
"""
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.Utilities import Global, internal_shell_exec


class RunLocalCommand(TestStepBase):
    """
    Run local command
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context

        Context parameters list:
        - cmd: command to be run
        - timeout: script execution timeout in sec
        - silent_mode: Display logs in ACS logger
        - save_as: context variable name where stdout of the command is put
        """
        TestStepBase.run(self, context)

        # Fetch params values
        command = self._pars.command
        timeout = self._pars.timeout
        silent_mode = self._pars.silent_mode
        command_result = self._pars.save_as

        exit_status, output = internal_shell_exec(cmd=command, timeout=timeout, silent_mode=silent_mode)
        if not silent_mode:
            self._logger.info("RunLocalCommand: output for {0} command: '{1}'".format(command, output))

        if exit_status == Global.FAILURE:
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR,
                                  "RunCommand: Error at execution of '{0}' command: {1}".format(command, output))

        context.set_info(command_result, output)
