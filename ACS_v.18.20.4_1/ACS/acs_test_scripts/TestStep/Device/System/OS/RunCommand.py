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
:summary: This file implements the execution of commands on device
:since: 02/09/2014
:author: floeselx
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class RunCommand(DeviceTestStepBase):
    """
    Mount partition class
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Fetch params values
        timeout = self._pars.timeout
        command = self._pars.command
        force_execution = self._pars.force_execution
        wait_for_response = self._pars.wait_for_response
        silent_mode = self._pars.silent_mode
        command_result = self._pars.save_as

        status, result = self._device.run_cmd(command, timeout,
                                              force_execution, wait_for_response, silent_mode)
        self._logger.info("Content of result: " + result)

        context.set_info(command_result, result)

        if status == Global.FAILURE:
            msg = "Error at command execution : %s" % str(result)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)



