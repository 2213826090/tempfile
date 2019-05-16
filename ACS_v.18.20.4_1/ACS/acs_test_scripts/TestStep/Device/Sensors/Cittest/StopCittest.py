"""
@summary: Stop Cittest application on device
@since 23 September 2014
@author: Souyris Matthieu
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
import time


class StopCittest(DeviceTestStepBase):
    """
    Stop Cittest application on device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def __exec_cmd(self, cmd_to_exec):
        """
        Exec an adb command and take verdict into account
        :type cmd_to_exec: str
        :param cmd_to_exec: adb command to exec
        :return: content printed by the command on stdout
        """
        self._logger.debug("StopCittest: command to execute: '{0}'".format(cmd_to_exec))
        verdict, res_cmd = self._device.run_cmd(cmd_to_exec, timeout=10, silent_mode=True)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "StopCittest: command execution failed")

        return res_cmd

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        # Launch stop command
        self.__exec_cmd("adb shell am force-stop com.lst.app.cit")

        # Check that no more cittest application running
        process_info = "exist"
        nbr_attempt = 0
        while process_info != "" and nbr_attempt < 3:
            time.sleep(1.0)
            process_info = self.__exec_cmd("adb shell ps |grep 'com.lst.app.cit'")
            if not process_info:
                break
            self._logger.debug("StopCittest: Cittest still running, try one more time")
            nbr_attempt += 1

        if process_info:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "StopCittest: cittest still running (cittest stop seems failed")
