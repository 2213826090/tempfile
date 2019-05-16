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

:organization: INTEL SSG
:summary: This file implements the execution of a python script
:since: 27/01/2016
:author: tchourrx
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
import os
import subprocess


class ExecuteBoardSetupScript(DeviceTestStepBase):
    """
    This file implements the execution of a python script
    TEMPORARY patch to help PnP otimization, can't be used for official noreg or weekly tests
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Check if parameter SetupScriptPath is defined in the benchconfig, else do nothing
        script_path = DeviceManager().get_device_config("PHONE1").get("SetupScriptPath", None)

        if script_path is not None:
            self._logger.warning("WARNING - Dirty solution is used, need to be redefined !")
            if os.path.exists(script_path):
                script = os.path.basename(self._testcase_name) + ".py"
                self._logger.debug("script file: %s" % script)

                try:
                    if os.path.isfile(os.path.join(script_path, "all.py")):
                        result = subprocess.check_output(["python", os.path.join(script_path, "all.py")])
                        self._logger.debug("all.py script output: %s" % result)
                    if os.path.isfile(os.path.join(script_path, script)):
                        result = subprocess.check_output(["python", os.path.join(script_path, script)])
                        self._logger.debug("%s script output: %s" % (script, result))
                except subprocess.CalledProcessError:
                        msg = "Error, during board setup script execution : %s" % str(result)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "Error, script path defined does not exist : %s" % str(script_path)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            self._logger.debug("ExecuteBoardSetupScript step not used")