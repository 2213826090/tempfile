"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to insert a string in context
:since: 04/02/2015
:author: vdechefd
"""
import os
import sys

from Core.TestStep.TestStepBase import TestStepBase
from Core.PathManager import Paths
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.ExecScriptCtx import init_ctx


class ExecScript(TestStepBase):
    """
    Execute an external python script
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        script = self.__get_script_path(self._pars.script_path)
        self.__exec_script(script, context)

    def __exec_script(self, script, context):
        # Initialize execution context
        global_values = globals()
        init_ctx(global_values, self._logger, self._global_conf)

        # Tune the execution context for exec script test step
        my_path = os.path.dirname(os.path.abspath(script))
        global_values["MY_PATH"] = my_path
        global_values["EXEC_TS"] = self
        global_values["CTX"] = context
        global_values["ERROR_DEVICE"] = ExecScript.__error_device
        global_values["ERROR_ACSTOOL"] = ExecScript.__error_acstool
        global_values["ERROR_ACSCONFIG"] = ExecScript.__error_acsconfig
        global_values["ERROR_EQUIPMENT"] = ExecScript.__error_equipment

        sys.path.append(my_path)
        current_dir = os.getcwd()
        try:
            execfile(script, global_values)
        finally:
            os.chdir(current_dir)

    def __get_script_path(self, script_path):
        execution_config_path = os.path.abspath(Paths.EXECUTION_CONFIG)

        new_path = script_path
        if not os.path.exists(script_path):
            new_path = os.path.join(execution_config_path, script_path)

        if not os.path.exists(new_path):
            new_path = os.path.join(execution_config_path, os.path.dirname(self._testcase_name), script_path)

        if not os.path.exists(new_path):
            raise AcsConfigException("Unable to find exec script with path {0}".format(new_path))
        else:
            return new_path

    @staticmethod
    def __error_device(message):
        raise DeviceException(message)

    @staticmethod
    def __error_acstool(message):
        raise AcsToolException(message)

    @staticmethod
    def __error_acsconfig(message):
        raise AcsConfigException(message)

    @staticmethod
    def __error_equipment(message):
        raise TestEquipmentException(message)





