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
:summary: This file implements a Test Step to check some properties of path
:since:18/09/2014
:author: msouyrix
"""
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
import os.path

class CheckPath(TestStepBase):
    """
    Check some properties of path
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)
        path_to_check = self._pars.path_to_check
        operator = self._pars.operator

        self._logger.info("CheckPath: path to check '{0}', operator '{1}'".format(path_to_check, operator))

        # Do required check
        result = False
        if operator == "EXIST":
            result = os.path.exists(path_to_check)
        elif operator == "IS_FILE":
            result = os.path.isfile(path_to_check)
        elif operator == "IS_DIRECTORY":
            result = os.path.isdir(path_to_check)

        # Invert result if needed
        if not self._pars.pass_if:
            result = not result

        if not result:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED, "Path check not satisfied")
