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

:organization: INTEL NDG sw
:summary: This file implements a Test Step to check if path exist on the DUT
:since: 23/10/2014
:author: dpierrex
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
import os.path


class CheckPathExist(DeviceTestStepBase):
    """
    Check some properties of path
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._file_api = self._device.get_uecmd("File")

    def run(self, context):
        """
        Runs the test step

        :type context: DeviceTestStepBase
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        path_to_check = self._pars.path_to_check
        operator = self._pars.operator

        self._logger.info("path to check '{0}', operator '{1}'".format(path_to_check, operator))

        # Do required check
        result = False
        exist, output = self._file_api.exist(path_to_check)
        if exist:
            if operator in ["EXIST", "IS_FILE"]:
                result = True
            elif operator == "IS_DIRECTORY" and ": directory" in output:
                result = True

        # Invert result if needed
        if not self._pars.pass_if:
            result = not result

        if not result:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED, "Path check not satisfied")
