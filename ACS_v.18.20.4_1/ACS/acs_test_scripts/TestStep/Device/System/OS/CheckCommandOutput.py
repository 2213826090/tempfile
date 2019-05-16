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
:summary: This file implements the check after executing a command on a device
:since: 04/30/2015
:author: mmaraci
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class CheckCommandOutput(DeviceTestStepBase):
    """
    Check if a given string is in the one provided as a result of an executed command
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Fetch params values
        # timeout = self._pars.timeout
        output_to_check = self._pars.output_to_check
        check_content = self._pars.check_content

        if check_content not in output_to_check:
            msg = 'The content you are searching for is not present in the provided output: {0} versus {1}'.format(check_content, output_to_check)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)