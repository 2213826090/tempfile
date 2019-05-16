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
:summary: This file implements a Test Step to get trigger messages catched from DUT logs
:since: 16/06/2016
:author: tchourrx
"""
import re
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class GetTriggerMessages(DeviceTestStepBase):
    """
    Get trigger messages triggered
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        ret = self._device.get_device_logger().get_message_triggered_status(self._pars.trigger_message)

        if not ret:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't get trigger message '{0}' in DUT logs".format(self._pars.trigger_message))

        if "regex:" in self._pars.trigger_message:
            regex = self._pars.trigger_message.split("regex:")[-1]
        else:
            regex = self._pars.trigger_message

        result = [list(re.search(regex, line).groups()) for line in ret]

        context.set_info(self._pars.result, result)