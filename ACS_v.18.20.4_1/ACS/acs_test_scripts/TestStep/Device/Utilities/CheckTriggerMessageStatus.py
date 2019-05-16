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
:summary: This file implements a Test Step to check a trigger log is available in DUT logs
:since: 03/12/2014
:author: kturban
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class CheckTriggerMessageStatus(DeviceTestStepBase):
    """
    Check that trigger message has been triggered
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        res = None
        end_time = time.time() + self._pars.timeout
        while (time.time() < end_time):
            res = self._device.get_device_logger().get_message_triggered_status(self._pars.trigger_message)
            if res:
                break

        if not res:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't get trigger message '{0}' in DUT logs".format(self._pars.trigger_message))
        self._ts_verdict_msg = "Message '{0}' has been triggered".format(self._pars.trigger_message)
        if self._pars.success_output:
            self._ts_verdict_msg = "SUCCESS: {0}".format(self._pars.success_output)
        self._logger.info(self._ts_verdict_msg)
