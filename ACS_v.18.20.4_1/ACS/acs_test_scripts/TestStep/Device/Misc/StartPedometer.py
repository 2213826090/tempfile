"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL SI PNP
:summary: This file implements a Test Step to start PnP mode for Pedometer test case
:since: 07/10/2014
:author: matton
"""

from Device.DeviceManager import DeviceManager
from Core.TestStep.TestStepBase import TestStepBase


class StartPedometer(TestStepBase):

    """
    Stop Skype app
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._logger.info('START_PEDOMETER')

        # Get device manager object
        device_manager_obj = DeviceManager()

        # Get device object
        device_obj = device_manager_obj.get_device(self._pars.device)

        # Enable HeciWriteAndroid on platform
        adbCmd = "adb shell chmod 777 %s" % self._pars.heciwriteandroid_file
        device_obj.run_cmd(cmd=adbCmd, timeout=120)

        # Enable PnP Mode for Pedometer Test Case
        adbCmd = "adb shell %s %s" % (self._pars.heciwriteandroid_file, self._pars.pedometer_mode_file)
        device_obj.run_cmd(cmd=adbCmd, timeout=120)

        msg = "Pedometer mode is started on device {0}".format((self._pars.device))
        self.ts_verdict_msg = msg
