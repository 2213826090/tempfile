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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to reboot the device
:since:10/01/2014
:author: ssavrim
"""
import time

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class RebootDevice(DeviceTestStepBase):
    """
    Reboot the device in MOS either using hardware action or software action
    """
    HARD_WAY = "hard"
    SOFT_WAY = "soft"

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        # SOFT REBOOT
        if self._pars.way == self.SOFT_WAY:
            # reboot will all default options
            if self._pars.mode in ["", None]:
                self._device.reboot()
            else:
                self._device.reboot(mode=self._pars.mode)

        # HARD REBOOT
        else:
            if not self._device.is_booted():
                status, log = self._device.switch_on()
                if status != Global.SUCCESS:
                    raise DeviceException(DeviceException.DUT_BOOT_ERROR, log)
            else:
                status, log = self._device.switch_off()
                if status == Global.SUCCESS:
                    status, log = self._device.switch_on()
                    if status != Global.SUCCESS:
                        raise DeviceException(DeviceException.DUT_BOOT_ERROR, log)
                else:
                    raise DeviceException(DeviceException.DUT_SHUTDOWN_ERROR, log)
