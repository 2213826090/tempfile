"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL MCG PSI
@summary: This file implements a Test Step to switch off the device
@since 01/26/2016
@author: sdubrayx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import Global


class RecoverDevice(DeviceTestStepBase):
    """
    Boot the device in MOS using hardware action, if possible
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        count = 0
        is_rooted = self._device.enable_adb_root()
        try_nb = self._pars.try_nb

        while count < try_nb and not is_rooted:
            count += 1
            msg = "Recover the device with hard shutdown & switch on (try %d/%d)" % (count, try_nb)
            self._logger.warning(msg)

            status = self._device.hard_shutdown()
            if status != True:
                self._logger.warning("Hard shutdown failed (try %d/%d)" % (count, try_nb))

            status, _ = self._device.switch_on()
            if status != Global.SUCCESS:
                self._logger.warning("Device not switched on (try %d/%d)" % (count, try_nb))

            is_rooted = self._device.enable_adb_root()
