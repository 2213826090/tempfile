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

:organization: INTEL NDG sw
:summary: This file implements the NTP setting of the DUT
:since: 2014-10-21
:author: dpierrex

"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import UtilitiesFWK.Utilities as Util


class SetAvailableMemory(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._system_api = self._device.get_uecmd("System")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        if self._pars.restoration:
            self._system_api.restore_available_memory()
            self._device.reboot(wait_settledown_duration=True)
        else:
            if self._pars.memory_top.endswith("M"):
                try:
                    memory_top = int(self._pars.memory_top[:-1]) * 1024 * 1024
                except ValueError:
                    msg="SetAvailableMemory: memory top {0} is an invalid format. You can use suffix 'M' with decimal values. i.e., 1024M".format(self._pars.memory_top)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            elif self._pars.memory_top.endswith("k"):
                try:
                    memory_top = int(self._pars.memory_top[:-1]) * 1024
                except ValueError:
                    msg="SetAvailableMemory: memory top {0} is an invalid format. You can use suffix 'k' with decimal values. i.e., 1048576k".format(self._pars.memory_top)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            elif self._pars.memory_top.startswith("0x"):
                try:
                    memory_top = int(self._pars.memory_top, 16)
                except ValueError:
                    msg="SetAvailableMemory: memory top {0} is an invalid format. You can use hexadecimal values with prefix 0x. i.e., 0x40000000".format(self._pars.memory_top)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                try:
                    memory_top = int(self._pars.memory_top)
                except ValueError:
                    msg="SetAvailableMemory: memory_top {0} is an invalid format. It must be decimal, hexadecimal with '0x' prefix, or a decimal with a 'k' or 'M' suffix.".format(self._pars.memory_top)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self._system_api.set_available_memory(memory_top)
            self._device.reboot(wait_settledown_duration=True)
