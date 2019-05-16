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

:organization: INTEL OTC SSG SI PnP
:summary: This file implements a Test Step to insert usb using io card
:since:10/01/2016
:author: tchourrx
"""
import time
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException


class InsertUsbWithRetries(DeviceTestStepBase, EquipmentTestStepBase):
    """
    Insert usb using io card
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._adbConnectionTimeout = self._device.get_config("adbConnectTimeout", 10, float)
        self._system_api = self._device.get_uecmd("System")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        EquipmentTestStepBase.run(self, context)

        io_card = self._equipment_manager.get_io_card(self._pars.eqt)
        if not io_card:
            self._logger.warning("_switch_on : No IO_CARD detected")
            raise DeviceException(DeviceException.OPERATION_FAILED, "No IO card detected, device can't be plugged with")

        for cnt in range(0, self._pars.retry):
            # Plug the USB
            if not io_card.usb_host_pc_connector(True):
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Fail to insert usb using {0}".format(self._pars.eqt))

            ret_code = self._system_api.wait_for_device(self._adbConnectionTimeout)
            if not ret_code:
                self._logger.warning("timeout on wait-for-device, trying to unplug/replug (try %s/%s)" %
                                    (str(cnt+1), str(self._pars.retry)))
                io_card.usb_host_pc_connector(False)
                time.sleep(self._pars.sleep_between_retry)
                continue

            self._logger.debug("device retrieved after %s tries" % str(cnt+1))
            return

        raise DeviceException(DeviceException.OPERATION_FAILED,
                              "Could not retrieve the device after %s plug/unplug cycles" %
                              str(self._pars.retry))

