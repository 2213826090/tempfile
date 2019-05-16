"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android SSG
:summary: This script implements the TestStep for starting a GAP advertising filtering of the possible power levels
available device
:since: 7/30/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.BLE.BleBase import BleBase
from ErrorHandling.DeviceException import DeviceException


class BleFilterAdvertisedPowerLevels(BleBase):
    """
    TestStep that filters a BLE advertisement of multiple power levels

    """

    def run(self, context):
        """
        If the result contains 4 different found advertisements, then the result is PASS.
        Any different and it means that the advertisement is not being sent or read correctly
        :param context:
        :return:
        """

        scan_mode = BleBase.check_scan_mode_valid(self, self._pars.scan_mode)

        BleBase.run(self, context)

        if scan_mode is not False:
            broadcasted_instances = self._api.ble_scan_advertising_power_levels(scan_mode, self._pars.timeout)
        else:
            msg = "You have not provided a valid code for ADVERTISE_MODE"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not broadcasted_instances:
            msg = "No broadcast was observed"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        count_advertised_instances = len(broadcasted_instances)

        if count_advertised_instances != 4:
            msg = "There should be 4 different addresses advertised for each power level, actual count is: " +\
                  str(count_advertised_instances)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        for instance in broadcasted_instances:
            self._logger.debug("Found broadcast result: " + instance.to_string())