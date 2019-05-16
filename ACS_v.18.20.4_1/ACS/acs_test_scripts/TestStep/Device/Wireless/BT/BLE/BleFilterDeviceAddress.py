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
:summary: This script implements the TestStep filtering a BLE Advertisement by the emitting device's MAC Address
:since: 7/30/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.BLE.BleBase import BleBase
from ErrorHandling.DeviceException import DeviceException


class BleFilterDeviceAddress(BleBase):
    """
    This script implements the TestStep filtering a BLE Advertisement by the emitting device's MAC Address
    """

    def run(self, context):
        """
        If no device is returned by the Agent filtered search, the the operation has failed
        :param context:
        :return:
        """

        scan_mode = BleBase.check_scan_mode_valid(self, self._pars.scan_mode)

        BleBase.run(self, context)

        if scan_mode is not False:
            bluetooth_device = self._api.ble_scan_filter_address(self._pars.remote_address, scan_mode,
                                                                 self._pars.timeout)
        else:
            msg = "You have not provided a valid code for SCAN_MODE"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not bluetooth_device:
            msg = "No device was found using the address filter: " + str(self._pars.remote_address)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)