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
:summary: This script implements the TestStep for stopping a BLE advertising by callback code on enabled Android
        device
:since: 7/30/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.BLE.BleBase import BleBase
from ErrorHandling.DeviceException import DeviceException


class BleStopAdvertising(BleBase):
    """
    This script implements the TestStep for stopping a BLE advertising by callback code on enabled Android device
    """

    def run(self, context):
        """
       This test step will fail if at any point an error occurred and the advertising could not be stopped or you are
       attempting to stop an advertisement that was not start and the associated callback object is null
        :param context:
        :return:
        """

        advertise_code = BleBase.check_advertise_code_valid(self, self._pars.advertise_code)

        BleBase.run(self, context)

        if (advertise_code is not False):
            self._api.ble_stop_advertising(advertise_code)
        else:
            msg = "You have not provided a valid code for ADVERTISE_CODE :" \
                  "advertiseCode: %s" % str(advertise_code)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)