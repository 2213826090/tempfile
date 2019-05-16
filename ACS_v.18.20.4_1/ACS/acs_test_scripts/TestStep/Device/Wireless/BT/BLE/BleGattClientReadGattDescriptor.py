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
:summary: This script implements the TestStep that tries to read a GATT Descriptor from a server it is connected to
:since: 8/28/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.BLE.BleBase import BleBase


class BleGattClientReadGattDescriptor(BleBase):
    """
    This script implements the TestStep that tries to read a GATT Descriptor from a server it is connected to
    """

    def run(self, context):
        """
        If the descriptor cannot be read, the the operation has failed
        :param context:
        :return:
        """

        BleBase.run(self, context)

        output = self._api.ble_gatt_client_read_descriptor(self._pars.address)
        self._logger.debug(output)