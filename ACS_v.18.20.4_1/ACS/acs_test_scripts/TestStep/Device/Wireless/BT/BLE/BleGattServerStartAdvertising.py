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
:summary: This script implements the TestStep for starting to advertise a GATT Server
:since: 8/28/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.BLE.BleBase import BleBase
from ErrorHandling.DeviceException import DeviceException


class BleGattServerStartAdvertising(BleBase):
    """
    This script implements the TestStep for starting to advertise a GATT Server
    """

    def run(self, context):
        """
        This test step will fail if at any point an error occurred and the advertising could not be started
        or the server was not created properly
        :param context:
        :return:
        """

        advertise_mode = BleBase.check_advertise_mode_valid(self, self._pars.advertise_mode)
        tx_power_level = BleBase.check_tx_power_level_valid(self, self._pars.tx_power_level)
        gatt_server_type = BleBase.check_gatt_server_type(self, self._pars.gatt_server_type)

        BleBase.run(self, context)

        if (advertise_mode is not False) and (tx_power_level is not False) and (gatt_server_type is not False):
            output = self._api.ble_gatt_advertise_server(tx_power_level, advertise_mode, self._pars.advertise_timeout,
                                                gatt_server_type)
            self._logger.debug(output)
        else:
            msg = "You have not provided a valid code for ADVERTISE_MODE and/or TX_POWER_LEVEL and/or GATT_SERVER_TYPE" \
                  " advertiseMode: %s tx_power_level: %s gatt_server_type: %s" % \
                  (str(advertise_mode), str(tx_power_level), str(gatt_server_type))
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)