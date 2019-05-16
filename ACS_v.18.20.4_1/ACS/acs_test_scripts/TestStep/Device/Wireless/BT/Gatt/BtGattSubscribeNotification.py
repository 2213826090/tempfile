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

:organization: INTEL NDG SW DEV
:summary: This file implements a Test Step to subscribe to gatt notification (bluetooth LE)
:since:08/01/2015
:author: jreynaux
"""
from TestStep.Device.Wireless.BT.Base import BtBase


class BtGattSubscribeNotification(BtBase):
    """
    Implements the BtGattSubscribeNotification test step for BT
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        service_name = str(self._pars.service_name)
        char_name = str(self._pars.characteristic_name)
        subscribe = self._pars.subscribe

        if subscribe:
            self._logger.info("Subscribe to notification about characteristic {0} on service {1}".format(char_name, service_name))
            self._api.bt_gatt_subscribe_notification(service_name=service_name, char_name=char_name)
        elif not subscribe:
            self._logger.info("Unsubscribe to notification about characteristic {0} on service {1}".format(char_name, service_name))
            self._api.bt_gatt_unsubscribe_notification(service_name=service_name, char_name=char_name)