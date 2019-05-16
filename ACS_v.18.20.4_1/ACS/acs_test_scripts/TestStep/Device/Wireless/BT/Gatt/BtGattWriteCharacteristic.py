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
:summary: This file implements a Test Step to write on gatt characteristic (bluetooth LE)
:since:13/10/2014
:author: jreynaux
"""
from TestStep.Device.Wireless.BT.Base import BtBase


class BtGattWriteCharacteristic(BtBase):
    """
    Implements the base test step for BT
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
        data = self._pars.data
        data_type = self._pars.data_type
        acknowledgement = self._pars.acknowledgement

        self._logger.info("Write {0} on characteristic {1} on service {2} as {3} type".format(data, char_name,
                                                                                              service_name, data_type))

        self._api.bt_gatt_write_characteristic(service_name=service_name, char_name=char_name,
                                               data=data, data_type=data_type,
                                               acknowledgement=acknowledgement)
