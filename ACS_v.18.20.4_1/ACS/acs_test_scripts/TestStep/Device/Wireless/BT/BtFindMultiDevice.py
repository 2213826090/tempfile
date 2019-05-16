"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

@organization: INTEL ICDG ACP
@summary: This file implements a Test Step that scans and tries to find the devices identified by the list [DEVICES_TO_FIND]
@since 29/05/2015
@author: emarchan
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtFindMultiDevice(BtBase):
    """
    Implements the test step to scan device with another device
    Attributes:
        DEVICE: @see BtTestStepBase
        DEVICES_TO_FIND (list of string): the BT names or addresses of the devices to find
        MUST_FIND (string): all the BT devices should be find or not
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        BtBase.run(self, context)
        assert self._pars.must_find in [True, False], \
            "MUST_FIND (%s) value should have been checked by the framework" % self._pars.must_find
        all_devices_to_find = self._pars.devices_to_find.split(',')
        devices_found = self._api.bt_find_multi_devices(all_devices_to_find, self._pars.tries)
        if not devices_found and self._pars.must_find is True:
            msg = "At list one device is not found"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif devices_found and self._pars.must_find is False:
            msg = "At list one device is not found and it shouldn't"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
