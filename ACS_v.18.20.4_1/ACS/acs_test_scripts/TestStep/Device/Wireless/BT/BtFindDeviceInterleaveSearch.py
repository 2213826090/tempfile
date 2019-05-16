"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements a Test Step that scan a device with another device
@since 13/03/2014
@author: jfranchx
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtFindDeviceInterleaveSearch(BtBase):
    """
    Implements the test step to scan device with another device
    Attributes:
        DEVICE: @see BtTestStepBase
        DEVICE_TO_FIND (string): the BT name or address of the device to find
        MUST_FIND (string): the BT device should be find or not
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
        try:
            device_found = self._api.bt_find_device_interleave_search(self._pars.device_to_find)
            if not device_found and self._pars.must_find is True:
                msg = "Device %s not found" % self._pars.device_to_find
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            elif device_found and self._pars.must_find is False:
                msg = "Device %s found" % self._pars.device_to_find
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except DeviceException as acs_exception:
            if self._pars.must_find is None:
                self._logger.debug("Error/Timeout on scan devices")
            else:
                raise
