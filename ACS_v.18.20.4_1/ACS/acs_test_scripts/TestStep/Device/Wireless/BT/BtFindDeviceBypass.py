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
@summary: This file implements a Test Step that scan a device with another device with a bypass for
the known issue that the Scan capability is failing on Android Lollipop
@since 11/02/2015
@author: mmaraci
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.Utilities.BluetoothConnectivity import BluetoothConnectivity


class BtFindDeviceBypass(BtBase):
    """
    Implements the test step to scan device with another device
    Attributes:
        DEVICE: @see BtTestStepBase
        DEVICE_TO_FIND (string): the BT name or address of the device to find
        MUST_FIND (string): the BT device should be find or not
    """
    STR_LOCAL_CONN = "LocalConnectivity"
    STR_PHONE2 = "PHONE2"

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        BtBase.run(self, context)
        self._bt_obj = BluetoothConnectivity(self._device, 15)
        self._phone2 = DeviceManager().get_device(BtFindDeviceBypass.STR_PHONE2)
        self._bt_api2 = self._phone2.get_uecmd(BtFindDeviceBypass.STR_LOCAL_CONN)
        assert self._pars.must_find in [True, False], \
            "MUST_FIND (%s) value should have been checked by the framework" % self._pars.must_find
        self._bt_obj.search_for_device_until_found(self._api, self._bt_api2, self._pars.device_to_find, self._pars.must_find)
