"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: unit test
:since:13/01/2014
:author: fbongiax
"""

import mock

from unit_test_fwk.UtDevice.UtDeviceModel.DeviceHelpers import DeviceHelpers
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import LocalConnectivity
from unit_test_fwk.UTestBase import UTestBase


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class BaseTestCase(UTestBase):
    """
    Base class for LocalConnectivity test cases
    """

    DEVICE_CAPABILITIES_CWS = ["bluetooth", "wifi", "nfc"]
    BDADDRESS = "11:22:33:44:55:66"

    def setUp(self):
        """
        Setup
        """
        self._device_helpers = DeviceHelpers()
        self._device_helpers.start_device_mock()

        UTestBase.setUp(self)
        mock_device = mock.MagicMock(device_capabilities=BaseTestCase.DEVICE_CAPABILITIES_CWS)
        self._sut = LocalConnectivity(mock_device)
        self._sut._internal_exec_v2 = mock.MagicMock(name="_internal_exec_v2")
        self._sut._logger = mock.Mock()

    # tearDown are executed after each test
    def tearDown(self):
        # Stop mocking engine
        self._device_helpers.stop_device_mock()

    def _assert_bluetooth_api_was_called_with(self, method, args=None, timeout=None):
        """
        Assert that _internal_exec_v2 was called with the given method and args
        """
        self._assert_internal_exec_v2_was_called(LocalConnectivity._BLUETOOTH_MODULE, method, args, timeout)
