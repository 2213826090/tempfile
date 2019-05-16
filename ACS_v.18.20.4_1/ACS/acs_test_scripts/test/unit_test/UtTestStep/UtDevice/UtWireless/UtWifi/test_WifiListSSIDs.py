"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:since 16/07/2014
:author: jfranchx
"""
import mock
import unittest

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiListSSIDs import WifiListSSIDs
from Core.TestStep.TestStepContext import TestStepContext


class WifiConnectTest(UTestTestStepBase):
    """
    List SSIDs test cases
    """

    LIST_SSIDS = ["SSID_ONE", "SSID_TWO", "SSID_THREE", "SSID_FOUR", "SSID_FIVE"]

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._list_ssids_return_values = None

    def test_list_ssids_ok(self):
        self._list_ssids_return_values = self.LIST_SSIDS
        sut = self._create_sut({"SAVE_AS": "LIST_SSIDS"})
        self._context = TestStepContext()
        sut.run(self._context)
        self.assertEqual(self._context.get_info("LIST_SSIDS"), str(self.LIST_SSIDS))

    def test_list_ssids_fail(self):
        self._list_ssids_return_values = None
        sut = self._create_sut({"SAVE_AS": "LIST_SSIDS"})
        self._context = TestStepContext()
        self._assert_run_throw_device_exception(sut, "No SSIDs found after scan !")

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiListSSIDs(None, None, test_step_pars, mock.Mock())
        self._method_connect = sut._api.wifi_connect
        sut._api.list_ssids.return_value = self._list_ssids_return_values
        return sut


if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
