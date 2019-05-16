"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: unit test
:since 10/09/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Common.IPMgmt.GetMacAddress import GetMacAddress
from UtilitiesFWK.Utilities import TestConst


class GetMacAddressTest(UTestTestStepBase):
    """
    GetMacAddress test cases
    """

    WIFI_MAC_ADDRESS = "AA:BB:CC:DD:EE:FF"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._dest_var = "mac_address"
        self._context = TestStepContext()

    def test_get_mac_address_ok(self):
        sut = self._create_sut({"INTERFACE": "wlan0"})

        value_expected = self.WIFI_MAC_ADDRESS
        self._method.return_value = self.WIFI_MAC_ADDRESS

        sut.run(self._context)
        value_got = self._context.get_info(self._dest_var)
        self._method.assert_called_once_with("wlan0")
        self.assertEqual(value_expected, value_got)

    def test_get_mac_address_fail(self):

        sut = self._create_sut({"INTERFACE": "invalid_interface"})
        self._method.return_value = self.WIFI_MAC_ADDRESS
        self._assert_run_throw_config_exception(sut, "Only wlan0 interface is supported - can't get invalid_interface MAC address")

    # pylint: disable=W0212
    def _create_sut(self, args={}):
        """
        Create the SUT with only test step pars
        """
        test_args = {"MAC_ADDR": self._dest_var}
        test_args.update(args)
        sut = GetMacAddress(None, None, test_args, mock.Mock())
        self._method = sut._api.get_interface_mac_addr
        return sut
