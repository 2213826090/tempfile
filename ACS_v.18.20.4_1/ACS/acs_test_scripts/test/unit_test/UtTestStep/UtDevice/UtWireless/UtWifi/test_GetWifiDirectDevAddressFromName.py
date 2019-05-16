"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG
:summary: unit test
:since: 2014-08-05
:author: emarchan

"""
import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.GetWifiDirectDevAddressFromName import GetWifiDirectDevAddressFromName
from Core.TestStep.TestStepContext import TestStepContext

NAME_PARAM = "WIFI_DIRECT_NAME"
class GetWifiDirectDevAddressFromNameTest(UtWifiDirectBase):
    """
    Gets a Wifi Direct device address from its name.
    """

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)
        self._context = TestStepContext()

    def test_get_wifi_direct_dev_address_from_name_call_ok(self):
        sut = self._create_sut({NAME_PARAM: self.DEFAULT_PEER_NAME, self.SAVE_AS_PARAM:self.DEFAULT_SAVE_VAR})
        sut._api.get_wifi_direct_dev_address_from_name.return_value = self.DEFAULT_PEER_ADDR

        sut.run(self._context)

        sut._api.get_wifi_direct_dev_address_from_name.assert_called_once_with(self.DEFAULT_PEER_NAME)

    def test_get_wifi_direct_dev_address_from_name_save_ok(self):
        sut = self._create_sut({NAME_PARAM: self.DEFAULT_PEER_NAME, self.SAVE_AS_PARAM:self.DEFAULT_SAVE_VAR})
        sut._api.get_wifi_direct_dev_address_from_name.return_value = self.DEFAULT_PEER_ADDR

        sut.run(self._context)

        self.assertEqual(self.DEFAULT_PEER_ADDR, self._context.get_info(self.DEFAULT_SAVE_VAR))

    def test_get_wifi_direct_dev_address_wrong_mac_ko(self):
        sut = self._create_sut({NAME_PARAM: self.DEFAULT_PEER_NAME, self.SAVE_AS_PARAM:self.DEFAULT_SAVE_VAR})
        sut._api.get_wifi_direct_dev_address_from_name.return_value = "blabla"

        self._assert_run_throw_device_exception(sut, "Invalid value detected for %s" % self.DEFAULT_PEER_NAME)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetWifiDirectDevAddressFromName(None, None, test_step_pars, mock.Mock())
        return sut
