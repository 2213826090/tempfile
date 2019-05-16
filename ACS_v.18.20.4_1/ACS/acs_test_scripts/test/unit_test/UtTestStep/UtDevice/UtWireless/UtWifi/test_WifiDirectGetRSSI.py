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
:since: 19/01/2015
:author: jfranchx

"""
import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from TestStep.Device.Wireless.WifiDirect.WifiDirectGetRSSI import WifiDirectGetRSSI
from Core.TestStep.TestStepContext import TestStepContext


class WifiDirectGetRSSITest(UtWifiDirectBase):
    """
    Gets the WiFi direct rssi unit test.
    """

    P2P_INTERFACE = "p2p-wlan0-0"
    WIFI_DIRECT_RSSI = "WIFI_DIRECT_RSSI"
    P2P_RSSI_VALUE = "-40"

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)
        self._context = TestStepContext()

    def test_get_wifi_direct_rssi_call_ok(self):
        sut = self._create_sut({"INTERFACE": self.P2P_INTERFACE, "SAVE_WIFI_DIRECT_RSSI": self.WIFI_DIRECT_RSSI})
        sut.run(self._context)
        sut._api.get_wifi_direct_rssi.assert_called_once_with(self.P2P_INTERFACE)

    def test_get_wifi_direct_rssi_save_ok(self):
        sut = self._create_sut({"INTERFACE": self.P2P_INTERFACE, "SAVE_WIFI_DIRECT_RSSI": self.WIFI_DIRECT_RSSI})
        sut.run(self._context)
        self.assertEqual(self.P2P_RSSI_VALUE, self._context.get_info(self.WIFI_DIRECT_RSSI))


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDirectGetRSSI(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_direct_rssi.return_value = self.P2P_RSSI_VALUE

        return sut
