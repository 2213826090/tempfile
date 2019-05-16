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
:since: 2014-11-19
:author: Gangx, Lu

"""

import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from TestStep.Device.Wireless.WifiDirect.WifiDirectDisconnect import WifiDirectDisconnect

WIFI_DIRECT_NAME_PARAM = "WIFI_DIRECT_NAME"
class WifiDirectDisconnectTest(UtWifiDirectBase):
    """
    Disconnects from a wifi direct peer device.
    """

    INTERFACE_RETURN="p2p-wlan0-0"

    def _return_get_wifi_direct_interface(self):
        """
        Stub method
        """
        return self._wifi_direct_interface

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)
        self._wifi_direct_interface = self.INTERFACE_RETURN

    def test_wifidirect_disconnect_ok(self):
        wifi_direct_name="ACS_DUT2"
        sut = self._create_sut({WIFI_DIRECT_NAME_PARAM:wifi_direct_name})
        sut.run(self._context)
        sut._api.wifi_direct_disconnect.assert_called_once_with(wifi_direct_name)

    def test_wifidirect_disconnect_without_arg_ok(self):
        sut = self._create_sut()
        sut.run(self._context)
        sut._api.get_wifi_direct_interface.assert_called_once_with()

    def test_wifidirect_disconnect_without_arg_but_interface_ok(self):
        sut = self._create_sut()
        sut._api.get_wifi_direct_interface=self._return_get_wifi_direct_interface
        sut.run(self._context)
        sut._api.wifi_direct_disconnect.assert_called_once_with(self.INTERFACE_RETURN)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDirectDisconnect(None, None, test_step_pars, mock.Mock())
        return sut
