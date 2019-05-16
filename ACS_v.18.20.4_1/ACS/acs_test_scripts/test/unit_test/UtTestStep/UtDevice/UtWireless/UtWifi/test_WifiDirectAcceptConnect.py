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
:since: 2014-08-06
:author: emarchan

"""
""

import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectAcceptConnect import WifiDirectAcceptConnect

PEER_MAC = "PEER_DEV_MAC"
class WifiDirectAcceptConnectTest(UtWifiDirectBase):
    """
    Connects to a wifi direct peer device.
    """

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)

    def test_wifidirect_accept_connect_ok(self):
        peer_mac = "02:14:22:56:55:AA"
        sut = self._create_sut({PEER_MAC:peer_mac})
        sut._api._exec.return_value = "OK"
        sut.run(self._context)

        sut._api.wifi_direct_accept_connection.assert_called_once_with(peer_mac)

    def test_wifidirect_accept_connect_bad_param_ko(self):
        peer_mac = "toto"
        sut = self._create_sut({PEER_MAC:peer_mac})
        self._assert_run_throw_config_exception(sut, "Invalid MAC address provided for incoming_peer")


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDirectAcceptConnect(None, None, test_step_pars, mock.Mock())
        return sut
