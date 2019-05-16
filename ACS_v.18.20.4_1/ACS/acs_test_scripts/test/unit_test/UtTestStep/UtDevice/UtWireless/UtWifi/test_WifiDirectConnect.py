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
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectConnect import WifiDirectConnect

PEER_DEVICE_PARAM = "PEER_DEVICE"
TIMEOUT_PARAM = "TIMEOUT"
GO_INTENT_PARAM = "GO_INTENT"
P2P_COM_FREQ_PARAM = "P2P_COM_FREQ"


class WifiDirectConnectTest(UtWifiDirectBase):
    """
    Connects to a wifi direct peer device.
    """

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)

    def test_wifidirect_connect_ok(self):
        timeout = 20
        go_intent = 8
        forced_freq = 0
        sut = self._create_sut({PEER_DEVICE_PARAM: self.DEFAULT_PEER_NAME, TIMEOUT_PARAM: timeout,
                                GO_INTENT_PARAM: go_intent, P2P_COM_FREQ_PARAM: forced_freq})
        sut.run(self._context)
        sut._api.wifi_direct_connect.assert_called_once_with(self.DEFAULT_PEER_NAME, timeout, go_intent, forced_freq)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDirectConnect(None, None, test_step_pars, mock.Mock())
        return sut
