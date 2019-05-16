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

:organization: INTEL MCG
:summary: unit test
:since 15/01/2015
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectForceConfiguration import WifiDirectForceConfiguration


class WifiDirectForceConfigurationTest(UTestTestStepBase):
    """
    WiFi Direct force configuration test
    """

    P2P_MODE_GO = "GO"
    P2P_MODE_CLI = "CLI"
    P2P_FREQUENCY_2_4GHZ = 2412
    P2P_FREQUENCY_5GHZ = 5180

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

    def test_set_mode_go_ok(self):
        sut = self._create_sut({"P2P_MODE": self.P2P_MODE_GO})
        sut.run(self._context)
        sut._api.set_wifi_direct_mode.assert_called_once_with(self.P2P_MODE_GO)

    def test_set_mode_cli_ok(self):
        sut = self._create_sut({"P2P_MODE": self.P2P_MODE_CLI})
        sut.run(self._context)
        sut._api.set_wifi_direct_mode.assert_called_once_with(self.P2P_MODE_CLI)

    def test_set_frequency_2_4ghz_ok(self):
        sut = self._create_sut({"P2P_FREQUENCY": self.P2P_FREQUENCY_2_4GHZ})
        sut.run(self._context)
        sut._api.set_wifi_direct_frequency.assert_called_once_with(self.P2P_FREQUENCY_2_4GHZ)

    def test_set_frequency_5ghz_ok(self):
        sut = self._create_sut({"P2P_FREQUENCY": self.P2P_FREQUENCY_5GHZ})
        sut.run(self._context)
        sut._api.set_wifi_direct_frequency.assert_called_once_with(self.P2P_FREQUENCY_5GHZ)

    def test_set_mode_and_frequency_ok(self):
        sut = self._create_sut({"P2P_MODE": self.P2P_MODE_GO, "P2P_FREQUENCY": self.P2P_FREQUENCY_5GHZ})
        sut.run(self._context)
        sut._api.set_wifi_direct_mode.assert_called_once_with(self.P2P_MODE_GO)
        sut._api.set_wifi_direct_frequency.assert_called_once_with(self.P2P_FREQUENCY_5GHZ)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDirectForceConfiguration(None, None, test_step_pars, mock.Mock())
        return sut
