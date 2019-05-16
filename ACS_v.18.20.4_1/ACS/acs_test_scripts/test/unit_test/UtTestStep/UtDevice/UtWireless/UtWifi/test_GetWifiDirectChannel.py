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

import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.GetWifiDirectChannel import GetWifiDirectChannel
from Core.TestStep.TestStepContext import TestStepContext

SAVE_AS = "report"
EXP_CHANNEL = 1
DEFAULT_FREQ = 2412
class GetWifiDirectChannelTest(UtWifiDirectBase):
    """
    Connects to a wifi direct peer device.
    """

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)
        self._context = TestStepContext()

    def test_wifidirect_channel_ok(self):
        sut = self._create_sut({"SAVE_WIFI_DIRECT_CHANNEL":SAVE_AS})
        sut._api.get_wifi_direct_channel_freq.return_value = DEFAULT_FREQ
        sut.run(self._context)

        self.assertEqual(EXP_CHANNEL, self._context.get_info(SAVE_AS + ":CHANNEL"))
        self.assertEqual(DEFAULT_FREQ, self._context.get_info(SAVE_AS + ":FREQUENCY"))

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetWifiDirectChannel(None, None, test_step_pars, mock.Mock())
        return sut
