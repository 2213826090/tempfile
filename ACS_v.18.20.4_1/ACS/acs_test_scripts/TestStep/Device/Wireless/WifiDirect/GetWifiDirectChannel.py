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
:summary: This file implements the step to get the current channel (number and frequency) used for WiFi Direct.
:since: 2014-12-15
:author: emarchan

"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase
from Utilities.NetworkingUtilities import AcsWifiFrequencies

class GetWifiDirectChannel(WifiDirectBase):
    """
    Gets the current channel (number and frequency) used for WiFi Direct.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        WifiDirectBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        WifiDirectBase.run(self, context)

        dest_variable = self._pars.save_wifi_direct_channel

        cur_freq = self._api.get_wifi_direct_channel_freq()
        cur_channel = AcsWifiFrequencies.get_wifi_channel_from_frequency(str(cur_freq))

        context.set_nested_info([dest_variable, "FREQUENCY"], cur_freq)
        context.set_nested_info([dest_variable, "CHANNEL"], cur_channel)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}\n".format(str(cur_freq)) % "FREQUENCY"
        self.ts_verdict_msg += "%s stored as {0}\n".format(str(cur_channel)) % "CHANNEL"
        self._logger.debug(self.ts_verdict_msg)
