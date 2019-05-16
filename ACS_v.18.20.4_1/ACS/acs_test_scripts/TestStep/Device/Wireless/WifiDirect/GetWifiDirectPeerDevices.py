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
:summary: This file implements the step to get the list of wifi direct peer devices scanned.
:since: 2014-07-23
:author: emarchan

"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase

class GetWifiDirectPeerDevices(WifiDirectBase):
    """
    Gets the list of wifi direct peer devices scanned.
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

        state = self._pars.state

        assert state in ["all", "available", "invited", "connected"], "state value (%s) is invalid" % state

        value = self._api.get_wifi_direct_peer_devices(state)

        str_result = ""
        for cur_value in value:
            str_result = str_result + cur_value + '|'
        if str_result.endswith('|'):
            str_result = str_result[:-1]

        context.set_info(self._pars.save_as, str_result)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(str(str_result)) % self._pars.save_as
        self._logger.debug(self.ts_verdict_msg)

