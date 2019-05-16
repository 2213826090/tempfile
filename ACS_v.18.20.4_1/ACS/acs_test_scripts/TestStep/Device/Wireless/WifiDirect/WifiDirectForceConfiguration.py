"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to force WiFi Direct configuration
:since 15/01/2015
:author: jfranchx
"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase


class WifiDirectForceConfiguration(WifiDirectBase):
    """
    Implements force configuration test step for WiFi Direct
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        WifiDirectBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiDirectBase.run(self, context)

        assert self._pars.p2p_mode in [None, "GO", "CLI"], \
            "passed value (%s) is invalid at this stage" % self._pars.p2p_mode
        assert self._pars.p2p_frequency in [None, 2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462,
                                            2467, 2472, 5180, 5200, 5220, 5240], \
            "passed value (%s) is invalid at this stage" % self._pars.p2p_frequency

        # Force GO/CLI P2P mode
        if self._pars.p2p_mode is not None:
            self._api.set_wifi_direct_mode(self._pars.p2p_mode)

        # Force channel
        if self._pars.p2p_frequency is not None:
            self._api.set_wifi_direct_frequency(self._pars.p2p_frequency)
