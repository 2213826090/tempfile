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
:summary: This file implements the step to connect to a wifi direct peer device.
:since: 2014-08-01
:author: tnizan

"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase

class WifiDirectInvite(WifiDirectBase):
    """
    Invite a peer to join a group or to reinvoke a persistent group.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        WifiDirectBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._peer_device = None
        self._force_freq = 0
        self._regulatory_domain = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiDirectBase.run(self, context)
        self._peer_device = self._pars.peer_device

        if self._pars.go_intent in Constants.LIST_GO_INTENT_VALUES:
            self._go_intent = self._pars.go_intent

        if (self._pars.p2p_com_freq in Constants.LIST_US_2G_CHANNELS) or (self._pars.p2p_com_freq in Constants.LIST_US_5G_CHANNELS):
            self._regulatory_domain = "US"
            self._force_freq = self._pars.p2p_com_freq

        if (self._pars.p2p_com_freq in Constants.LIST_FR_2G_CHANNELS) or (self._pars.p2p_com_freq in Constants.LIST_FR_5G_CHANNELS):
            self._regulatory_domain = "FR"
            self._force_freq = self._pars.p2p_com_freq

        if self._regulatory_domain != None:
            self._nw_api.set_regulatorydomain(self._regulatory_domain)


        self._api.wifi_direct_invite(self._peer_device, self._force_freq)
