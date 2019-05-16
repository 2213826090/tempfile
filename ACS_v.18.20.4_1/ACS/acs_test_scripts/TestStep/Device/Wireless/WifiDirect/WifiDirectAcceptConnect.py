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
:summary: This file implements the step to grant a connection request from a peer device.
:since: 2014-08-06
:author: emarchan

"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.NetworkingUtilities import is_valid_mac_address

class WifiDirectAcceptConnect(WifiDirectBase):
    """
    Grants a connection request from a peer device.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        WifiDirectBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._accept_connection = 0
        self._regulatory_domain = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiDirectBase.run(self, context)

        if (self._pars.p2p_com_freq in Constants.LIST_US_2G_CHANNELS) or (self._pars.p2p_com_freq in Constants.LIST_US_5G_CHANNELS):
            self._regulatory_domain = "US"

        if (self._pars.p2p_com_freq in Constants.LIST_FR_2G_CHANNELS) or (self._pars.p2p_com_freq in Constants.LIST_FR_5G_CHANNELS):
            self._regulatory_domain = "FR"

        if self._regulatory_domain != None:
            self._nw_api.set_regulatorydomain(self._regulatory_domain)

        incoming_peer = self._pars.peer_dev_mac
        valid_mac = is_valid_mac_address(incoming_peer)
        if valid_mac is not True:
            msg = "Invalid MAC address provided for incoming_peer (%s)" % incoming_peer
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._api.wifi_direct_accept_connection(incoming_peer)
