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
:summary: This file implements the step to disconnect from a wifi direct peer device.
:since: 2014-11-19
:author: Gangx, Lu

"""


from TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase


class WifiDirectDisconnect(WifiDirectBase):
    """
    Disconnects from a wifi direct peer device.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        WifiDirectBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._peer_device = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        WifiDirectBase.run(self, context)

        self._wifi_direct_name = self._pars.wifi_direct_name

        if self._pars.wifi_direct_name is None:
            self._wifi_direct_name = self._api.get_wifi_direct_interface()
            if self._wifi_direct_name == "":
                self._logger.debug("no interface p2p-wlan0-x to disconnect")
                return

        self._api.wifi_direct_disconnect(self._wifi_direct_name)