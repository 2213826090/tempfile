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
:summary: This file implements the step to get a Wifi Direct device address from its name.
:since: 2014-08-01
:author: emarchan

"""

from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase
from acs_test_scripts.Utilities.NetworkingUtilities import is_valid_mac_address

class GetWifiDirectMacAddress(WifiDirectBase):
    """
    Gets the device Wifi Direct mac address.
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

        value = self._api.get_wifi_direct_mac_address()

        if is_valid_mac_address(value) is not True:
            self._raise_device_exception("Invalid mac address detected %s" % (value))

        context.set_info(self._pars.save_as, value)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(str(value)) % self._pars.save_as
        self._logger.debug(self.ts_verdict_msg)

