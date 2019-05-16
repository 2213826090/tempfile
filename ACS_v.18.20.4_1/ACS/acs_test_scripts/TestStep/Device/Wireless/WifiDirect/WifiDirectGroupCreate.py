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

:organization: INTEL NDG
:summary: This file implements the step to create a WiFi Direct Autonomous Group for Linux devices.
:since: 2014-11-13
:author: agoeax

"""

from TestStep.Device.Wireless.WifiDirect.WifiDirectBase import WifiDirectBase

class WifiDirectGroupCreate(WifiDirectBase):
    """
    Create Autonomous Group for P2P connections on Linux
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

        assert self._pars.group_freq in ["2412", "2417", "2422", "2427", "2432", "2437", "2442", "2447", "2452",
                                         "2457", "2462"], "Wireless frequency %s is not acceptable" % self._pars.group_freq

        self._logger.info("Creating P2P Autonomous Group on frequency %s" % self._pars.group_freq)
        pin_code=self._api.wifi_direct_create_group(self._pars.group_freq, self._pars.authentication_type)

        if self._pars.authentication_type == "WPS-PIN" and pin_code !="" and pin_code.isdigit():
            context.set_info(self._pars.save_as, pin_code)
            self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(pin_code) % self._pars.save_as
            self._logger.debug(self.ts_verdict_msg)


