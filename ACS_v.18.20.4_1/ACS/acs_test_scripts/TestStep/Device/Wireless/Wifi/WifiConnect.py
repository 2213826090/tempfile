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

:organization: INTEL NDG SW
:summary: This file implements a Test Step for Wifi Connect
:since 26/03/2014
:author: floeselx
"""
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase


class WifiConnect(WifiBase):
    """
    Implements the Ping Test Step for Wifi
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)

        ssid = str(self._pars.ssid)
        timeout = int(self._pars.timeout)
        self._logger.info("Wifi connection to %s" % ssid)

        self._api.wifi_connect(ssid=ssid, check_connection=False)

        if timeout > 0:
            self._logger.info("Check connection state (timeout {0}s)".format(timeout))
            self._api.check_connection_state(ssid=ssid, timeout=timeout)
