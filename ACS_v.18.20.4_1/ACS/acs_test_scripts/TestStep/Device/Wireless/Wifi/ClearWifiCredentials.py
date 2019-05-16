"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to remove certificates installed on the device
:since: 16/06/2015
:author: razzix
"""

from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase

class ClearWifiCredentials(WifiBase):
    """
    Remove WIFI Certificates from device and remove Pin CODE if necessary
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        WifiBase.run(self, context)

        # remove wifi certificates
        self._api.remove_wpa_certificates(self._pars.credential_password, self._pars.pin_code_removal)
