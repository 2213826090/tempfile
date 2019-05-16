"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: unit test
:since: 19/09/2014
:author: jfranchx
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.Cellular.CheckRatWithPreferredNetwork import CheckRatWithPreferredNetwork
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class CheckRatWithPreferredNetworkTest(UTestTestStepBase):
    """
    CheckRatWithPreferredNetwork test cases
    """

    def test_check_rat_with_preferred_network_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": "4G_PREF", "TIMEOUT": 30})
        sut.run(self._context)
        self._method_connect.assert_called_with('4G_PREF', 30)

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = CheckRatWithPreferredNetwork(None, None, test_step_pars, mock.Mock())
        self._method_connect = sut._modem_api.check_rat_with_pref_network
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
