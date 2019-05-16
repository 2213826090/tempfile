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
:summary: unit test
:since 16/07/2014
:author: jfranchx
"""
import mock
import unittest

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiConnect import WifiConnect


class WifiConnectTest(UTestTestStepBase):
    """
    Connect test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._initialize_params()

    def _initialize_params(self):
        self._state_called = False
        self._state_ssid = None
        self._state_timeout = None

    def _check_connection_state_called(self, ssid, timeout):
        self._state_called = True
        self._state_ssid = ssid
        self._state_timeout = timeout
        return True

    def test_connect_ok(self):
        self._initialize_params()
        sut = self._create_sut({"SSID": "TEST_SSID", "TIMEOUT": "10"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with(ssid='TEST_SSID', check_connection=False)
        self.assertIs(self._state_called, True)
        self.assertIs(self._state_ssid, 'TEST_SSID')
        self.assertIs(self._state_timeout, 10)

    def test_connect_timeout_none_ok(self):
        self._initialize_params()
        sut = self._create_sut({"SSID": "TEST_SSID", "TIMEOUT": "0"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with(ssid='TEST_SSID', check_connection=False)
        self.assertIs(self._state_called, False)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiConnect(None, None, test_step_pars, mock.Mock())
        self._method_connect = sut._api.wifi_connect
        sut._api.check_connection_state = self._check_connection_state_called
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
