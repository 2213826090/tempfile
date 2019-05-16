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
:since: 12/09/2014
:author: jfranchx
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiWaitForSSIDState import WifiWaitForSSIDState
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class WifiWaitForSSIDStateTest(UTestTestStepBase):
    """
    WifiWaitForSSIDState test cases
    """

    SSID_ALL = "SSID_ALL"
    SSID_CONNECTED = "SSID_CONNECTED"
    SSID_NOT_CONNECTED = "SSID_NOT_CONNECTED"
    SSID_REMEMBERED = "SSID_REMEMBERED"
    SSID_NOT_REMEMBERED = "SSID_NOT_REMEMBERED"
    SSID_VISIBLE = "SSID_VISIBLE"
    SSID_NOT_VISIBLE = "SSID_NOT_VISIBLE"
    SSID_ALL_FAIL = "SSID_ALL_FAIL"
    SSID_CONNECTED_FAIL = "SSID_CONNECTED_FAIL"
    SSID_NOT_CONNECTED_FAIL = "SSID_NOT_CONNECTED_FAIL"
    SSID_REMEMBERED_FAIL = "SSID_REMEMBERED_FAIL"
    SSID_NOT_REMEMBERED_FAIL = "SSID_NOT_REMEMBERED_FAIL"
    SSID_VISIBLE_FAIL = "SSID_VISIBLE_FAIL"
    SSID_NOT_VISIBLE_FAIL = "SSID_NOT_VISIBLE_FAIL"
    SSID_MULTIPLE = "SSID_MULTIPLE"
    TIMEOUT_ZERO = 0
    TIMEOUT_DEFAULT = 30

    def test_wait_for_ssid_all(self):
        """
        Test wait for ssid in all success
        """
        sut = self._create_sut({"SSID": self.SSID_ALL, "STATE": "ALL", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_ALL])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - ALL")

    def test_wait_for_ssid_all_fail(self):
        """
        Test wait for ssid in all fail
        """
        sut = self._create_sut({"SSID": self.SSID_ALL_FAIL, "STATE": "ALL", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_ALL])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("ALL",self.TIMEOUT_ZERO))

    def test_wait_for_ssid_connected_ok(self):
        """
        Test wait for ssid connected success
        """
        sut = self._create_sut({"SSID": self.SSID_CONNECTED, "STATE": "CONNECTED", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_CONNECTED])
        self._assert_run_succeeded_with_msg(sut,"All parameters checked : - CONNECTED")

    def test_wait_for_ssid_connected_fail(self):
        """
        Test wait for ssid connected fail
        """
        sut = self._create_sut({"SSID": self.SSID_CONNECTED_FAIL, "STATE": "CONNECTED", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_CONNECTED])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("CONNECTED",self.TIMEOUT_ZERO))

    def test_wait_for_ssid_not_connected_ok(self):
        """
        Test wait for ssid connected success
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_CONNECTED, "STATE": "NOT_CONNECTED", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_CONNECTED])
        self._assert_run_succeeded_with_msg(sut,"All parameters checked : - NOT_CONNECTED")

    def test_wait_for_ssid_not_connected_fail(self):
        """
        Test wait for ssid connected fail
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_CONNECTED_FAIL, "STATE": "NOT_CONNECTED", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_CONNECTED])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("NOT_CONNECTED",self.TIMEOUT_ZERO))

    def test_wait_for_ssid_remembered_ok(self):
        """
        Test ssid remembered success
        """
        sut = self._create_sut({"SSID": self.SSID_REMEMBERED, "STATE": "REMEMBERED", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_REMEMBERED])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - REMEMBERED")

    def test_wait_for_ssid_remembered_fail(self):
        """
        Test ssid remembered fail
        """
        sut = self._create_sut({"SSID": self.SSID_REMEMBERED_FAIL, "STATE": "REMEMBERED", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_REMEMBERED])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("REMEMBERED",self.TIMEOUT_ZERO))

    def test_wait_for_ssid_not_remembered_ok(self):
        """
        Test wait for ssid not remembered success
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_REMEMBERED, "STATE": "NOT_REMEMBERED", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_REMEMBERED])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - NOT_REMEMBERED")

    def test_wait_for_ssid_not_remembered_fail(self):
        """
        Test wait for ssid not remembered fail
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_REMEMBERED_FAIL, "STATE": "NOT_REMEMBERED", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_REMEMBERED])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("NOT_REMEMBERED",self.TIMEOUT_ZERO))

    def test_ssid_visible_ok(self):
        """
        Test ssid visible success
        """
        sut = self._create_sut({"SSID": self.SSID_VISIBLE, "STATE": "VISIBLE", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_VISIBLE])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - VISIBLE")

    def test_ssid_visible_fail(self):
        """
        Test ssid visible fail
        """
        sut = self._create_sut({"SSID": self.SSID_VISIBLE_FAIL, "STATE": "VISIBLE", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_VISIBLE])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("VISIBLE",self.TIMEOUT_ZERO))

    def test_ssid_not_visible_ok(self):
        """
        Test ssid not visible success
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_VISIBLE, "STATE": "NOT_VISIBLE", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_VISIBLE])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - NOT_VISIBLE")

    def test_ssid_not_visible_fail(self):
        """
        Test ssid not visible fail
        """
        sut = self._create_sut({"SSID": self.SSID_NOT_VISIBLE_FAIL, "STATE": "NOT_VISIBLE", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_VISIBLE])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("NOT_VISIBLE",self.TIMEOUT_ZERO))

    def test_ssid_multiple_parameters_ok(self):
        """
        Test ssid multiple parameters success
        """
        sut = self._create_sut({"SSID": self.SSID_MULTIPLE, "STATE": "ALL,REMEMBERED,CONNECTED,VISIBLE", "TIMEOUT": self.TIMEOUT_DEFAULT})
        self._simulate_list_ssids_returning(sut, [self.SSID_MULTIPLE, self.SSID_MULTIPLE, self.SSID_MULTIPLE, self.SSID_MULTIPLE])
        self._assert_run_succeeded_with_msg(sut, "All parameters checked : - ALL - REMEMBERED - CONNECTED - VISIBLE")

    def test_ssid_multiple_parameters_fail(self):
        """
        Test ssid multiple parameters success
        """
        sut = self._create_sut({"SSID": self.SSID_MULTIPLE, "STATE": "ALL,REMEMBERED,CONNECTED,VISIBLE", "TIMEOUT": self.TIMEOUT_ZERO})
        self._simulate_list_ssids_returning(sut, [self.SSID_NOT_VISIBLE, self.SSID_NOT_REMEMBERED, self.SSID_NOT_CONNECTED, self.SSID_NOT_VISIBLE])
        self._assert_run_throw_device_exception(sut, "Expected SSID State fail : %s - Timeout reached after %ss" % ("ALL,REMEMBERED,CONNECTED,VISIBLE",self.TIMEOUT_ZERO))


    # ------------------------------------------------------------------------------------------------------------------
    def _side_effect_list_ssids_response(self, interface, option):
        return self._responsed.pop(0)

    def _simulate_list_ssids_returning(self, sut, expected_list):
        self._responsed = expected_list
        sut._api.list_ssids.side_effect = self._side_effect_list_ssids_response
        return sut

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = WifiWaitForSSIDState(None, None, test_step_pars, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
