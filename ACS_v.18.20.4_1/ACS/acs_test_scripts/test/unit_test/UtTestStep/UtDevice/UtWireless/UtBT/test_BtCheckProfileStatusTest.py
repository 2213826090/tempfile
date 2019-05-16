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
:since:08/01/2014
:author: fbongiax
"""
import unittest2 as unittest
import mock
from acs_test_scripts.Device.UECmd.UECmdTypes import BtConState

from acs_test_scripts.TestStep.Device.Wireless.BT.BtCheckProfileStatus import BtCheckProfileStatus
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants


class BtCheckProfileStatusTest(UTestTestStepBase):
    """
    BtCheckProfileStatus test cases
    """
    _COMMON_ARGS = {"DEVICE": "PHONE1", "PROFILE": "A2DP", "BDADDR": "11:22:33:44:55:66"}

    def setUp(self):
        """
        Setup
        """
        UTestTestStepBase.setUp(self)
        self._count = 0

    def test_ok(self):
        """
        Test check succeeds
        """
        # pylint: disable=W0212
        sut = self._create_sut()
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.CONNECTED]

        self._assert_run_succeeded(sut)

    def test_fail_if_connected_fail(self):
        """
        Test check fails as per FAIL_IF argument
        """
        # pylint: disable=W0212
        sut = self._create_sut({"FAIL_IF": Constants.PROFILE_CONNECTED})
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.CONNECTED]

        self._assert_run_throw_device_exception(sut, "Profile connection state is CONNECTED")

    def test_fail_if_connected_pass(self):
        """
        Test check passes as FAIL_IF is not verified
        """
        # pylint: disable=W0212
        sut = self._create_sut({"FAIL_IF": Constants.PROFILE_CONNECTED})
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.DISCONNECTED]

        self._assert_run_succeeded(sut)

    def test_fail_if_disconnected_fail(self):
        """
        Test check fails as per FAIL_IF argument
        """
        # pylint: disable=W0212
        sut = self._create_sut({"FAIL_IF": Constants.PROFILE_DISCONNECTED})
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.DISCONNECTED]

        self._assert_run_throw_device_exception(sut, "Profile connection state is DISCONNECTED")

    def test_fail_if_disconnected_pass(self):
        """
        Test check passes as FAIL_IF is not verified
        """
        # pylint: disable=W0212
        sut = self._create_sut({"FAIL_IF": Constants.PROFILE_DISCONNECTED})
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.CONNECTED]

        self._assert_run_succeeded(sut)

    def test_time_out_expires(self):
        """
        Test check doesn't retrieve the right status
        """
        # pylint: disable=W0212
        sut = self._create_sut({"FAIL_IF": Constants.PROFILE_DISCONNECTED})

        # force sut timing to 0 not to waste time waiting
        sut._sleep_time = 0
        sut._timeout = 0
        sut._api.get_bt_connection_state.return_value = BtConState.d[BtConState.CONNECTING]
        self._assert_run_throw_device_exception(sut, "Expected status hasn't been reached before timeout")

    def test_pending_status(self):
        """
        Test check doesn't retrieve the right status
        """
        # pylint: disable=W0212
        sut = self._create_sut()

        # force sut timing to 0 not to waste time waiting
        sut._sleep_time = 0
        sut._api.get_bt_connection_state = self._stub_get_bt_connection_state

        self._assert_run_succeeded(sut)

    def _stub_get_bt_connection_state(self, addr, profile):
        # pylint: disable=W0613
        """
        Stub method
        """
        self._count += 1
        if self._count < 5:
            return BtConState.d[BtConState.CONNECTING]
        return BtConState.d[BtConState.CONNECTED]

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        args = self._COMMON_ARGS if not test_step_pars else dict(self._COMMON_ARGS.items() + test_step_pars.items())
        sut = BtCheckProfileStatus(None, None, args, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
