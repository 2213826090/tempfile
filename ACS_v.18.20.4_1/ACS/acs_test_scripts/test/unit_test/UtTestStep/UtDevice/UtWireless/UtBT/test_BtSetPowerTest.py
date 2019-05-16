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
:since:20/12/2013
:author: fbongiax
"""
import unittest2 as unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.BtSetPower import BtSetPower
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class SetPowerTest(UTestTestStepBase):
    """
    SetPower test cases
    """

    def _stub_get_bt_power_status(self):
        """
        Stub method
        """
        response = self._resp[self._index]
        self._index = self._index + 1
        return response

    def _set_api_responses(self, resp):
        """
        Set API set_power / get_power responses
        """
        self._resp = resp
        self._index = 0

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

        self._resp = None
        self._index = 0

    def test_set_power_ok(self):
        """
        Test pairing succeed
        """
        sut = self._create_sut({"POWER": "ON"})
        self._set_api_responses([str(BT_STATE.STATE_ON)])
        self._assert_run_succeeded(sut)

    def test_set_power_fail(self):
        """
        Test pairing succeed
        """
        sut = self._create_sut({"POWER": "ON"})
        self._set_api_responses([str(BT_STATE.STATE_OFF)])
        self._assert_run_throw_device_exception(sut, "set BT ON failure")

    def test_set_power_off_ok(self):
        """
        Test pairing succeed
        """
        sut = self._create_sut({"POWER": "OFF"})
        self._set_api_responses([str(BT_STATE.STATE_OFF)])
        self._assert_run_succeeded(sut)

    def test_set_power_off_fail(self):
        """
        Test pairing succeed
        """
        sut = self._create_sut({"POWER": "OFF"})
        self._set_api_responses([str(BT_STATE.STATE_ON)])
        self._assert_run_throw_device_exception(sut, "set BT OFF failure")

    def test_set_power_multi_ok(self):
        """
        Test pairing succeed
        """
        sut = self._create_sut({"POWER": "ON, OFF, ON, OFF"})
        self._set_api_responses([str(BT_STATE.STATE_ON), str(BT_STATE.STATE_OFF), str(BT_STATE.STATE_ON), str(BT_STATE.STATE_OFF)])
        self._assert_run_succeeded(sut)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = BtSetPower(None, None, test_step_pars, mock.Mock())
        sut._api.get_bt_power_status = self._stub_get_bt_power_status

        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
