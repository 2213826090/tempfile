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
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.BtPair import BtPair
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class PairTest(UTestTestStepBase):
    """
    GetAddress test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

        self._sut = self._create_sut({"BDADDR": "00:00:00:00:00:00"})

    def test_pair_ok(self):
        """
        Test pairing succeed
        """
        # pylint: disable=E1101
        self._set_pair_result(BT_BOND_STATE.BOND_BONDED, "")

        self._assert_run_succeeded_with_msg(self._sut, "Pairing with device succeeded")

    def test_pair_fail(self):
        """
        Test device failed
        """
        # pylint: disable=E1101
        self._set_pair_result(BT_BOND_STATE.BOND_NONE, "")
        self._assert_run_throw_device_exception(self._sut, "Pairing with device failed")

    def test_api_args_are_correct(self):
        """
        Test PairingConfig is created to manage pairing args
        """
        self._sut = self._create_sut({"BDADDR": "00:00:00:00:00:00", "UNPAIR_FIRST": True, "ACCEPT_PAIRING": True,
                                      "PIN_CODE": "1234", "PASS_KEY": 12})
        # pylint: disable=E1101
        self._set_pair_result(BT_BOND_STATE.BOND_BONDED, "")

        self._sut.run(self._context)

        self._sut._api.pair_to_device.assert_called_with('00:00:00:00:00:00', 'on', 1, '1234', 12)


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = BtPair(None, None, test_step_pars, mock.Mock())

        return sut

    def _set_pair_result(self, state, pin):
        """
        Set result info returned by paring api
        """
        self._sut._api.pair_to_device.return_value = (state, pin)

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
