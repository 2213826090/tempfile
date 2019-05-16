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

from acs_test_scripts.TestStep.Device.Wireless.BT.BtWaitPairing import BtWaitPairing
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class WaitPairingTest(UTestTestStepBase):
    """
    WaitPairing test cases
    """
    DEFAULT_TIMEOUT_SECS = 20

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

        self._sut = self._create_sut({"BDADDR": "00:00:00:00:00:00"})

    def test_all_default_args_ok(self):
        """
        Test pairing succeed
        """
        self._assert_run_succeeded(self._sut)
        self._sut._api.wait_for_pairing.assert_called_with('00:00:00:00:00:00', 'off', 1, '0000', 0, 20)

    def test_api_args_are_correct(self):
        """
        Test pairing args are passed to the API correctly
        """
        self._sut = self._create_sut({"BDADDR": "00:00:00:00:00:00", "UNPAIR_FIRST": True, "ACCEPT_PAIRING": False,
                                      "PIN_CODE": "1234", "PASS_KEY": 12, "TIMEOUT": 55})
        self._sut.run(self._context)
        self._sut._api.wait_for_pairing.assert_called_with('00:00:00:00:00:00', 'on', 0, '1234', 12, 55)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars):
        """
        Create the SUT with only test step pars
        """

        sut = BtWaitPairing(None, None, test_step_pars, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
