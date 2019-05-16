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

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.TestStep.Device.Wireless.BT.BtPairingConfig import PairingConfig
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class PairingConfigTest(UTestTestStepBase):
    """
    PairingConfig test cases
    """

    def test_unpair_first_not_provided(self):
        """
        Test
        """
        sut = self._create_sut(None)
        self.assertEqual("off", sut.reconnect, "when unpair_firt arg is not passed it must default to off")

    def test_unpair_first_true(self):
        """
        Test
        """
        sut = self._create_sut({"UNPAIR_FIRST": True})
        self.assertEqual("on", sut.reconnect, "when unpair_first arg is True it must be set to on")

    def test_unpair_first_false(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut({"UNPAIR_FIRST": False})
        self.assertEqual("off", sut.reconnect, "when unpair_first arg is False it must be set to off")

    def test_pin_code_not_provided(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut(None)
        self.assertEqual("0000", sut.pincode, "when pin_code arg not provided it must default to 0000")

    def test_pin_code_provided(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut({"PIN_CODE": "1234"})
        self.assertEqual("1234", sut.pincode, "pin_code must be 1234")

    def test_pass_key_not_provided(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut(None)
        self.assertEqual(0, sut.passkey, "when pass_key arg not provided it must default to 0")

    def test_pass_key_provided(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut({"PASS_KEY": 46})
        self.assertEqual(46, sut.passkey, "pass_key must be 46")

    def test_accept_not_provided(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut(None)
        self.assertEqual(1, sut.accept, "when accept arg is not passed it must default to 1 (true)")

    def test_accept_true(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut({"ACCEPT_PAIRING": True})
        self.assertEqual(1, sut.accept, "accept arg as True must be converted to passed to the api as 1")

    def test_accept_false(self):
        """
        Test accept arg is correct
        """
        sut = self._create_sut({"ACCEPT_PAIRING": False})
        self.assertEqual(0, sut.accept, "accept arg as False must be converted to passed to the api as 0")

    def _create_sut(self, args):
        """
        Create the SUT
        """
        ts = TestStepBase(None, None, args, mock.Mock())
        return PairingConfig(ts._pars)

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
