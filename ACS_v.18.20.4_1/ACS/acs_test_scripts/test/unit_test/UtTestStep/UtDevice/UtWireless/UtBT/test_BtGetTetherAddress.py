"""
:summary: unit test for GetTetherAddress test step
:since: 28/04/14
:author: Val Peterson
:organization: INTEL PEG-SVE-DSV

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
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.BtGetTetherAddress import BtGetTetherAddress
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext


class GetTetherAddressTest(UTestTestStepBase):
    """
    GetTetherAddress test cases
    """
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None
        self._context = TestStepContext()

    def test_run_ok(self):
        """
        Test runs ok
        """
        self._sut = self._create_sut()
        self._sut._pars.save_as= "MYADDR"
        self._sut._device.get_uecmd.return_value.get_interface_ipv4_address.return_value = "192.168.0.1"
        self._sut.run(self._context)
        returned_value = self._context.get_info("MYADDR")
        assert(returned_value == "192.168.0.1")

    def _create_sut(self):
        """
        Create the SUT with only test step pars
        """
        sut = BtGetTetherAddress(None, None, None, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
