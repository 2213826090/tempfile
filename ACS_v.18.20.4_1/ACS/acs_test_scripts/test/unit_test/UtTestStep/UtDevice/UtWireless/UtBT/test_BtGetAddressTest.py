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

from acs_test_scripts.TestStep.Device.Wireless.BT.BtGetAddress import BtGetAddress
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext


class GetAddressTest(UTestTestStepBase):
    """
    GetAddress test cases
    """

    def _return_addr(self):
        """
        Stub method
        """
        return "00:00:00:00:00"

    def test_run_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"SAVE_AS": "BDARRD"})

        self._context = TestStepContext()
        sut.run(self._context)
        self.assertEqual("00:00:00:00:00", self._context.get_info("BDARRD"))

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = BtGetAddress(None, None, test_step_pars, mock.Mock())
        sut._api.get_bt_adapter_address = self._return_addr
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
