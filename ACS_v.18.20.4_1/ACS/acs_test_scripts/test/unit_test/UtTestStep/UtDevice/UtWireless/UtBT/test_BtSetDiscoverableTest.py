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

from acs_test_scripts.TestStep.Device.Wireless.BT.BtSetDiscoverable import BtSetDiscoverable
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class SetDiscoverableTest(UTestTestStepBase):
    """
    SetDiscoverable test cases
    """
    def test_run_default_mode_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut()

        sut.run(self._context)
        sut._api.set_bt_discoverable.assert_called_with('both', 0)

    def test_run_inquiry_mode_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"MODE": "inquiry"})

        sut.run(self._context)
        sut._api.set_bt_discoverable.assert_called_with('inquiry', 0)

    def test_run_timeout_passed_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"TIMEOUT": "10"})

        sut.run(self._context)
        sut._api.set_bt_discoverable.assert_called_with('both', 10)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = BtSetDiscoverable(None, None, test_step_pars, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
