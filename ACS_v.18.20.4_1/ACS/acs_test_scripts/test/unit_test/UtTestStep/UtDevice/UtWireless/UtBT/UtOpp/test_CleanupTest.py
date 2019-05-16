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
:since:30/12/2013
:author: fbongiax
"""

import unittest2 as unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.Opp.BtOppCleanup import BtOppCleanup
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class BtOppCleanupTest(UTestTestStepBase):
    """
    BtOppCleanup test cases
    """

    # pylint: disable=C0103
    def setUp(self):
        """
        Setup method
        """
        UTestTestStepBase.setUp(self)

        self._clean_is_called = False
        self._init_is_called = False

    def test_run_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut(None)

        sut.run(self._context)
        self.assertTrue(self._clean_is_called)

    def test_run_with_files(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"REMOVE_FILES": "True", "FILES": "1MB.txt"})

        sut.run(self._context)
        self.assertTrue(self._clean_is_called)
        self.assertTrue(self._init_is_called)

    def _stub_bt_opp_clean_notification_list(self):
        """
        Method stub
        """
        self._clean_is_called = True

    def _stub_bt_opp_init(self, file_name):
        """
        Method stub
        """
        self._init_is_called = True
        self.assertEqual("1MB.txt", file_name)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars):
        """
        Create the SUT with only test step pars
        """
        sut = BtOppCleanup(None, None, test_step_pars, mock.Mock())
        sut._api.bt_opp_clean_notification_list = self._stub_bt_opp_clean_notification_list
        sut._api.bt_opp_init = self._stub_bt_opp_init
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
