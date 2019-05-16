"""
@summary: Unit test module for CheckPassFailLog
@since: 21 March 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Files.CheckPassFailLog import CheckPassFailLog
from ErrorHandling.DeviceException import DeviceException
from Core.TestStep.TestStepContext import TestStepContext

class CheckPassFailTestCase(UTestTestStepBase):
    _mock_file_to_match = None

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def _create_sut(self):
        self._sut = CheckPassFailLog(None, None, None, Mock())

    def _mock_check_file_exists(self, test_file):
        if test_file == self._sut._pars.file_path + "/" + self._mock_file_to_match:
            return (True, "bogus passed")
        else:
            return (False, "bogus failed")

    def test_pass(self):
        """
            Check that it reports passing status if it finds pass.log
        """
        self._create_sut()
        self._sut._pars.file_path = "src_files"
        self._sut._pars.pass_file = "pass.log"
        self._sut._pars.fail_file = "fail.log"
        self._sut._pars.timeout = 1
        self._sut._pars.polling_interval = 1
        self._sut._pars.result_msg_string = "My Favorite Test Step"
        self._mock_file_to_match = "pass.log"
        self._sut.file_api.exist = Mock()
        self._sut.file_api.exist.side_effect = self._mock_check_file_exists
        self._sut.run(self._context)
        assert(self._sut._ts_verdict_msg == self._sut._pars.result_msg_string + ": PASSED")

    def test_fail(self):
        """
            Check that it reports passing status if it finds pass.log
        """
        self._create_sut()
        self._sut._pars.file_path = "src_files"
        self._sut._pars.pass_file = "pass.log"
        self._sut._pars.fail_file = "fail.log"
        self._sut._pars.timeout = 1
        self._sut._pars.polling_interval = 1
        self._sut._pars.result_msg_string = "My Favorite Test Step"
        self._mock_file_to_match = "fail.log"
        self._sut.file_api.exist = Mock()
        self._sut.file_api.exist.side_effect = self._mock_check_file_exists
        with self.assertRaises(DeviceException):
            self._sut.run(self._context)
