"""

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: UMG PSI Validation
@summary: This file implements the unit test for GetComputerDateTime

@author: emarchan

"""
import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Equipment.Computer.GetComputerDateTime import GetComputerDateTime

DEFAULT_DAY = "11"
DEFAULT_MONTH = "12"
DEFAULT_YEAR = "2014"
DEFAULT_HOURS = "15"
DEFAULT_MINUTES = "23"
DEFAULT_SECONDS = "10"
DEFAULT_DEST = "ctx_var"
class Test_GetComputerDateTime(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._sut = None
        self._ping_return = None

    def test_connect_call_ok(self):
        self._create_sut({"COMPUTER_CURRENT_DATE_TIME": DEFAULT_DEST})
        self._sut.run(self._context)

        self.assertEqual(DEFAULT_DAY, self._context.get_info(DEFAULT_DEST + ":DAY"))
        self.assertEqual(DEFAULT_MONTH, self._context.get_info(DEFAULT_DEST + ":MONTH"))
        self.assertEqual(DEFAULT_YEAR, self._context.get_info(DEFAULT_DEST + ":YEAR"))
        self.assertEqual(DEFAULT_HOURS, self._context.get_info(DEFAULT_DEST + ":HOURS"))
        self.assertEqual(DEFAULT_MINUTES, self._context.get_info(DEFAULT_DEST + ":MINUTES"))
        self.assertEqual(DEFAULT_SECONDS, self._context.get_info(DEFAULT_DEST + ":SECONDS"))

    def _create_sut(self, test_step_pars=None):
        self._sut = GetComputerDateTime(None, mock.Mock(), test_step_pars, mock.Mock())
        self._sut._get_current_date_and_time = self._return_date_time
        return self._sut

    def _return_date_time(self):

        return {"DAY": DEFAULT_DAY,
                "MONTH": DEFAULT_MONTH,
                "YEAR": DEFAULT_YEAR,
                "HOURS": DEFAULT_HOURS,
                "MINUTES": DEFAULT_MINUTES,
                "SECONDS": DEFAULT_SECONDS}
