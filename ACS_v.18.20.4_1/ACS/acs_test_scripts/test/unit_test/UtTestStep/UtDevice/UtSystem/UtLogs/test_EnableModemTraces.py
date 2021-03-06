# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the unit test for disabling modem traces.
:since: 2014-10-23
:author: emarchan

"""
import mock
from acs_test_scripts.test.unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Logs.EnableModemTraces import EnableModemTraces
from acs_test_scripts.Device.UECmd.UECmdTypes import BPLOG_LOC

DEFAULT_TRACES_DEST = "EMMC"
class test_EnableModemTraces(UTestTestStepBase):

    def test_enable_traces_call_ok(self):
        self._create_sut({"MODEM_TRACES_DEST":DEFAULT_TRACES_DEST})
        self._sut.run(self._context)
        self._sut._modem_api.activate_modem_trace.assert_called_once_with(BPLOG_LOC.EMMC)

    def test_enable_traces_call_ko(self):
        self._create_sut({"MODEM_TRACES_DEST":"Dummy"})
        with self.assertRaisesRegexp(AssertionError, "is invalid at this stage"):
            self._sut.run(self._context)

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        self._sut = EnableModemTraces(None, None, test_step_pars, mock.MagicMock())
