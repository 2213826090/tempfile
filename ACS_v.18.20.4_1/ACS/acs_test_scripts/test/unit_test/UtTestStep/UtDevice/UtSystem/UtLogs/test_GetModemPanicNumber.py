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
:summary: This file implements the unit test for GetModemPanicNumber.
:since: 2014-10-27
:author: emarchan

"""
import mock
from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtSystem.UtLogs.test_ModemPanicBase import test_ModemPanicBase
from acs_test_scripts.TestStep.Device.System.Logs.GetModemPanicNumber import GetModemPanicNumber
from acs_test_scripts.TestStep.Device.System.Logs.SysDebugInit import SysDebugInit
from ErrorHandling.DeviceException import DeviceException
from lxml import etree

DEFAULT_MODEM_PANIC_COUNT = "my_panics_value"
DEFAULT_SYSDEBUG_CONFIG = "ModemPanic; CrashInfo;"

class test_GetModemPanicNumber(test_ModemPanicBase):

    def setUp(self):
        test_ModemPanicBase.setUp(self)

    def test_init_check_ok(self):
        self._create_sut({"MODEM_PANIC_COUNT":DEFAULT_MODEM_PANIC_COUNT})
        self._sut._get_sysdebug_logs = self._create_fake_syslog_with_panic
        self._sut.run(self._context)
        valueGot = self._context.get_info(DEFAULT_MODEM_PANIC_COUNT)
        valueExp = '1'
        self.assertEqual(valueExp, valueGot)
        self.assertEqual('VERDICT: %s modem panic found.' % valueExp, self._sut.ts_verdict_msg)

    def test_init_check_fail(self):
        self._create_sut({"MODEM_PANIC_COUNT":DEFAULT_MODEM_PANIC_COUNT})
        self._assert_run_throw_device_exception(self._sut, "Can't find ModemPanics in the logs, did you enable it at init?")

    def test_call_ok(self):
        self._create_sut({"MODEM_PANIC_COUNT":DEFAULT_MODEM_PANIC_COUNT})
        with self.assertRaisesRegexp(DeviceException, "Can't find ModemPanics in the logs, did you enable it at init?"):
            self._sut.run(self._context)
            self._sut._sysdebug_apis.report.assert_called_once_with(DEFAULT_MODEM_PANIC_COUNT)


    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        SysDebugInit(None, None, {"SYSDEBUG_CONFIG":DEFAULT_SYSDEBUG_CONFIG}, mock.MagicMock())

        self._sut = GetModemPanicNumber(None, None, test_step_pars, mock.MagicMock())

