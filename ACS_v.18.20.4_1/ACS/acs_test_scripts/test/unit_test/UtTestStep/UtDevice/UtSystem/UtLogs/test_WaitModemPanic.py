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
:summary: This file implements the unit test for WaitModemPanic
:since: 2014-10-27
:author: emarchan

"""
import mock
from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtSystem.UtLogs.test_ModemPanicBase import test_ModemPanicBase
from acs_test_scripts.TestStep.Device.System.Logs.WaitModemPanic import WaitModemPanic
from acs_test_scripts.TestStep.Device.System.Logs.SysDebugInit import SysDebugInit
from lxml import etree

DEFAULT_MODEM_PANIC_TIMEOUT = "60"
DEFAULT_MODEM_PANIC_OCCURRED = "has_modem_panics"
DEFAULT_SYSDEBUG_CONFIG = "ModemPanic; CrashInfo;"

class test_WaitModemPanic(test_ModemPanicBase):

    def setUp(self):
        test_ModemPanicBase.setUp(self)
        self._responsed = []

    def test_panic_occurred_ok(self):
        self._create_sut({"MODEM_PANIC_TIMEOUT":DEFAULT_MODEM_PANIC_TIMEOUT,
                          "MODEM_PANIC_OCCURRED":DEFAULT_MODEM_PANIC_OCCURRED})
        self._sut._get_sysdebug_logs = self._create_fake_syslog_with_panic
        self._sut.run(self._context)
        valueGot = self._context.get_info(DEFAULT_MODEM_PANIC_OCCURRED)
        valueExp = 'true'
        self.assertEqual(valueExp, valueGot)
        self.assertEqual('VERDICT: A modem panic occurred.', self._sut.ts_verdict_msg)

    def test_no_panic_occurred_ok(self):
        self._create_sut({"MODEM_PANIC_TIMEOUT":DEFAULT_MODEM_PANIC_TIMEOUT,
                          "MODEM_PANIC_OCCURRED":DEFAULT_MODEM_PANIC_OCCURRED})
        self._sut._get_sysdebug_logs = self._create_fake_syslog_with_no_panic
        self._sut.run(self._context)
        valueGot = self._context.get_info(DEFAULT_MODEM_PANIC_OCCURRED)
        valueExp = 'false'
        self.assertEqual(valueExp, valueGot)
        self.assertEqual('VERDICT: No modem panic occurred.', self._sut.ts_verdict_msg)

    def test_panic_occurred_after_sometime_ok(self):
        self._create_sut({"MODEM_PANIC_TIMEOUT":DEFAULT_MODEM_PANIC_TIMEOUT,
                          "MODEM_PANIC_OCCURRED":DEFAULT_MODEM_PANIC_OCCURRED})
        self._sut._timeout = 5
        self._responsed = [self._create_fake_syslog_with_no_panic, self._create_fake_syslog_with_no_panic,
                           self._create_fake_syslog_with_no_panic, self._create_fake_syslog_with_panic]
        self._sut._get_sysdebug_logs = self._stub_get_log_response
        self._sut._time_out_reached = mock.MagicMock(return_value=False)
        self._sut.run(self._context)
        valueGot = self._context.get_info(DEFAULT_MODEM_PANIC_OCCURRED)
        valueExp = 'true'
        self.assertEqual(valueExp, valueGot)
        self.assertEqual('VERDICT: A modem panic occurred.', self._sut.ts_verdict_msg)

    def test_bad_timeout_call_ko(self):
        with self.assertRaisesRegexp(AssertionError, "is invalid at this stage"):
            self._create_sut({"MODEM_PANIC_TIMEOUT":"Dummy", "MODEM_PANIC_OCCURRED":DEFAULT_MODEM_PANIC_OCCURRED})

    def test_get_safe_range_not_enabled_ko(self):
        self._create_sut({"MODEM_PANIC_TIMEOUT":DEFAULT_MODEM_PANIC_TIMEOUT,
                      "MODEM_PANIC_OCCURRED":DEFAULT_MODEM_PANIC_OCCURRED})
        self._assert_run_throw_device_exception(self._sut, "Can't find ModemPanics in the logs, did you enable it at init?")

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        SysDebugInit(None, None, {"SYSDEBUG_CONFIG":DEFAULT_SYSDEBUG_CONFIG}, mock.MagicMock())
        self._sut = WaitModemPanic(None, None, test_step_pars, mock.MagicMock())
        self._sut._timeout = 0
        self._sut._wait_for = 0

    def _create_fake_syslog_with_panic(self):
        """
        Simulates a real system log output with a modem panic
        """
        xmltree = self._create_fake_syslog_with_no_panic()
        xmlpanic = etree.Element("mpanic")
        xmlpanic.attrib["errno"] = "DUMMY_ERR"
        xmltree.append(xmlpanic)
        return xmltree

    def _create_fake_syslog_with_no_panic(self):
        """
        Simulates a real system log output without modem panic
        """
        xmltree = etree.Element("ModemPanics")
        return xmltree

    def _stub_get_log_response(self):
        answer = self._responsed.pop(0)
        return answer()
