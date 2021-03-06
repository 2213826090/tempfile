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
:summary: This file implements the unit test for sysdebug init.
:since: 2014-10-23
:author: emarchan

"""
import mock
from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtSystem.UtLogs.test_SysDebugBase import test_SysDebugBase
from acs_test_scripts.TestStep.Device.System.Logs.SysDebugInit import SysDebugInit

DEFAULT_SYSDEBUG_CONFIG = "ModemPanic; CrashInfo;"
class test_SysDebugInit(test_SysDebugBase):

    def setUp(self):
        test_SysDebugBase.setUp(self)

    def test_init_call_ok(self):
        self._create_sut({"SYSDEBUG_CONFIG":DEFAULT_SYSDEBUG_CONFIG})
        self._sut.run(self._context)
        self._sut._sysdebug_apis.init.assert_called_once_with(DEFAULT_SYSDEBUG_CONFIG)

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        self._sut = SysDebugInit(None, None, test_step_pars, mock.MagicMock())
