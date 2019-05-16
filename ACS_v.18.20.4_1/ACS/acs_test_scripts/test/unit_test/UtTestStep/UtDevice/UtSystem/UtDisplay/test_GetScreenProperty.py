# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG DRD
:summary: Unit test module
:since: 10/03/14
:author: emarchan
"""

from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Display.GetScreenProperty import GetScreenProperty
from Core.TestStep.TestStepContext import TestStepContext
from UtilitiesFWK.Utilities import TestConst


class GetScreenPropertyTestCase(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None
        self._dest_var = "_GetScreenProperty__var"
        # Create my own context because the _context is mocked in the UTestTestStepBase
        self._context = TestStepContext()

    def test_get_state_on_succeed(self):
        self._create_sut("STATE")

        valueExp = TestConst.STR_ON
        method = self._sut._api.get_screen_status

        method.return_value = valueExp
        self._sut.run(self._context)

        ValueGot = self._context.get_info(self._dest_var)
        self._assert_method_called_and_value_good(self._sut, method, valueExp, ValueGot,
                                                  'VERDICT: %s stored as %s' % (self._dest_var, str(valueExp)))

    def test_get_timeout_succeed(self):
        self._create_sut("TIMEOUT")
        self._sut.run(self._context)
        self._sut._api.get_screen_timeout.assert_called_once_with()

    def test_get_resolution_succeed(self):
        self._create_sut("RESOLUTION")
        self._sut.run(self._context)
        self._sut._api.get_screen_resolution.assert_called_once_with()

    def test_get_backlight_succeed(self):
        self._create_sut("BACKLIGHT")
        self._sut.run(self._context)
        self._sut._api.get_backlight_level.assert_called_once_with()


    def _create_sut(self, property):
         self._sut = GetScreenProperty(None, None, {"PROPERTY":property, "SAVE_AS":self._dest_var}, Mock())
