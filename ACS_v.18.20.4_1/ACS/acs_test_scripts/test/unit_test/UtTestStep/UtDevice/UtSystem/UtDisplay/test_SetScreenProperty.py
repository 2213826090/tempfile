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
:since: 07/03/14
:author: emarchan
"""

from mock import patch, Mock, call
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Display.SetScreenProperty import SetScreenProperty
from UtilitiesFWK.Utilities import Global


class SetScreenPropertyTestCase(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_set_state_on_succeed(self):
        self._create_sut("STATE", "ON")
        self._sut.run(self._context)
        self._assert_set_state_was_called_with_value("1")

    def test_set_state_off_succeed(self):
        self._create_sut("STATE", "OFF")
        self._sut.run(self._context)
        self._assert_set_state_was_called_with_value("0")

    def test_set_state_fail(self):
        self._create_sut("STATE", "blabla")
        self._assert_run_throw_config_exception(self._sut, "Invalid parameter \(Invalid value\)")

    def test_set_timeout_10_succeed(self):
        self._create_sut("TIMEOUT", "10")
        self._sut.run(self._context)
        self._assert_set_timeout_was_called_with_value(10)

    def test_set_timeout_wrong_text_fail(self):
        self._create_sut("TIMEOUT", "blabla")
        self._assert_run_throw_config_exception(self._sut, "Invalid value")

    def test_set_brightness_50_succeed(self):
        self._create_sut("BRIGHTNESS", 50)
        self._sut.run(self._context)
        self._assert_set_brightness_was_called_with_value(50)

    def test_set_brightness_wrong_text_fail(self):
        self._create_sut("BRIGHTNESS", "blabla")
        self._assert_run_throw_config_exception(self._sut, "Invalid value")

    def test_set_backlight_50_succeed(self):
        self._create_sut("BACKLIGHT", 50)
        self._sut.run(self._context)
        self._assert_set_backlight_was_called_with_value(50)

    def test_set_backlight_wrong_text_fail(self):
        self._create_sut("BACKLIGHT", "blabla")
        self._assert_run_throw_config_exception(self._sut, "Invalid value")


    def _create_sut(self, property, value):
        self._sut = SetScreenProperty(None, None, {"PROPERTY":property, "VALUE":value}, Mock())

    def _assert_set_state_was_called_with_value(self, value):
        self._check_call_value(self._sut._api.set_phone_screen_lock_on, value)

    def _assert_set_timeout_was_called_with_value(self, value):
        self._check_call_value(self._sut._api.set_screen_timeout, value)

    def _assert_set_brightness_was_called_with_value(self, value):
        self._check_call_value(self._sut._api.set_display_brightness, value)

    def _assert_set_backlight_was_called_with_value(self, value):
        self._check_call_value(self._sut._api.set_backlight_level, value)

    def _check_call_value(self, method, value):
        method.assert_called_once_with(value)
        self.assertEqual("No errors", self._sut.ts_verdict_msg)
