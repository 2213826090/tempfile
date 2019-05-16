# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test test step module base
:since: 24/02/14
:author: cbonnard
"""

from mock import patch, Mock
from unit_test_fwk.UTestBase import UTestBase
from UtilitiesFWK.Utilities import AcsConstants
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException

class UTestTestStepBase(UTestBase):
    """
       TestStep UnitTest service class
    """
    def setUp(self):
        UTestBase.setUp(self)
        self._context_patch = patch("Core.TestStep.TestStepContext.TestStepContext")
        self._context = self._context_patch.start()

    def tearDown(self):
        self._context_patch.stop()
        UTestBase.tearDown(self)

    def _assert_run_succeeded(self, sut):
        """
        Assert that the test step run correctly
        """
        self._assert_run_succeeded_with_msg(sut, AcsConstants.NO_ERRORS)

    def _assert_run_succeeded_with_msg(self, sut, expected):
        """
        Assert that the test step run correctly
        """
        sut.run(self._context)
        self.assertEqual(expected, sut.ts_verdict_msg)

    def _assert_run_throw_device_exception(self, sut, msg):
        """
        Assert that a device exception was raised with the given message
        """
        with self.assertRaisesRegexp(DeviceException, msg):
            sut.run(self._context)

    def _assert_run_throw_config_exception(self, sut, msg):
        """
        Assert that a config exception was raised with the given message
        """
        with self.assertRaisesRegexp(AcsConfigException, msg):
            sut.run(self._context)

    def _assert_run_throw_equipment_exception(self, sut, msg):
        """
        Assert that a test equipment exception was raised with the given message
        """
        with self.assertRaisesRegexp(TestEquipmentException, msg):
            sut.run(self._context)

    def _assert_method_called_and_value_good(self, sut, method, valueExp, valueGot, verdict=None):
        """
        Assert that a method was called with the good parameters, and returned what was expected.
        @type method: method
        @param method: The method we want to check the call
        @type valueExp: Whatever (the same as valueGot)
        @param valueExp: Expected value we want the method call to return
        @type valueGot: Whatever (the same as valueExp)
        @param valueGot : Actual value returned by the method call
        @type verdict: String
        @param verdict: The verdict returned by the method call.

        """
        method.assert_called_once_with()
        self.assertEqual(valueExp, valueGot)
        if verdict is not None:
            self.assertEqual(verdict, sut.ts_verdict_msg)

    def _assert_method_called_and_value_bad(self, method, valueExp, valueGot):
        """
        Assert that a method was called with the bad parameters, and didn't return what was expected.
        @type method: method
        @param method: The method we want to check the call
        @type valueExp: Whatever (the same as valueGot)
        @param valueExp: Expected value we want the method call to return
        @type valueGot: Whatever (the same as valueExp)
        @param valueGot : Actual value returned by the method call

        """
        method.assert_called_once_with()
        self.assertNotEqual(valueExp, valueGot)
