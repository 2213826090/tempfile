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

:organization: INTEL MCG PSI
:summary: Unit test module
:since: 27/03/14
:author: ssavrim
"""

from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Utilities.Checks.CheckContextInfoExists import CheckContextInfoExists


class ConnectDeviceTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None
        self._context = TestStepContext()

    def test_key_in_context(self):
        self._create_sut("EXISTING_KEY")
        self._context.set_info("EXISTING_KEY", "MY_VALUE")
        self._assert_run_succeeded(self._sut)

    def test_key_not_in_context(self):
        self._create_sut("UKNOWN_KEY")
        self._assert_run_throw_config_exception(self._sut, "UKNOWN_KEY is not found in the context")

    def _create_sut(self, key_name):
        self._sut = CheckContextInfoExists(None, None, {"KEY": key_name}, Mock())
        return self._sut
