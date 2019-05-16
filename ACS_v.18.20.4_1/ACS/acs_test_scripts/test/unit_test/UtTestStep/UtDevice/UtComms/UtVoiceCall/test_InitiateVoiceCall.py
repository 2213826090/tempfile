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
:summary: Unit test module
:since: 23/06/2014
:author: jfranchx
"""
import unittest2 as unittest
import mock
from acs_test_scripts.TestStep.Device.Comms.VoiceCall.InitiateVoiceCall import InitiateVoiceCall

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext


class InitiateVoiceCallTestCase(UTestTestStepBase):
    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._flight_mode_state = 0

    def test_init_ok(self):
        """
        Test voice call init succeed
        """
        sut = self._create_sut({"NUMBER_TO_CALL": "0123456789", "VC_TYPE": "CIRCUIT"})
        self._set_flight_mode(0)
        self._assert_run_succeeded(sut)
        self._method.assert_called_with("0123456789", False)

    def test_init_flight_mode_on(self):
        """
        Test voice call init fail due to flight mode
        """
        sut = self._create_sut({"NUMBER_TO_CALL": "0123456789", "VC_TYPE": "CIRCUIT"})
        self._set_flight_mode(1)
        self._assert_run_throw_device_exception(sut, "Can't perform VC_DIAL : flight mode is enabled")

    def test_init_unsupported_type_voip(self):
        """
        Test voice call inti fail due to unsupported type
        """
        sut = self._create_sut({"NUMBER_TO_CALL": "0123456789", "VC_TYPE": "VOIP"})
        self._assert_run_throw_device_exception(sut, "Can't perform VC_TYPE - unsupported type : VOIP")

    def _set_flight_mode(self, state):
        """
        Set flight mode state
        """
        self._flight_mode_state = state

    def _get_flight_mode(self):
        """
        Get flight mode state
        """
        return self._flight_mode_state

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = InitiateVoiceCall(None, None, test_step_pars, mock.Mock())
        self._method = sut._voice_call_api.dial
        sut._networking_api.get_flight_mode = self._get_flight_mode

        return sut


if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
