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

@organization: INTEL MCG PSI
@summary: unit test
@since 04/04/2014
@author: obouzain
"""

import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.GNSS.GNSSSetPower import GNSSSetPower
from UtilitiesFWK.Utilities import TestConst

GET_PWR_ON = 1
GET_PWR_OFF = 0
POWER_PARAM = "POWER"
class GNSSSetPowerTest(UTestTestStepBase):
    """
    SetPower test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._responsed = []
        self.calls = []

    def test_set_power_on_ok(self):
        power = TestConst.STR_ON
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [power])
        self._assert_run_returned(sut, power)

    def test_set_power_off_ok(self):
        power = TestConst.STR_OFF
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [power])
        self._assert_run_returned(sut, power)


    def test_set_power_on_afer_several_times_ok(self):
        power = TestConst.STR_ON
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [GET_PWR_OFF, GET_PWR_OFF, GET_PWR_ON])
        sut._timeout = 5
        self._assert_run_returned(sut, power)


    def test_set_power_on_ko(self):
        power = TestConst.STR_ON
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [TestConst.STR_OFF])
        self._assert_run_throw_device_exception(sut, "Set GPS %s failure" % power)

    def test_set_power_off_ko(self):
        power = TestConst.STR_OFF
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [TestConst.STR_ON])
        self._assert_run_throw_device_exception(sut, "Set GPS %s failure" % power)

    def test_set_power_on_off_on_ok(self):
        power = "%s,%s,%s" % (TestConst.STR_ON,
                              TestConst.STR_OFF,
                              TestConst.STR_ON)
        sut = self._create_sut({POWER_PARAM: power})
        self._simulate_get_power_returning(sut, power, [GET_PWR_ON, GET_PWR_OFF, GET_PWR_ON])
        self._track_set_GPS_power_calls(sut)
        sut.run(None)
        self.assertEqual([TestConst.STR_ON,
                          TestConst.STR_OFF,
                          TestConst.STR_ON], self.calls)

    def _track_set_GPS_power_calls(self, sut):
        """
        Save all the parameters set_GPS_power was called with in order to check every calls went well.
        """
        sut._api.set_gps_power.side_effect = lambda x:self.calls.append(x)

    def _side_effect_get_power_response(self):
        return self._responsed.pop(0)

    def _simulate_get_power_returning(self, sut, set_power, expected_power):
        self._responsed = expected_power
        sut._api.get_gps_power_status.side_effect = self._side_effect_get_power_response
        return sut

    def _assert_run_returned(self, sut, expected):
        sut.run(None)
        sut._api.set_gps_power.assert_called_once_with(expected)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GNSSSetPower(None, None, test_step_pars, mock.Mock())
        sut._timeout = 0
        return sut

