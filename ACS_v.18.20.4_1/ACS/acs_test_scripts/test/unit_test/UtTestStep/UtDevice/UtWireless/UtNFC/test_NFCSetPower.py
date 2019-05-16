"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: unit test
:since 05/09/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.NFC.NFCSetPower import NFCSetPower
from UtilitiesFWK.Utilities import TestConst


class NFCSetPowerTest(UTestTestStepBase):
    """
    NFCSetPower test cases
    """

    SET_PWR_ON = "on"
    SET_PWR_OFF = "off"
    GET_PWR_ON = "ON"
    GET_PWR_OFF = "OFF"
    POWER_PARAM = "POWER"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._responsed = []
        self.calls = []

    def test_set_power_on_ok(self):
        power = self.SET_PWR_ON
        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_ON])
        sut.run(None)
        sut._api.nfc_enable.assert_called_once_with()

    def test_set_power_off_ok(self):
        power = self.SET_PWR_OFF
        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_OFF])
        sut.run(None)
        sut._api.nfc_disable.assert_called_once_with()

    def test_set_power_off_afer_several_times_ok(self):
        power = self.SET_PWR_OFF
        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_ON, self.GET_PWR_ON, self.GET_PWR_OFF])
        sut._timeout = 5
        sut.run(None)
        sut._api.nfc_disable.assert_called_once_with()

    def test_set_power_on_ko(self):
        power = self.SET_PWR_ON
        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_OFF])
        self._assert_run_throw_device_exception(sut, "Set NFC %s failure" % power)

    def test_set_power_off_ko(self):
        power = self.SET_PWR_OFF
        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_ON])
        self._assert_run_throw_device_exception(sut, "Set NFC %s failure" % power)

    def test_set_power_on_off_on_ok(self):
        power = "%s,%s,%s" % (self.SET_PWR_ON,
                              self.SET_PWR_OFF,
                              self.SET_PWR_ON)

        sut = self._create_sut({self.POWER_PARAM: power})
        self._simulate_get_nfc_status_returning(sut, [self.GET_PWR_ON, self.GET_PWR_OFF, self.GET_PWR_ON])
        self._track_set_nfc_power_calls(sut)

        sut.run(None)
        self.assertEqual([self.SET_PWR_ON, self.SET_PWR_OFF, self.SET_PWR_ON], self.calls)


    def _track_set_nfc_power_calls(self, sut):
        """
        Save all the parameters nfc_enable and nfc_disable were called with in order to check every calls went well.
        """
        sut._api.nfc_enable.side_effect = lambda: self.calls.append(self.SET_PWR_ON)
        sut._api.nfc_disable.side_effect = lambda: self.calls.append(self.SET_PWR_OFF)

    def _side_effect_get_nfc_status_response(self):
        return self._responsed.pop(0)

    def _simulate_get_nfc_status_returning(self, sut, expected_power):
        self._responsed = expected_power
        sut._api.get_nfc_status.side_effect = self._side_effect_get_nfc_status_response
        return sut

    def _assert_run_returned(self, sut):
        sut.run(None)
        sut._api.set_wifi_power.assert_called_once_with()

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = NFCSetPower(None, None, test_step_pars, mock.Mock())
        sut._timeout = 0
        return sut

