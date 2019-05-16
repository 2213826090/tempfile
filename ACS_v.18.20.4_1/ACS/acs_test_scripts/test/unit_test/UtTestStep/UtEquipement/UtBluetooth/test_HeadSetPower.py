# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:author: fbongiax
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Equipment.Bluetooth.HeadSetPower import HeadSetPower


class HeadSetPowerTest(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_on_ok(self):
        sut = self._create_sut({"STATE": "ON"})
        sut.run(None)
        sut._api.set_power.assert_called_with(True)

    def test_off_ok(self):
        sut = self._create_sut({"STATE": "OFF"})
        sut.run(None)
        sut._api.set_power.assert_called_with(False)

    def test_sequence_ok(self):
        sut = self._create_sut({"STATE": "OFF, ON, OFF"})
        sut.run(None)
        sut._api.set_power.assert_called_with(False)

    def test_pairable_ok(self):
        sut = self._create_sut({"STATE": "PAIRABLE"})
        sut.run(None)
        sut._api.set_discoverable.assert_called_with()

    def test_reset_off_headset_was_on(self):
        sut = self._create_sut({"STATE": "RESET_OFF"})
        sut.run(None)
        sut._api.set_power.assert_called_with(False)

    def test_reset_off_headset_was_off(self):
        sut = self._create_sut({"STATE": "RESET_OFF"})
        sut._dut_bt_api.bt_find_device.return_value = False

        sut.run(None)
        sut._api.set_discoverable.assert_called_with()
        sut._api.set_power.assert_called_with(False)

    def test_dut_bt_was_off_put_it_back_to_off(self):
        sut = self._create_sut({"STATE": "RESET_OFF"})
        sut._dut_bt_api.get_bt_power_status_eot.return_value = "STATE_OFF"
        sut.run(None)
        sut._dut_bt_api.set_bt_power.assert_called_with("off")

    def _create_sut(self, args=None):
        self._sut = HeadSetPower(None, mock.Mock(), args, mock.Mock())
        return self._sut
