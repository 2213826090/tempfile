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
:summary: Nokia BH214 implementation
:since: 20/02/2014
:author: fbongiax
"""

import mock

from acs_test_scripts.Equipment.Bluetooth.Mouse.BtMouse import BtMouse
from unit_test.UtEquipment.IOCardSutTestHelper import IOCardSutTestHelper
from ErrorHandling.TestEquipmentException import TestEquipmentException
from unit_test_fwk.UTestBase import UTestBase


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class BtMouseTest(UTestBase):
    """
    Test class for BtMouse
    """
    def setUp(self):
        UTestBase.setUp(self)
        self._sut = BtMouse("NokiaBH214", "NokiaBH214", None, mock.MagicMock(), mock.Mock())
        self._helper = IOCardSutTestHelper(self._sut)

        self._button_name = None
        self._relay = None
        self._duration_secs = 1
        self._call_seq = []

    def test_left_button_not_configured(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "leftButton/keyPressTimer not configured"):
            self._sut.left_click()

    def test_right_button_not_configured(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "rightButton/keyPressTimer not configured"):
            self._sut.right_click()

    def test_middle_button_not_configured(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "middleButton/keyPressTimer not configured"):
            self._sut.middle_click()

    def test_power_button_not_configured(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "powerButton not configured"):
            self._sut.switch_on()

    def test_pair_button_not_configured(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "powerButton/pairButton/pairTimer not configured"):
            self._sut.set_discoverable()

    def _set_button_info(self, button, rele_line):
        self._button_name = button
        self._relay = rele_line
        self._helper.set_buttons_info({button: str(rele_line), "keyPressTimer": self._duration_secs})

    def test_left_button_click(self):
        self._test_button_click("leftButton", 1, self._sut.left_click)

    def test_right_button_click(self):
        self._test_button_click("rightButton", 2, self._sut.right_click)

    def test_middle_button_click(self):
        self._test_button_click("middleButton", 3, self._sut.middle_click)

    def test_switch_on(self):
        self._set_button_info("powerButton", 1)
        self._sut.init()
        self._sut.switch_on()
        self._sut._iocard.enable_line.assert_called_with(1)

    def test_switch_off(self):
        self._set_button_info("powerButton", 1)
        self._sut.init()
        self._sut.switch_off()
        self._sut._iocard.disable_line.assert_called_with(1)

    def test_pair_button(self):
        pair_line = 2
        pair_time_secs = 7
        self._helper.set_bench_params({"powerButton": "1", "pairButton": str(pair_line),
                                       "pairTimer": str(pair_time_secs)})
        self._sut.init()
        self._sut.switch_on = mock.MagicMock("switch_on", side_effect=self._track_switch_on)
        self._sut.switch_off = mock.MagicMock("switch_off", side_effect=self._track_switch_off)

        self._sut.set_discoverable()

        self.assertEqual(2, len(self._call_seq), "either switch_on() or switch_off() hasn't been called")
        self.assertEqual(["switch_off", "switch_on"], self._call_seq, "switch_off() / switch_on() invalid sequence")

        # Check pair button was pressed for the right period
        self._helper.assert_iocard_line_activated(pair_line, pair_time_secs)

    def _track_switch_on(self):
        self._call_seq.append("switch_on")

    def _track_switch_off(self):
        self._call_seq.append("switch_off")

    def _test_button_click(self, button, relay, method):
        self._set_button_info(button, relay)
        self._sut.init()
        method()
        self._helper.assert_iocard_line_activated(self._relay, self._duration_secs)
