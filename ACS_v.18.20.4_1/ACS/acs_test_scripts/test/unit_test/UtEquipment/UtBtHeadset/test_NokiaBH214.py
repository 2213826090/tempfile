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
:summary: unit test
:since:28/01/2014
:author: fbongiax
"""
import time
import mock
from acs_test_scripts.Equipment.BTHeadset.NokiaBH214.NokiaBH214 import NokiaBH214
from ErrorHandling.TestEquipmentException import TestEquipmentException
from unit_test_fwk.UTestBase import UTestBase


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class NokiaBH214Test(UTestBase):
    BDADDRESS = "11:22:33:44:55:66"

    def _side_effect(self, key):
        if key in self._bench_params:
            return self._bench_params[key]
        return None

    def _set_buttons_info(self, values):
        for key in values:
            self._bench_params[key] = values[key]

    def _assert_iocard_line_activated(self, line, duration):
        self._sut._iocard.enable_line.assert_called_with(line)
        time.sleep.assert_called_with(duration)
        self._sut._iocard.disable_line.assert_called_with(line)

    def setUp(self):
        UTestBase.setUp(self)
        self._sut = NokiaBH214("NokiaBH214", "NokiaBH214", None, mock.MagicMock(), mock.Mock())
        self._bench_params = {}
        time.sleep = mock.MagicMock()
        self._sut._iocard.get_bench_params().get_param_value.return_value = "USB_RLY08"
        self._sut._bench_params.has_parameter = mock.MagicMock(side_effect=self._side_effect)
        self._sut._bench_params.get_param_value = mock.MagicMock(side_effect=self._side_effect)

    def test_invalid_iocard(self):
        self._sut._iocard.get_bench_params().get_param_value.return_value = ""
        with self.assertRaisesRegexp(TestEquipmentException, "IOCard Model shall be USB_RLY08 or USBRELAY32"):
            self._sut.init()

    def test_bdaddress_found(self):
        self._bench_params = {"BD_Address": self.BDADDRESS}
        self._sut.init()
        self.assertEqual(self.BDADDRESS, self._sut._bdaddress)

    def test_bdaddress_not_found(self):
        self._sut.init()
        self.assertEqual("00:00:00:00:00:00", self._sut._bdaddress)

    def test_get_bdaddress_from_bench(self):
        self._bench_params = {"BD_Address": self.BDADDRESS}
        self._sut.init()
        self.assertEqual(self.BDADDRESS, self._sut.get_bdaddress())

    def test_get_bdaddress_default(self):
        self._sut.init()
        self.assertEqual("00:00:00:00:00:00", self._sut.get_bdaddress())

    def test_set_power_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "powerButton/powerOnTimer/powerOffTimer not configured"):
            self._sut.set_power(True)

    def _assert_test_power(self, power, duration):
        self._set_buttons_info({"powerButton": "1", "powerOnTimer": str(duration), "powerOffTimer": str(duration)})
        self._sut.init()
        self._sut.set_power(power)
        self._assert_iocard_line_activated(1, duration)

    def test_set_power_true(self):
        self._assert_test_power(True, 1)

    def test_set_power_false(self):
        self._assert_test_power(False, 2)

    def test_set_discoverable_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "powerButton/pairingTimer not configured"):
            self._sut.set_discoverable()

    def test_set_discoverable_ok(self):
        timer = 7
        self._set_buttons_info({"powerButton": "1", "pairingTimer": str(timer)})
        self._sut.init()
        self._sut.set_discoverable()
        self._assert_iocard_line_activated(1, timer)

    def test_set_volume_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "volUpButton/volDownButton/defaultShortKeyPressTimer not configured"):
            self._sut.set_volume(True)

    def test_set_volum_up(self):
        timer = 1
        self._set_buttons_info({"volUpButton": "1", "volDownButton": "2", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.set_volume(True)
        self._assert_iocard_line_activated(1, timer)

    def test_set_volum_down(self):
        timer = 1
        self._set_buttons_info({"volUpButton": "1", "volDownButton": "2", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.set_volume(False)
        self._assert_iocard_line_activated(2, timer)

    def test_pickup_call_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "callButton/defaultShortKeyPressTimer not configured"):
            self._sut.pickup_call()

    def test_pick_call_ok(self):
        timer = 1
        self._set_buttons_info({"callButton": "5", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.pickup_call()
        self._assert_iocard_line_activated(5, timer)

    def test_redial_call_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "callButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured"):
            self._sut.redial_call()

    def test_redial_call_ok(self):
        timer = 1
        self._set_buttons_info({"callButton": "5", "defaultShortKeyPressTimer": str(timer),
                                "betweenDoubleKeyPressTimer": "0.0"})
        self._sut.init()
        self._sut.redial_call()
        self._assert_iocard_line_activated(5, timer)

    def test_voice_dial_call_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "callButton/voiceDialTimer not configured"):
            self._sut.voice_dial_call()

    def test_voice_dial_call_ok(self):
        timer = 1
        self._set_buttons_info({"callButton": "5", "voiceDialTimer": str(timer)})
        self._sut.init()
        self._sut.voice_dial_call()
        self._assert_iocard_line_activated(5, timer)

    def test_hangup_call_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "callButton/defaultShortKeyPressTimer not configured"):
            self._sut.hangup_call()

    def test_hangup_call_ok(self):
        timer = 1
        self._set_buttons_info({"callButton": "5", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.hangup_call()
        self._assert_iocard_line_activated(5, timer)

    def test_reject_call_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "callButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured"):
            self._sut.reject_call()

    def test_reject_call_ok(self):
        timer = 1
        self._set_buttons_info({"callButton": "4", "defaultShortKeyPressTimer": str(timer),
                                "betweenDoubleKeyPressTimer": "0.0"})
        self._sut.init()
        self._sut.reject_call()
        self._assert_iocard_line_activated(4, timer)

    def test_play_pause_music_toggle_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "playPauseButton/defaultShortKeyPressTimer not configured"):
            self._sut.play_pause_music_toggle()

    def test_play_pause_music_toggle_ok(self):
        timer = 0.0
        self._set_buttons_info({"playPauseButton": "1", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.play_pause_music_toggle()
        self._assert_iocard_line_activated(1, timer)

    def test_stop_music_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "playPauseButton/defaultLongKeyPressTimer not configured"):
            self._sut.stop_music()

    def test_stop_music_ok(self):
        timer = 0.0
        self._set_buttons_info({"playPauseButton": "1", "defaultLongKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.stop_music()
        self._assert_iocard_line_activated(1, timer)

    def test_next_audio_track_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException, "fwdButton/defaultShortKeyPressTimer not configured"):
            self._sut.next_audio_track()

    def test_next_audio_track_ok(self):
        timer = 1.3
        self._set_buttons_info({"fwdButton": "7", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.next_audio_track()
        self._assert_iocard_line_activated(7, timer)

    def test_previous_audio_track_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "rwdButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured"):
            self._sut.previous_audio_track()

    def test_previous_audio_track_ok(self):
        timer = 0.5
        self._set_buttons_info({"rwdButton": "4", "defaultShortKeyPressTimer": str(timer),
                                "betweenDoubleKeyPressTimer": "0.0"})
        self._sut.init()
        self._sut.previous_audio_track()
        self._assert_iocard_line_activated(4, timer)

    def test_fforward_audio_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "fwdButton/defaultShortKeyPressTimer not configured"):
            self._sut.fforward_audio(None)

    def test_fforward_audio_with_duration(self):
        timer = 10
        self._set_buttons_info({"fwdButton": "7", "defaultShortKeyPressTimer": "0"})
        self._sut.init()
        self._sut.fforward_audio(timer)
        self._assert_iocard_line_activated(7, timer)

    def test_fforward_audio_with_default_duration(self):
        timer = 5
        self._set_buttons_info({"fwdButton": "7", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.fforward_audio()
        self._assert_iocard_line_activated(7, timer)

    def test_frewind_audio_error(self):
        self._sut.init()
        with self.assertRaisesRegexp(TestEquipmentException,
                                     "rwdButton/defaultShortKeyPressTimer not configured"):
            self._sut.frewind_audio(None)

    def test_frewind_audio_with_duration(self):
        timer = 10
        self._set_buttons_info({"rwdButton": "8", "defaultShortKeyPressTimer": "0"})
        self._sut.init()
        self._sut.frewind_audio(timer)
        self._assert_iocard_line_activated(8, timer)

    def test_frewind_audio_with_default_duration(self):
        timer = 5
        self._set_buttons_info({"rwdButton": "8", "defaultShortKeyPressTimer": str(timer)})
        self._sut.init()
        self._sut.frewind_audio()
        self._assert_iocard_line_activated(8, timer)
