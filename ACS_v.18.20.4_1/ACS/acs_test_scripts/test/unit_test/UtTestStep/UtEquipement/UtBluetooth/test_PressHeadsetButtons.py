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
from acs_test_scripts.TestStep.Equipment.Bluetooth.PressHeadsetButtons import PressHeadsetButtons


class PressHeadsetButtonsTest(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_press_play_ok(self):
        sut = self._create_sut("PLAY")
        sut.run(None)
        sut._api.play_pause_music_toggle.assert_called_with()

    def test_press_pause_ok(self):
        sut = self._create_sut("PAUSE")
        sut.run(None)
        sut._api.play_pause_music_toggle.assert_called_with()

    def test_press_stop_ok(self):
        sut = self._create_sut("STOP")
        sut.run(None)
        sut._api.stop_music.assert_called_with()

    def test_press_backward_ok(self):
        sut = self._create_sut("BACKWARD")
        sut.run(None)
        sut._api.backward_audio_track.assert_called_with()

    def test_press_forward_ok(self):
        sut = self._create_sut("FORWARD")
        sut.run(None)
        sut._api.next_audio_track.assert_called_with()

    def test_press_fforward_ok(self):
        sut = self._create_sut("FASTFORWARD")
        sut.run(None)
        sut._api.fforward_audio.assert_called_with()

    def test_press_rewind_ok(self):
        sut = self._create_sut("REWIND")
        sut.run(None)
        sut._api.frewind_audio.assert_called_with()

    def test_volume_up_ok(self):
        sut = self._create_sut("VOLUMEUP")
        sut.run(None)
        sut._api.set_volume.assert_called_with(True)

    def test_volume_down_ok(self):
        sut = self._create_sut("VOLUMEDOWN")
        sut.run(None)
        sut._api.set_volume.assert_called_with(False)

    def test_call_ok(self):
        sut = self._create_sut("CALL")
        sut.run(None)
        sut._api.pickup_call.assert_called_with()

    def test_buttons_case_insensitive_ok(self):
        sut = self._create_sut("voLuMEdoWN")
        sut.run(None)
        sut._api.set_volume.assert_called_with(False)

    def test_buttons_sequence_ok(self):
        sut = self._create_sut("PLAY,PAUSE,STOP")
        sut.run(None)
        calls = sut._api.method_calls
        self.assertEqual("play_pause_music_toggle", calls[0][0])
        self.assertEqual("play_pause_music_toggle", calls[1][0])
        self.assertEqual("stop_music", calls[2][0])

    def _create_sut(self, buttons):
        self._sut = PressHeadsetButtons(None, mock.Mock(), {"BUTTONS": buttons, "WAIT_FOR": 0.0}, mock.Mock())
        return self._sut
