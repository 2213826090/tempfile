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
:summary: unit test
:since 27/25/2014
:author: jfranchx
"""
import unittest
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.BT.BtControlA2DPMediaPlayer import BtControlA2DPMediaPlayer


class BtControlA2DPMediaPlayerTest(UTestTestStepBase):
    """
    ControlA2DPMediaPlayer test cases
    """
    # "PLAY","PAUSE","STOP","NEXT_TRACK","PREVIOUS_TRACK","VOLUMEDOWN","VOLUMEUP"]:

    def test_start_player(self):
        sut = self._create_sut({"ACTION_CONTROL": "START_PLAYER", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.start_a2dp_media_player.assert_called_with("/mypath/myfile.mp3", 300)

    def test_stop_player(self):
        sut = self._create_sut({"ACTION_CONTROL": "STOP_PLAYER", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.stop_a2dp_media_player.assert_called_with()

    def test_bad_action_control_parameter(self):
        sut = self._create_sut({"ACTION_CONTROL": "BAD_ACTION", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        self._assert_run_throw_config_exception(sut, "Error parameter ACTION_CONTROL is unknown : BAD_ACTION")

    def test_float_timeout_parameter(self):
        sut = self._create_sut({"ACTION_CONTROL": "START_PLAYER", "FILENAME": "myfile.mp3", "TIMEOUT":30.2})
        self._assert_run_throw_config_exception(sut, "Error parameter TIMEOUT is not integer : <type 'float'>")

    def test_command_play(self):
        sut = self._create_sut({"ACTION_CONTROL": "PLAY", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("PLAY")

    def test_command_pause(self):
        sut = self._create_sut({"ACTION_CONTROL": "PAUSE", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("PAUSE")

    def test_command_stop(self):
        sut = self._create_sut({"ACTION_CONTROL": "STOP", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("STOP")

    def test_command_volumeup(self):
        sut = self._create_sut({"ACTION_CONTROL": "VOLUMEUP", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("VOLUMEUP")

    def test_command_volumedown(self):
        sut = self._create_sut({"ACTION_CONTROL": "VOLUMEDOWN", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("VOLUMEDOWN")

    def test_command_previous_track(self):
        sut = self._create_sut({"ACTION_CONTROL": "PREVIOUS_TRACK", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("PREVIOUS_TRACK")

    def test_command_next_track(self):
        sut = self._create_sut({"ACTION_CONTROL": "NEXT_TRACK", "FILENAME": "myfile.mp3", "TIMEOUT":300})
        sut.run(None)
        sut._api.control_a2dp_media_player.assert_called_with("NEXT_TRACK")

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = BtControlA2DPMediaPlayer(None, None, test_step_pars, mock.Mock())
        sut._device.multimedia_path = "/mypath"
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
