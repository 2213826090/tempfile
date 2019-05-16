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
:summary: This file implements a Test Step to press the buttons of a bluetooth headset.
:since 15/07/2013
:author: fbongiax
"""

from time import sleep
from acs_test_scripts.TestStep.Equipment.Bluetooth.HeadSetBase import HeadSetBase


class PressHeadsetButtons(HeadSetBase):
    """
    Implements a test step to click buttons on a HeadSet.
    The buttons to click is given to the text step as a sequence comma
    separated. It also takes a wait parameters that express the time to
    wait between one button and the next one.
    """

    STR_AVRCP = "AVRCP"

    # AVRCP buttons
    STR_PLAY = "PLAY"
    STR_PAUSE = "PAUSE"
    STR_STOP = "STOP"
    STR_BACKWARD = "BACKWARD"
    STR_FORWARD = "FORWARD"
    STR_VOL_UP = "VOLUMEUP"
    STR_VOL_DOWN = "VOLUMEDOWN"
    STR_REWIND = "REWIND"
    STR_FFORWARD = "FASTFORWARD"
    STR_CALL = "CALL"
    STR_RECONNECT = 'RECONNECT'

    STR_BUTTONS_SEP = ","

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        HeadSetBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._commands = None

    def run(self, context):
        """
        Execute the test step
        @see BtHeadSetBase
        """

        HeadSetBase.run(self, context)

        assert isinstance(self._pars.wait_for, float), "wait_for should have been converted to float by the framework"

        self._configure_as_avrcp()

        buttons = self._pars.buttons.upper().split(self.STR_BUTTONS_SEP)
        for button in buttons:
            assert button in [self.STR_PLAY, self.STR_PAUSE, self.STR_STOP, self.STR_BACKWARD, self.STR_FORWARD,
                              self.STR_FFORWARD, self.STR_REWIND, self.STR_VOL_UP, self.STR_VOL_DOWN, self.STR_CALL, self.STR_RECONNECT], \
                              "button (%s) value should have been checked by the framework" % button
            sleep(self._pars.wait_for)
            self._logger.info("Press %s button", button)
            self._commands[button]()

    def _configure_as_avrcp(self):
        """
        Configure the test step to accept / issue AVRCP buttons events
        """

        self._commands = {self.STR_PLAY: self._api.play_pause_music_toggle, \
                    self.STR_PAUSE: self._api.play_pause_music_toggle, \
                    self.STR_STOP: self._api.stop_music, \
                    self.STR_BACKWARD: self._api.backward_audio_track, \
                    self.STR_FORWARD: self._api.next_audio_track, \
                    self.STR_VOL_UP: self._volume_up, \
                    self.STR_VOL_DOWN: self._volume_down, \
                    self.STR_REWIND: self._api.frewind_audio, \
                    self.STR_FFORWARD: self._api.fforward_audio, \
                    self.STR_CALL: self._api.pickup_call,\
                    self.STR_RECONNECT: self._api.reconnect_profile}

    def _volume_up(self):
        """
        Command volume up button
        """

        self._api.set_volume(True)

    def _volume_down(self):
        """
        Command volume down button
        """

        self._api.set_volume(False)
