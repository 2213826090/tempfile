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
:summary: Generic BT Headset implementation
:since: 19/02/2013
:author: cmichelx
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.BTHeadset.Interface.IBTHeadset import IBTHeadset
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class GenericBTHeadset(EquipmentBase, IBTHeadset):

    """
    Class that implements GenericBTHeadset equipment
    """

    def __init__(self, name, model, eqt_params, bench_params, factory=None):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        EquipmentBase.__init__(self, name, model, eqt_params, factory)
        IBTHeadset.__init__(self)

        # instantiate IOCard
        iocard = str(bench_params.get_param_value("IOCard"))
        self._em = self._factory.create_equipment_manager()
        self._iocard = self._em.get_io_card(iocard)

        self._bench_params = bench_params
        self._power_scard = None
        self._powerbutton = None
        self._volupbutton = None
        self._voldownbutton = None
        self._callbutton = None
        self._fwdbutton = None
        self._rwdbutton = None
        self._playpausebutton = None
        self._powerontimer = None
        self._powerofftimer = None
        self._defaultshortkeypresstimer = None
        self._defaultlongkeypresstimer = None
        self._betweendoublekeypresstimer = None
        self._pairingtimer = None
        self._voicedialtimer = None
        self._bdaddress = None
        self._bt_power_status = None
        self._defaultreconnectprofiletimer = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("BT headset initialization")

        # Verify IOCard model is USBRLy08 or USBRELAY32
        if self._iocard.get_bench_params().get_param_value("Model") \
                != "USB_RLY08" and self._iocard.get_bench_params().get_param_value("Model") != "USBRELAY32":
            msg = "IOCard Model shall be USB_RLY08 or USBRELAY32"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

        if ((self._bench_params.has_parameter("BD_Address")) and
           (self._bench_params.get_param_value("BD_Address") != "")):
            self._bdaddress = \
                str(self._bench_params.get_param_value("BD_Address")).upper()
            self.get_logger().info("Set BD address to %s", self._bdaddress)
        else:
            self._bdaddress = "00:00:00:00:00:00"

        self._powerbutton = self._set_button_line("powerButton")
        self._volupbutton = self._set_button_line("volUpButton")
        self._voldownbutton = self._set_button_line("volDownButton")
        self._callbutton = self._set_button_line("callButton")
        self._fwdbutton = self._set_button_line("fwdButton")
        self._rwdbutton = self._set_button_line("rwdButton")
        self._playpausebutton = self._set_button_line("playPauseButton")
        self._powerontimer = self._set_button_timer("powerOnTimer")
        self._powerofftimer = self._set_button_timer("powerOffTimer")
        self._defaultshortkeypresstimer = self._set_button_timer("defaultShortKeyPressTimer")
        self._defaultlongkeypresstimer = self._set_button_timer("defaultLongKeyPressTimer")
        self._betweendoublekeypresstimer = self._set_button_timer("betweenDoubleKeyPressTimer")
        self._pairingtimer = self._set_button_timer("pairingTimer")
        self._voicedialtimer = self._set_button_timer("voiceDialTimer")
        self._defaultreconnectprofiletimer = self._set_button_timer("defaultReconnectProfileTimer")

    def _configure_button(self, name):
        """
        If information for the button exist in the bench config, returns it,
        otherwise it returns None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        if ((self._bench_params.has_parameter(name)) and
           (self._bench_params.get_param_value(name) != "")):
            return self._bench_params.get_param_value(name)
        return None

    def _set_button_line(self, name):
        """
        Return the rele line for the button with the given name if defined in the bench config or None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        value = self._configure_button(name)
        return int(value) if value else None

    def _set_button_timer(self, name):
        """
        Return the time to wait for the button with the given name between closing / opening the rele
        (to simulate the button click) if defined in the bench config or None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        value = self._configure_button(name)
        return float(value) if value else None

    def _wait_for_secs(self, secs):
        """
        Waits for "secs" seconds
        """
        time.sleep(secs)

    def _press_relay(self, line, duration):
        """
        Press slave IOcard relay during duration (in second)
        :type line: integer
        :param line: ID of the relay to press
        :type duration: integer
        :param duration: duration between press and release relay, in second
        """
        self._iocard.enable_line(line)
        self._wait_for_secs(duration)
        self._iocard.disable_line(line)

    def get_bdaddress(self):
        """
        Returns bd address parameter of the equipment
        :rtype: str
        :return: BD address of the equipment. format 00:00:00:00:00:00
        """
        assert self._bdaddress, "self._bdaddress cannot be empty or None at this stage"
        return str(self._bdaddress)

    def set_power(self, state=True):
        """
        Set BT headset power ON or OFF
        Control the on/off line
        :type state: boolean
        :param state: true to power ON, false to power OFF
        """
        self._raise_error_if_any_is_none([self._powerbutton, self._powerontimer, self._powerofftimer],
                                          "powerButton/powerOnTimer/powerOffTimer not configured; No action taken!")
        if state:
            self._press_relay(self._powerbutton, self._powerontimer)
        else:
            self._press_relay(self._powerbutton, self._powerofftimer)

    def set_discoverable(self):
        """
        Set BT headset in discoverable mode (able to be paired)
        Control the on/off line to switch to pairing mode
        """
        self._raise_error_if_any_is_none([self._powerbutton, self._pairingtimer],
                                          "powerButton/pairingTimer not configured; No action taken!")
        self._press_relay(self._powerbutton, self._pairingtimer)

    def set_volume(self, state):
        """
        Set Volume up/down depend on state value
        Control the volume up line
        :type state: boolean
        :param state: true to volume UP, false to volume DOWN
        """
        self._raise_error_if_any_is_none([self._volupbutton, self._voldownbutton, self._defaultshortkeypresstimer],
                            "volUpButton/volDownButton/defaultShortKeyPressTimer not configured; No action taken!")
        if state:
            self._press_relay(self._volupbutton, self._defaultshortkeypresstimer)
        else:
            self._press_relay(self._voldownbutton, self._defaultshortkeypresstimer)

    def pickup_call(self):
        """
        Control the call button line
        """
        self._raise_error_if_any_is_none([self._callbutton, self._defaultshortkeypresstimer],
                                      "callButton/defaultShortKeyPressTimer not configured; No action taken!")
        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)

    def redial_call(self):
        """
        Control the call button line with double short press
        to enable redial
        """
        self._raise_error_if_any_is_none([self._callbutton, self._defaultshortkeypresstimer,
                                          self._betweendoublekeypresstimer],
                    "callButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured; No action taken!")

        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)
        self._wait_for_secs(self._betweendoublekeypresstimer)
        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)

    def voice_dial_call(self):
        """
        Start voice recognition with a long press on call button line
        """
        self._raise_error_if_any_is_none([self._callbutton, self._voicedialtimer],
                                      "callButton/voiceDialTimer not configured; No action taken!")
        self._press_relay(self._callbutton, self._voicedialtimer)

    def hangup_call(self):
        """
        Control the call button line
        """
        self._raise_error_if_any_is_none([self._callbutton, self._defaultshortkeypresstimer],
                                      "callButton/defaultShortKeyPressTimer not configured; No action taken!")
        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)

    def reject_call(self):
        """
        Control the call button line with double short press
        to reject an incoming call
        """
        self._raise_error_if_any_is_none([self._callbutton, self._defaultshortkeypresstimer,
                                          self._betweendoublekeypresstimer],
                    "callButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured; No action taken!")

        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)
        self._wait_for_secs(self._betweendoublekeypresstimer)
        self._press_relay(self._callbutton, self._defaultshortkeypresstimer)

    def play_pause_music_toggle(self):
        """
        Control the play/pause line to start/pause playing audio
        """
        self._raise_error_if_any_is_none([self._playpausebutton, self._defaultshortkeypresstimer],
                                         "playPauseButton/defaultShortKeyPressTimer not configured; No action taken!")
        self._press_relay(self._playpausebutton, self._defaultshortkeypresstimer)

    def stop_music(self):
        """
        Control the play/pause line to stop playing audio
        """
        self._raise_error_if_any_is_none([self._playpausebutton, self._defaultlongkeypresstimer],
                                         "playPauseButton/defaultLongKeyPressTimer not configured; No action taken!")
        self._press_relay(self._playpausebutton, self._defaultlongkeypresstimer)

    def next_audio_track(self):
        """
        Control the fwd line to jump to next track
        """
        self._raise_error_if_any_is_none([self._fwdbutton, self._defaultshortkeypresstimer],
                                         "fwdButton/defaultShortKeyPressTimer not configured; No action taken!")
        self._press_relay(self._fwdbutton, self._defaultshortkeypresstimer)

    def previous_audio_track(self):
        """
        Control the rwd line to jump to previous track
        Press Rwd button twice
        """
        self._raise_error_if_any_is_none([self._rwdbutton, self._defaultshortkeypresstimer,
                                          self._betweendoublekeypresstimer],
                    "rwdButton/defaultShortKeyPressTimer/betweenDoubleKeyPressTimer not configured; No action taken!")

        self._press_relay(self._rwdbutton, self._defaultshortkeypresstimer)
        self._wait_for_secs(self._betweendoublekeypresstimer)
        self._press_relay(self._rwdbutton, self._defaultshortkeypresstimer)

    def backward_audio_track(self):
        """
        Control the rwd line to backward track
        """
        self._raise_error_if_any_is_none([self._rwdbutton, self._defaultshortkeypresstimer],
                                         "rwdButton/defaultShortKeyPressTimer not configured; No action taken!")
        self._press_relay(self._rwdbutton, self._defaultshortkeypresstimer)

    def _set_default_duration_if_needed(self, duration):
        if duration is None or duration < 1:
            duration = self._defaultshortkeypresstimer
        return duration

    def fforward_audio(self, duration=None):
        """
        Control the fwd line to go ahead in the audio track
        :type line: integer
        :param duration: the time we want to go ahead in the audio track
        """
        self._raise_error_if_any_is_none([self._fwdbutton, self._defaultshortkeypresstimer],
                                         "fwdButton/defaultShortKeyPressTimer not configured; No action taken!")
        duration = self._set_default_duration_if_needed(duration)
        self._press_relay(self._fwdbutton, duration)

    def frewind_audio(self, duration=None):
        """
        Control the rewind line to go back in the audio track
        :type line: integer
        :param duration: the time we want to go back in the audio track
        """
        self._raise_error_if_any_is_none([self._rwdbutton, self._defaultshortkeypresstimer],
                                         "rwdButton/defaultShortKeyPressTimer not configured; No action taken!")
        duration = self._set_default_duration_if_needed(duration)
        self._press_relay(self._rwdbutton, duration)

    def _raise_error_if_any_is_none(self, values, msg):
        """
        If any of the passed values is None it raises an exception with the given message
        :type values: array of objects
        :param values: list of values which can be None
        """
        for item in values:
            if item is None:
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def reconnect_profile(self):
        """
        Control the call button line to reconnect the headset to the paired device
        """
        self._raise_error_if_any_is_none([self._callbutton, self._defaultreconnectprofiletimer],
                                      "callButton/defaultReconnectProfileTimer not configured; No action taken!")
        self._press_relay(self._callbutton, self._defaultreconnectprofiletimer)
