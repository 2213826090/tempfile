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
:summary: This script implements unitary actions for voice call features
:since: 19/07/2010
:author: asebbane
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IVoiceCall():

    """
    Abstract class that defines the interface to be implemented
    by voice call handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, phone):
        """
        Initializes this instances.

        Nothing to be done in abstract class.
        """
        pass

    def dial(self, number_to_call, check_state=True, single_dial=False, call_type=None):
        """
        Dials a voice call.

        :type number_to_call: str
        :param number_to_call: number to call (MSISDN)

        :type check_state: boolean
        :param check_state: check call state or not.

        :type single_dial: bool
        :param single_dial: dial a single key for DTMF generating purpose

        :type call_type: str
        :param call_type: (optional) the IMS call type which has to be one of:
            - IR94_AUDIO
            - IR94_RX
            - IR94_TX
            - IR94_BIDIRECTIONAL

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def emergency_dial(self, number_to_call, check_state=True):
        """
        Dials an emergency call.
        this uecmd Will consider that the number you enter is an emergency number and will dial it.

        :type number_to_call: str
        :param number_to_call: number to call (MSISDN)

        :type check_state: boolean
        :param check_state: check call state or not.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def release(self, check_state=True):
        """
        Releases all voice calls

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def answer(self):
        """
        Answers all voice calls.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_state(self):
        """
        Returns voice call state.

        :rtype: object
        :return: A value of VOICE_CALL_STATE.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_state(self, state, timeout):
        """
        Waits to reach a voice call state until a timeout.

        :type state: UECmd.VOICE_CALL_STATE
        :param state: expected state (see UECmd.VOICE_CALL_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_state(self, state):
        """
        Checks whether the state of a voice call matches the given one.

        :type state: UECmd.VOICE_CALL_STATE
        :param state: expected state (see UECmd.VOICE_CALL_STATE)

        :rtype: boolean
        :return: true if state equals elsewhere false
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_last_call_details(self):
        """
        Returns the last incoming call details (sim, incoming number, and call
        type).

        :rtype: tuple
        :return: A phone number on international format (+33xxxxxxxxx), the
        call type UECmd.VOICE_CALL_TYPE and used SIM.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_audio_state(self):
        """
        Returns audio state.

        :rtype: object
        :return: A value of AUDIO_STATE.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_audio_state(self, state, timeout):
        """
        Waits to reach a voice call state until a timeout.

        :type state: UECmd.AUDIO_STATE
        :param state: expected state (see UECmd.AUDIO_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def set_mute(self, muted):
        """
        Mutes or unmutes the microphone for the active call

        @param muted: muted true to mute the microphone, false to activate the microphone.
        @type muted: int

        @rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_holding_and_active(self):
        """
        Places any active calls on hold, and makes any held calls active.

        @rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_call_forwarding(self, mode, phone_number):
        """
        Enable call forwarding

        :param mode: Mode for call forwarding request
        :type mode: str
        :param phone_number: Phone number used for call forwarding request
        :type phone_number: str

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_call_forwarding(self, mode):
        """
        Disable call forwarding

        :param mode: Mode for call forwarding request
        :type mode: str

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_call_waiting(self):
        """
        Enable call waiting

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_call_waiting(self):
        """
        Disable call waiting

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_call_forwarding_state(self, expected_state):
        """
        Check the call forwarding status

        :param expected_state: The expected call forwarding state
        :type expected_state: str

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_foreground_call_state(self):
        """
        Returns the status of the foreground call.

        :rtype: VOICE_CALL_STATE
        :return: the call status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_background_call_state(self):
        """
        Returns the status of the background call.

        :rtype: VOICE_CALL_STATE
        :return: the call status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_call_barring(self, mode, barring_code):
        """
        Enable call barring

        :param mode: Mode for call barring request
        :type mode: str
        :param barring_code: Code used for call barring request
        :type barring_code: str

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_call_barring(self, mode, barring_code):
        """
        Disable call barring

        :param mode: Mode for call barring request
        :type mode: str
        :param barring_code: Code used for call barring request
        :type barring_code: str

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def answer_ims_video_call(self):
        """
        Answer using ADB shell command to an IR94 video call

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
