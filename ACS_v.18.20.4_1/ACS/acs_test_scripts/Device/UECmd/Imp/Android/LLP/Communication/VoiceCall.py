"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file implements the VoiceCall UEcmd for Android LLP devices
:since: 17/06/2015
:author: mariussx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Communication.VoiceCall import VoiceCall as VoiceCallCommon
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE

from ErrorHandling.DeviceException import DeviceException

class VoiceCall(VoiceCallCommon):
    """
    :summary: VoiceCall UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    @need('modem')
    def __init__(self, phone):
        """
        Constructor.
        """
        VoiceCallCommon.__init__(self, phone)

        # pylint: disable=E1101
        self._vc_state = {
            "no_call": VOICE_CALL_STATE.NOCALL,
            "on_hold": VOICE_CALL_STATE.ON_HOLD,
            "ringing": VOICE_CALL_STATE.INCOMING,
            "dialing": VOICE_CALL_STATE.DIALING,
            "alerting": VOICE_CALL_STATE.ALERTING,
            "active": VOICE_CALL_STATE.ACTIVE,
            "disconnected": VOICE_CALL_STATE.DISCONNECTED,
            "unknown": VOICE_CALL_STATE.UNKNOWN}

    #def wait_for_state(self, state, timeout):
    #    """
    #    Waits to reach a voice call state until a timeout.

    #    :type state: UECmd.VOICE_CALL_STATE
    #    :param state: expected state (see UECmd.VOICE_CALL_STATE)

    #    :type timeout: int
    #    :param timeout: maximum time to wait in seconds

    #    :return: None
    #    """
    #    self._logger.info(
    #        "Waiting for %s state before %d seconds...", state, timeout)
    #    time_count = 0
    #    read_state = "UNKNOWN"
    #    state_reached = False

    #    while not state_reached and time_count <= timeout:
    #        time_count += 1
    #        # For 'dialing', 'alerting' and 'active' states, retrieve the status from foreground call
    #        if state in (VOICE_CALL_STATE.DIALING, VOICE_CALL_STATE.ALERTING, VOICE_CALL_STATE.ACTIVE):
    #            read_state = self.get_foreground_call_state()
    #        # For 'on hold' state, retrieve the status from background call
    #        elif state == VOICE_CALL_STATE.ON_HOLD:
    #            read_state = self.get_background_call_state()
    #        else:
    #            read_state = self.get_state()
    #
    #        self._logger.info("State is %s !" % (str(read_state)))
    #        if read_state == state:
    #            state_reached = True
    #
    #    if not state_reached:
    #        err_msg = "Did not reach %s state before %ds." % (str(state), timeout)
    #        self._logger.error(err_msg)
    #        raise DeviceException(DeviceException.TIMEOUT_REACHED, err_msg)
    #    else:
    #        self._logger.info("Reach State: %s, in %ds." % (str(read_state), time_count))
    #
    #def check_state(self, state):
    #    """
    #    Checks whether the state of a voice call matches the given one.
    #
    #    :type state: UECmd.VOICE_CALL_STATE
    #    :param state: expected state (see UECmd.VOICE_CALL_STATE)
    #
    #    :rtype: None
    #    :return: None
    #    """
    #    # For 'dialing', 'alerting' and 'active' states, retrieve the status from foreground call
    #    if state in (VOICE_CALL_STATE.DIALING, VOICE_CALL_STATE.ALERTING, VOICE_CALL_STATE.ACTIVE):
    #        current_state = self.get_foreground_call_state()
    #    # For 'on hold' state, retrieve the status from background call
    #    elif state == VOICE_CALL_STATE.ON_HOLD:
    #        current_state = self.get_background_call_state()
    #    else:
    #        current_state = self.get_state()
    #
    #    if current_state == state:
    #        msg = "Current state matches the expected state (%s)" % (str(current_state))
    #        self._logger.info(msg)
    #    else:
    #        err_msg = "Current state (%s) did not match expected state (%s)!" % (str(current_state), str(state))
    #        self._logger.error(err_msg)
    #        raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)
    #
    def get_foreground_call_state(self):
        """
        Returns the status of the foreground call.

        :rtype: VOICE_CALL_STATE
        :return: the call status
        """
        method = "getForegroundCallState"
        output = self._internal_exec_v2(self._callcommand_module, method, is_system=True)
        state = str(output["foreground_call_state"])

        return self._vc_state[state]

    def get_background_call_state(self):
        """
        Returns the status of the background call.

        :rtype: VOICE_CALL_STATE
        :return: the call status
        """
        method = "getBackgroundCallState"
        output = self._internal_exec_v2(self._callcommand_module, method, is_system=True)
        state = str(output["background_call_state"])

        return self._vc_state[state]

    def answer_ims_video_call(self):
        """
        Answer using ADB shell command to an IR94 video call
        :return: None
        """

        self._logger.info("Answering to an incoming IMS voide call...")
        # Define the key code for answering
        KEYCODE_HEADSETHOOK = "79"
        # Execute ADB command
        self._exec("adb shell input keyevent " + KEYCODE_HEADSETHOOK, 2, True, False)

        # Wait for the expected state
        self.wait_for_state(self._vc_state.get("active"), self._call_setup_timeout)