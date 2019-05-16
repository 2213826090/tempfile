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
:summary: This file implements the VoiceCall UEcmd for Android phone
:since: 06/04/2011
:author: dgonzalez
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Misc.PhoneSystem import PhoneSystem
from acs_test_scripts.Device.UECmd.Interface.Communication.IVoiceCall import IVoiceCall
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE
from acs_test_scripts.Device.UECmd.UECmdTypes import AUDIO_STATE
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd import UECmdTypes
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class VoiceCall(BaseV2, IVoiceCall):
    """
    :summary: VoiceCall UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    @need('modem')
    def __init__(self, phone):
        """
        Constructor.

        """
        BaseV2.__init__(self, phone)
        IVoiceCall.__init__(self, phone)
        self._logger = phone.get_logger()
        self._phone_system = PhoneSystem(phone)
        self._call_setup_timeout = phone.get_call_setup_timeout()
        self.component = "com.intel.acs.agent/.VoiceCall"
        # pylint: disable=E1101
        self._vc_state = {
            "no_call": VOICE_CALL_STATE.NOCALL,
            "ringing": VOICE_CALL_STATE.INCOMING,
            "active": VOICE_CALL_STATE.ACTIVE}
        # pylint: disable=E1101
        self._audio_state = {
            "invalid": AUDIO_STATE.INVALID,
            "current": AUDIO_STATE.CURRENT,
            "in_call": AUDIO_STATE.IN_CALL,
            "in_sip_call": AUDIO_STATE.IN_COMMUNICATION,
            "no_call": AUDIO_STATE.NORMAL,
            "ringing": AUDIO_STATE.RINGTONE}
        # pylint: enable=E1101

        self.KEYCODE_DIAL_PAD = {"0": "7",
                                 "1": "8",
                                 "2": "9",
                                 "3": "10",
                                 "4": "11",
                                 "5": "12",
                                 "6": "13",
                                 "7": "14",
                                 "8": "15",
                                 "9": "16",
                                 "*": "17",
                                 "#": "18"}
        self._ir94_extras = {
            "IR94_AUDIO": 0,
            "IR94_RX": 1,
            "IR94_TX": 2,
            "IR94_BIDIRECTIONAL": 3}

        self._voicecall_module = "acscmd.telephony.voicecall.VoiceCallModule"
        self._audio_module = "acscmd.audio.AudioOutputModule"
        self._callcommand_module = "acscmd.telephony.TelephonyModule"

    def _dial_ims(self, number_to_call, check_state, call_type):
        """
        Initiates an IMS call.

        :type number_to_call: str
        :param number_to_call: number to call (MSISDN)

        :type check_state: bool
        :param check_state: check call state or not.

        :type call_type: str
        :param call_type: the IMS call type which has to be one of:
            - IR94_AUDIO
            - IR94_RX
            - IR94_TX
            - IR94_BIDIRECTIONAL

        :raise DeviceException - INVALID_PARAMETER: If the "call_type" parameter is invalid

        :return: None
        """

        # Build the ADB command
        adb_command = "adb shell am start -a %s -d tel:%s" % (
            "android.intent.action.CALL_PRIVILEGED",
            number_to_call)

        # Add the extras specific for IR94
        if call_type in self._ir94_extras:
            # Initialize some local variables
            extra_template = "--ei android.telecom.extra.START_CALL_WITH_VIDEO_STATE"
            # Build the Intent extra
            extra = "%s %d" % (extra_template, self._ir94_extras[call_type])
            adb_command += " %s" %(extra)

        # Run the commandndroid.intent.action.CALL_PRIVILEGED
        self._exec(adb_command)

        # Wait for VOICE_CALL_STATE.ACTIVE state
        if check_state:
            self.wait_for_state(self._vc_state.get("active"),
                                self._call_setup_timeout)


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

        :raise DeviceException - INVALID_PARAMETER: If the "call_type" parameter is invalid

        :return: None
        """
        # If call_type has been provided then we are trying to make an IMS call
        if call_type == "IR92" or call_type in self._ir94_extras:
            self._dial_ims(number_to_call, check_state, call_type)
            return
        # Otherwise proceed with other call types
        emergency_number = False
        if single_dial:
            self._logger.info("Dial %s", number_to_call)
            cmd = "adb shell input keyevent " + self.KEYCODE_DIAL_PAD[number_to_call]
            self._exec(cmd)
        else:
            if number_to_call in UECmdTypes.EMERGENCY_NUMBERS_LIST:

                emergency_number = True
                # Unlock the phone
                self._logger.info("Unlocking phone ...")
                self._phone_system.set_phone_lock(0)

                self._logger.info("Dialing emergency number %s ...", number_to_call)

                cmd1 = "adb shell am start -a " \
                       + "android.intent.action.CALL_EMERGENCY tel://" + str(number_to_call)
                self._exec(cmd1)
            else:
                self._logger.info("Dialing %s ...", number_to_call)
                cmd = "adb shell am start -a android.intent.action.CALL tel:" + number_to_call
                self._exec(cmd)

            if self.get_state() == VOICE_CALL_STATE.NOCALL:
                # in some case the call may not be establish, push the call button
                cmd2 = "adb shell input keyevent KEYCODE_CALL"
                self._exec(cmd2)

        # Wait for VOICE_CALL_STATE.ACTIVE state
        if check_state:
            self.wait_for_state(self._vc_state.get("active"),
                                self._call_setup_timeout)

        if emergency_number:
            # Re-enable phone lock after the call
            self._logger.info("Re-enable phone locking")
            self._phone_system.set_phone_lock(1)

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
        self._logger.info("Dialing emergency number %s ...", number_to_call)
        cmd1 = "adb shell am start -a android.intent.action.CALL_EMERGENCY tel://" + str(number_to_call)
        self._exec(cmd1)
        # Wait for VOICE_CALL_STATE.ACTIVE state
        if check_state:
            self.wait_for_state(self._vc_state.get("active"),
                                self._call_setup_timeout)

    def get_state(self):
        """
        Returns voice call state.

        :rtype: object
        :return: A value of VOICE_CALL_STATE.
        """
        function = "getCallState"
        results = self._internal_exec_v2(self._voicecall_module, function, is_system=True)
        state = str(results["call_state"])

        return self._vc_state[state]

    def release(self, check_state=True):
        """
        Releases all voice calls

        :return: None
        """
        # Create the proper adb command
        cmd = "adb shell input keyevent KEYCODE_ENDCALL"
        # Run the command
        self._logger.info("Release all calls...")
        self._exec(cmd)

        # If we are not in IDLE state after timeout
        # wait_for_state will throw an exception
        if check_state:
            self.wait_for_state(self._vc_state.get("no_call"), self._call_setup_timeout)

    def answer(self):
        """
        Answers all voice calls.

        :return: None
        """
        # Create the proper adb command
        cmd = "adb shell input keyevent KEYCODE_CALL"
        # Run the command
        self._logger.info("Answering incoming call...")
        self._exec(cmd)

        #Wait for the expected state
        self.wait_for_state(self._vc_state.get("active"), self._call_setup_timeout)

    def wait_for_state(self, state, timeout):
        """
        Waits to reach a voice call state until a timeout.

        :type state: UECmd.VOICE_CALL_STATE
        :param state: expected state (see UECmd.VOICE_CALL_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        """
        self._logger.info(
            "Waiting for %s state before %d seconds...", state, timeout)
        time_count = 0
        read_state = "UNKNOWN"
        state_reached = False

        while not state_reached and time_count <= timeout:
            time_count += 1
            read_state = self.get_state()
            self._logger.info("State is %s !" % (str(read_state)))
            if read_state == state:
                state_reached = True

        if not state_reached:
            err_msg = "Did not reach %s state before %ds." % (str(state), timeout)
            self._logger.error(err_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, err_msg)
        else:
            self._logger.info("Reach State: %s, in %ds." % (str(read_state), time_count))

    def check_state(self, state):
        """
        Checks whether the state of a voice call matches the given one.

        :type state: UECmd.VOICE_CALL_STATE
        :param state: expected state (see UECmd.VOICE_CALL_STATE)

        :rtype: None
        :return: None
        """
        current_state = self.get_state()

        if current_state == state:
            msg = "Current state matches the expected state (%s)" % (str(current_state))
            self._logger.info(msg)
        else:
            err_msg = "Current state (%s) did not match expected state (%s)!" % (str(current_state), str(state))
            self._logger.error(err_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)

    def set_ringtone(self, ringtone_name):
        """
        Set Ringtone of the phone

        :type ringtone_name: str
        :param ringtone_name: the name of ringtone in the phone.

        """
        self._logger.info("Set Ringtone: %s" % ringtone_name)
        function = "setRingtone"
        self._internal_exec_v2(
            self._voicecall_module,
            function,
            "--es ringtone_name %s" % ringtone_name,
            is_system=True)

    def get_last_call_details(self):
        """
        Returns the last incoming call details (sim, incoming number, and call
        type).

        :rtype: tuple
        :return: A phone number on international format (+33xxxxxxxxx), the
        call type UECmd.VOICE_CALL_TYPE and used SIM.
        """
        self._logger.info("Getting last call information.")
        function = "getLastCallDetails"

        results = self._internal_exec_v2(self._voicecall_module, function, is_system=True)

        used_sim = str(results["used_sim"])
        dialed_number = str(results["dialed_number"])
        call_type = str(results["call_type"])
        self._logger.debug("Last call number: %s, type: %s and SIM: %s" % (dialed_number, call_type, used_sim))

        if call_type not in UECmdTypes.VOICE_CALL_TYPE.values():
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "The returned call_type should be part of the VOICE_CALL_TYPE enum: is %s"
                                  % call_type)

        return dialed_number, call_type, used_sim

    def get_audio_state(self):
        """
        Returns audio state.

        :rtype: object
        :return: A value of AUDIO_STATE.
        """
        function = "getAudioState"
        results = self._internal_exec_v2(self._audio_module, function, is_system=True)
        state = str(results["audio_state"])

        return self._audio_state[state]

    def wait_for_audio_state(self, state, timeout):
        """
        Waits to reach a voice call state until a timeout.

        :type state: UECmd.AUDIO_STATE
        :param state: expected state (see UECmd.AUDIO_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        """
        self._logger.info(
            "Waiting for %s state before %d seconds...", state, timeout)
        time_count = 0
        read_state = "UNKNOWN"
        state_reached = False

        while (state_reached == False) and time_count <= timeout:
            time_count += 1
            read_state = self.get_audio_state()
            self._logger.info("State is %s !" % (str(read_state)))

            if read_state == state:
                self._logger.info("State %s has been reached!" % (str(state)))
                state_reached = True

        if not state_reached:
            err_msg = "Did not reach %s state" % (str(state))
            raise AcsBaseException(
                AcsBaseException.TIMEOUT_REACHED,
                err_msg)

    def set_mute(self, muted):
        """
        Mutes or unmutes the microphone for the active call

        @param muted: muted true to mute the microphone, false to activate the microphone.
        @type muted: int

        @rtype: None
        """
        method = "setMute"
        method_args = "--ei muted %s" % (int(muted))
        if muted == 1:
            self._logger.info("Mute the current call")
        else:
            self._logger.info("Unmute the current call")

        result = self._internal_exec_v2(self._callcommand_module, method, method_args, is_system=True)

        if result["output"] is None:
            return "No error"
        else:
            return result["output"]

    def switch_holding_and_active(self):
        """
        Places any active calls on hold, and makes any held calls active.

        @rtype: None
        """
        method = "switchHoldingAndActive"
        self._logger.info("Hold the current call")
        result = self._internal_exec_v2(self._callcommand_module, method, is_system=True)

        if result["output"] is None:
            return "No error"
        else:
            return result["output"]

    def enable_call_forwarding(self, mode, phone_number):
        """
        Enable call forwarding

        :param mode: Mode for call forwarding request
        :type mode: str
        :param phone_number: Phone number used for call forwarding request
        :type phone_number: str

        :rtype: None
        :raise DeviceException - INVALID_PARAMETER: If the "mode" parameter is invalid
        """

        if not mode or mode.lower() not in ("unconditional", "busy", "no_answer", "not_reachable"):
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value",
                    str(mode),
                    "for parameter 'mode'."))
        if not phone_number:
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value",
                    str(phone_number),
                    "for parameter 'phone_number'."))

        self._logger.info("Enable call forwarding.")

        method = "enableCallForwarding"
        args = "--es mode %s --es number %s" % (
            mode,
            phone_number)
        self._internal_exec_v2(self._voicecall_module, method, args, is_system=True)

    def enable_call_waiting(self):
        """
        Enable call waiting

        :rtype: None
        """
        method = "EnableCallWait"
        self._internal_exec_v2(self._voicecall_module, method,is_system=True)
        self._logger.info("Call waiting Enabled ")

    def disable_call_forwarding(self, mode):
        """
        Disable call forwarding

        :param mode: Mode for call forwarding request
        :type mode: str

        :rtype: None
        :raise DeviceException - INVALID_PARAMETER: If the "mode" parameter is invalid
        """

        if not mode or mode.lower() not in ("unconditional", "busy", "no_answer", "not_reachable"):
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value ",
                    str(mode),
                    " for parameter 'mode'."))

        self._logger.info("Disable call forwarding for '%s' mode" % mode)

        method = "disableCallForwarding"
        args = "--es mode %s" % mode
        self._internal_exec_v2(self._voicecall_module, method, args, is_system=True)

    def disable_call_waiting(self):
        """
        Disable call waiting

        :rtype: None
        """
        method = "DisableCallWait"
        self._internal_exec_v2(self._voicecall_module, method ,is_system=True)
        self._logger.info("Call waiting disabled")

    def check_call_forwarding_state(self, expected_state):
        """
        Check the call forwarding status

        :param expected_state: The expected call forwarding state
        :type expected_state: str

        :return: None

        :raise DeviceException - INVALID_PARAMETER: If the "wanted_state" parameter is invalid
        :raise DeviceException - INVALID_DEVICE_STATE: If the states doesn't match
        """

        if expected_state in ("1", "enabled", "on"):
            expected_state = "1"
        elif expected_state in ("0", "disabled", "off"):
            expected_state = "0"
        else:
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value ",
                    expected_state,
                    " for parameter 'wanted_state'."))

        method = "getCallForwardingState"
        result = self._internal_exec_v2(self._callcommand_module, method, is_system=True)
        state = result["call_forward_status"]

        if expected_state == state:
            msg = "Current state matches the expected state \"(%s)\"." % (expected_state)
            self._logger.info(msg)
        else:
            err_msg = "Current state \"(%s)\" did not match expected state \"(%s)\" !" % (expected_state, state)
            self._logger.error(err_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  err_msg)

    def enable_call_barring(self, mode, barring_code=""):
        """
        Enable call barring

        :param mode: Mode for call barring request
        :type mode: str
        :param barring_code: Code used for call barring request
        :type barring_code: str

        :rtype: None
        :raise DeviceException - INVALID_PARAMETER: If the "mode" parameter is invalid
        """

        """
        "BAOC" - Barring of All Outgoing Calls
        "BOIC" - Barring of Outgoing International Calls
        "BAIC" - Barring of All Incoming Calls
        """
        if not mode or mode.lower() not in ("baoc", "boic", "baic"):
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value",
                    str(mode),
                    "for parameter 'mode'."))

        self._logger.info("Enable call barring.")

        method = "enableCallBarring"

        args = "--es mode %s" % (mode)
        if barring_code != "":
            args = args + "--es barring_code %s" % (barring_code)

        self._internal_exec_v2(self._voicecall_module, method, args, is_system=True)

    def disable_call_barring(self, mode, barring_code=""):
        """
        Disable call barring

        :param mode: Mode for call barring request
        :type mode: str
        :param barring_code: Code used for call barring request
        :type barring_code: str

        :rtype: None
        :raise DeviceException - INVALID_PARAMETER: If the "mode" parameter is invalid
        """

        """
        "BAOC" - Barring of All Outgoing Calls
        "BOIC" - Barring of Outgoing International Calls
        "BAIC" - Barring of All Incoming Calls
        """
        if not mode or mode.lower() not in ("baoc", "boic", "baic"):
            raise DeviceException(
                DeviceException.INVALID_PARAMETER,
                "%s '%s' %s." % (
                    "Unsupported parameter value ",
                    str(mode),
                    " for parameter 'mode'."))

        self._logger.info("Disable call barring for '%s' mode" % mode)

        method = "disableCallBarring"

        args = "--es mode %s" % (mode)
        if barring_code != "":
            args = args + "--es barring_code %s" % (barring_code)

        self._internal_exec_v2(self._voicecall_module, method, args, is_system=True)
    def get_sim_select(self,dialog,sim):
        """
        Set Default sim for DSDS feature

        :rtype: None
        """
        method = "Setdefaultsim"

        args = "--ei dialog %d --ei sim %d" % (
            dialog,
            sim)
        status=self._internal_exec_v2(self._voicecall_module, method,args,is_system=True)
        self._logger.info("Default SIM to be Set")
        return status
