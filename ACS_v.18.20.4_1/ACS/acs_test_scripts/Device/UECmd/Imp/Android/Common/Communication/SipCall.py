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
:summary: This file implements the SipCall UEcmd for Android phone
:since: 06/02/2013
:author: nprecigx
"""
import os
from Core.PathManager import Folders
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Communication.ISipCall import ISipCall
from acs_test_scripts.Device.UECmd.UECmdTypes import SIP_CALL_STATE
import re
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class SipCall(BaseV2, ISipCall):
    """
    :summary: SipCall UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, phone):
        """
        Constructor.

        """
        BaseV2.__init__(self, phone)
        ISipCall.__init__(self, phone)
        self._logger = phone.get_logger()
        # self._phone_system = PhoneSystem(phone)
        self._call_setup_timeout = phone.get_call_setup_timeout()
        self._sipcall_module = "acscmd.telephony.sipcall.SipCallModule"

        # pylint: disable=E1101
        self._sip_state = {
            "deregistering": SIP_CALL_STATE.DEREGISTERING,
            "in_call": SIP_CALL_STATE.IN_CALL,
            "incoming_call": SIP_CALL_STATE.INCOMING_CALL,
            "incoming_call_answering": SIP_CALL_STATE.INCOMING_CALL_ANSWERING,
            "not_defined": SIP_CALL_STATE.NOT_DEFINED,
            "outgoing_call": SIP_CALL_STATE.OUTGOING_CALL,
            "outgoing_call_canceling": SIP_CALL_STATE.OUTGOING_CALL_CANCELING,
            "outgoing_call_ring_back": SIP_CALL_STATE.OUTGOING_CALL_RING_BACK,
            "pinging": SIP_CALL_STATE.PINGING,
            "ready_to_call": SIP_CALL_STATE.READY_TO_CALL,
            "registering": SIP_CALL_STATE.REGISTERING}

    def initialize_sipcall_module(self):
        """
        initialize SipCall Module

        :return: None
        """
        self._logger.info("initialize SipCall Module")
        function = "initializeSipCallModule"
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def clean_sipcall_module(self):
        """
        clean sip module, remove all sip active profile

        :return: None
        """
        self._logger.info("Clean Sip Module")
        function = "cleanSipCallModule"
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def initialize_local_profile(self, phone_sip_address, password=None):
        """
        Initialize local profile

        :type phone_sip_address: str
        :param phone_sip_address: sip account name with server address in 'sip_account_name@sip_server' format

        :return: None
        :raise: AcsConfigException
        """
        # Check phone_sip_address in correctly format
        phone_sip_address_split = re.split('\@', phone_sip_address)
        if len(phone_sip_address_split) != 2:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Phone SIP address is not in user_name@sip_server format")
        if phone_sip_address_split[0] == "":
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Not user name in phone SIP address")
        if phone_sip_address_split[1] == "":
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Not sip server name in phone SIP address")

        sip_user_name = phone_sip_address_split[0]
        sip_server = phone_sip_address_split[1]

        self._logger.info("Set local SIP profile with %s address", phone_sip_address)
        function = "setLocalProfile"

        cmd_args = " --es sipUserName %s --es sipDomainServer %s" \
                   % (sip_user_name, sip_server)

        if password:
            cmd_args += " --es sipPassword %s" % password

        self._internal_exec_v2(self._sipcall_module, function, cmd_args, is_system=True)

    def dial(self, address_to_call, check_state=True):
        """
        Dials a SIP call.

        :type address_to_call: str
        :param address_to_call: SIP address to call
                                in 'sip_account_name@sip_server' format

        :type check_state: bool
        :param check_state: check call state or not.

        :return: None
        :raise: AcsConfigException
        """
        # Check phone_sip_address in correctly format
        address_to_call_split = re.split('\@', address_to_call)
        if len(address_to_call_split) != 2:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Phone SIP address is not in user_name@ip_server format")
        if address_to_call_split[0] == "":
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Not user name in phone SIP address")
        if address_to_call_split[1] == "":
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Not sip server name  phone SIP address")

        # Split sip_account_name and sip_server
        sip_user_name = address_to_call_split[0]
        sip_server = address_to_call_split[1]

        self._logger.info("Dialing %s ...", address_to_call)
        function = "makeSipCall"
        cmd_args = " --es sipPeerUserName %s --es sipPeerDomainServer %s" \
                   % (sip_user_name, sip_server)
        self._internal_exec_v2(self._sipcall_module, function, cmd_args, is_system=True)

        # Wait for SIP_CALL_STATE.IN_CALL state
        if check_state:
            self.wait_for_state(self._sip_state.get("outgoing_call_ring_back"), self._call_setup_timeout)

    def release(self):
        """
        Releases SIP call

        :return: None
        """
        function = "endSipCall"
        self._logger.info("Release SIP call...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

        self.wait_for_state(
            self._sip_state.get("ready_to_call"),
            self._call_setup_timeout)

    def answer(self):
        """
        Answer SIP call.

        :return: None
        """
        function = "answerSipCall"
        self._logger.info("Answering SIP incoming call...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

        self.wait_for_state(
            self._sip_state.get("in_call"),
            self._call_setup_timeout)

    def get_sip_call_state(self):
        """
        Returns SIP call state.

        :rtype: object
        :return: A value of SIP_CALL_STATE.
        """
        function = "getSipCallState"
        results = self._internal_exec_v2(self._sipcall_module, function, is_system=True)
        state = str(results["sip_call_state"])
        return self._sip_state[state]

    def wait_for_state(self, state, timeout):
        """
        Waits to reach a Sip call state until a timeout.

        :type state: UECmd.SIP_CALL_STATE
        :param state: expected state (see UECmd.SIP_CALL_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        :raise: AcsConfigException
        """
        self._logger.info("Waiting for %s state before %d seconds...", state, timeout)
        time_count = 0
        read_state = "UNKNOWN"
        state_reached = False

        while (state_reached == False) and time_count <= timeout:
            time_count += 1
            read_state = self.get_sip_call_state()
            if read_state == state:
                state_reached = True

        if not state_reached:
            err_msg = "Did not reach %s state" % (str(state))
            raise DeviceException(DeviceException.TIMEOUT_REACHED, err_msg)

    def switch_to_bluetooth(self):
        """
        Switch audio to bluetooth.

        :return: None
        """
        function = "enableBluetoothSipCall"
        self._logger.info("Switch audio to bluetooth...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def switch_to_bluetooth_a2dp(self):
        """
        Switch audio to bluetooth A2DP.

        :return: None
        """
        function = "enableBluetoothA2DPSipCall"
        self._logger.info("Switch audio to bluetooth A2DP...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def switch_to_speaker(self):
        """
        Switch audio to speaker.

        :return: None
        """
        function = "enableSpeakerSipCall"
        self._logger.info("Switch audio to speaker...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def switch_to_earpiece(self):
        """
        Switch audio to earpiece.

        :return: None
        """
        function = "enableEarpieceSipCall"
        self._logger.info("Switch audio to earpiece...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def set_profile_sip_phone_app(self, profile_name):
        """
        Restore a Sip profile on Phone App
        Need a Sip profile already saved

        :return: None
        """
        CURRENT_FOLDER = os.getcwd()
        profile_name_path = os.path.join(CURRENT_FOLDER,
                                         Folders.EXECUTION_CONFIG,
                                         profile_name)

        self._logger.info("Set Sip profile on phone app")
        self._exec("adb shell remount")
        self._exec("adb push " + profile_name_path + " /data/data/com.android.phone")
        self._exec("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db" +
                   " \"insert into system (name, value) values ('sip_receive_calls', '0')\"")
        self._exec("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db" +
                   " \"update system set value='0' where name='sip_receive_calls'\"")
        self._exec("adb shell \"sed -i 's/sip_receive_calls_key\" value=\"true\"/sip_receive_calls_key\"" +
                   " value=\"false\"/g' /data/data/com.android.phone/shared_prefs/com.android.phone_preferences.xml\"")
        self._exec("adb shell pkill com.android.phone")
        self._exec("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db" +
                   " \"update system set value='1' where name='sip_receive_calls'\"")
        self._exec("adb shell \"sed -i 's/sip_receive_calls_key\" value=\"false\"/sip_receive_calls_key\"" +
                   " value=\"true\"/g' /data/data/com.android.phone/shared_prefs/com.android.phone_preferences.xml\"")
        self._exec("adb shell pkill com.android.phone")

    def delete_profile_sip_phone_app(self):
        """
        Delete Sip profile previously saved on Phone App

        :return: None
        """
        self._logger.info("Remove Sip profile on phone app")
        phone_profile_dir = "/data/data/com.android.phone/files/profiles/"

        # Erase existing phone call settings if needed
        output = self._exec('adb shell test -d %s && echo 1 || echo 0' % phone_profile_dir)
        if output.isdigit() and bool(int(output)):
            self._exec("adb shell rm -R %s" % phone_profile_dir)

        self._exec("adb shell pkill com.android.phone")

    def toogle_mute(self):
        """
        Toogle Mute the current call.

        :return: None
        """
        function = "toogleMuteSipCall"
        self._logger.info("Toogle mute the current call...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def hold_call(self):
        """
        Hold the current call.

        :return: None
        """
        function = "holdSipCall"
        self._logger.info("Hold the current call...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)

    def unhold_call(self):
        """
        Unhold the current call.

        :return: None
        """
        function = "unholdSipCall"
        self._logger.info("UnHold the current call...")
        self._internal_exec_v2(self._sipcall_module, function, is_system=True)
