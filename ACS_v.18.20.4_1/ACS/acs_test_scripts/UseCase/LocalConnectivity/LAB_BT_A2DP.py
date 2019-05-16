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
:summary: This file implements the LIVE_DUAL_PHONE_BT_PAIRING
:since:10/12/2012
:author: cmichelx
"""

import time
import posixpath
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global, FINDKEY
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE, BtAudioCmd, BtProfile, BtConState
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabBTA2dp(LiveBTBase):

    """
    Lab BT A2DP test.
    """

    """
    Delay after headset connection to wait for before send the 1st HS command
    in seconds
    """
    DELAY_AFTER_HS_CONNECTION = 5

    """
    Delay of 1 second
    """
    DELAY_ONE_SECOND = 1

    """
    Default number of retry to get state data
    """
    DEFAULT_MAX_RETRY = 5

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Read CONNECT_INITIATOR from test case xml file
        self._connector = \
            str(self._tc_parameters.get_param_value("CONNECT_INITIATOR"))
        # Read MUSIC_CONTROL_SEQUENCE from test case xml file
        audiocmd_list = \
            str(self._tc_parameters.get_param_value("MUSIC_CONTROL_SEQUENCE"))
        # Read DURATION from test case xml file
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION"))

        # Name of the MP3 file to play
        self._file_name = str(self._tc_parameters.get_param_value("FILE_TO_PLAY"))

        # Split to create a list of audio command
        self._audiocmd_list = audiocmd_list.strip().split(",")

        # BT headset
        self._bt_headset = None

        # BD address of Headset
        self._hsaddr = ""

    #------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        # Check audio command list is not empty
        if (len(self._audiocmd_list) == 1
                and self._audiocmd_list[0].strip() == ""):
            msg = "MUSIC_CONTROL_SEQUENCE cannot be empty"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Check CONNECT_INITIATOR value is valid
        self._connector = self._connector.upper()
        if self._connector not in ("DUT", "HEADSET"):
            msg = "BAD CONNECT_INITIATOR value. only DUT or HEADSET expected"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Get Bluetooth instance
        self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")

        # BD address of Headset
        self._hsaddr = self._bt_headset.get_bdaddress()

        if self._hsaddr.lower() in ["none", "", "00:00:00:00:00:00"]:
            msg = "No BD addr defined for BT HEADSET in bench_config"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Check BT address validity
        if not NetworkingUtil.is_valid_mac_address(self._hsaddr):
            msg = "Wrong BD address for HEADSET [%s]" % self._hsaddr
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Initialize BT headset to OFF state
        self._init_headset_off()
        time.sleep(5)

        # In both case (connector DUT or HEADSET), DUT shall initiates pairing
        # Put HEADSET in pairing mode (HS shall be OFF prior to that step)
        self._logger.info("Set headset in pairable mode")
        self._bt_headset.set_discoverable()
        time.sleep(1)

        # DUT scan to discover remote headset
        if not self._bt_api.bt_find_device(self._hsaddr):
            msg = "Headset <%s> not found" % self._hsaddr
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        # DUT initiates pairing to headset
        pair_result = self._bt_api.pair_to_device(self._hsaddr, 1)

        # pylint: disable=E1101
        if pair_result[0] == BT_BOND_STATE.BOND_BONDED:
            msg = "Pairing with headset succeeded"
            self._logger.info(msg)
        elif pair_result[0] == BT_BOND_STATE.BOND_NONE:
            msg = "Pairing with headset failed"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            msg = "[PHONE1]Unexpected return value %s" % pair_result
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._connector == "HEADSET":
            # If Initiator is HEADSET, DUT shall be discoverable
            self._bt_api.set_bt_discoverable("connectable", 0)

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        self._connect_profile()
        self._raise_error_if_already_playing()
        self._start_media_player()

        try:
            self._do_run_test()
        finally:
            self._bt_api.stop_a2dp_media_player()
            self._disconnect_profile()

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------

    def _connect_profile(self):
        """
        Connect DUT to HEADSET
        """
        if self._connector == "HEADSET":
            self._logger.info("HEADSET connects to DUT")  # Headset auto-connect after power on
            self._bt_headset.set_power(False)
            time.sleep(3)
            self._bt_headset.set_power(True)
            time.sleep(5)
            self._check_connection_state(BtProfile.A2DP, BtConState.d[BtConState.CONNECTED], 20)
        else:
            self._logger.info("DUT connects to HEADSET")
            if (not self._bt_api.connect_bt_device(self._hsaddr,
                    BtProfile.A2DP)):
                msg = "Connection failed with <%s>" % self._hsaddr
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    #------------------------------------------------------------------------------

    def _raise_error_if_already_playing(self):
        """
        Check DUT does not stream music
        """
        if self._bt_api.get_bt_audio_state(self._hsaddr, BtProfile.A2DP):
            msg = "Music is already streaming"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    #------------------------------------------------------------------------------

    def _start_media_player(self):
        """
        Start A2DP Player, timeout must be long enough to allow the test to end.
        """
        timeout = self._calculate_player_timeout()
        file_name = posixpath.join(self._device.multimedia_path, self._file_name)
        self._bt_api.start_a2dp_media_player(file_name, timeout)

    #------------------------------------------------------------------------------

    def _calculate_player_timeout(self):
        """
        Calculates the timeout to be passed to the A2dpMediaPlayer. Once the media player is activated, it stays
        active until it gets released, or the given timeout expires.
        Timeout needs to be function of the test case duration argument but it needs to consider other factors.
        In fact the time spent in the test case is much longer (because of other API calls and other time spent around).
        """
        # Double the _duration and multiply it for the number of "audio control commands" to be sent.
        # That should give a good margin
        timeout = ((self._duration + self.DEFAULT_MAX_RETRY) * 2) * len(self._audiocmd_list)
        # Add the delay after hc connection as we know it'll be part of the waiting
        timeout += self.DELAY_AFTER_HS_CONNECTION
        return timeout

    #------------------------------------------------------------------------------

    def _disconnect_profile(self):
        """
        Disconnect A2DP profile
        """
        time.sleep(self._wait_btwn_cmd)
        if (not self._bt_api.disconnect_bt_device(self._hsaddr, BtProfile.A2DP)):
            msg = "Disconnection failed with <%s>" % self._hsaddr
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    #------------------------------------------------------------------------------

    def _raise_error_if_invalid_cmd(self, curr_cmd):
        """
        check audio cmd value is valid
        """
        if curr_cmd not in (BtAudioCmd.PLAY, BtAudioCmd.STOP, BtAudioCmd.PAUSE):
            msg = "audio command value <%s> is not valid" % curr_cmd
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    #------------------------------------------------------------------------------

    def _do_run_test(self):
        """
        Executes the core of the test, i.e. press the button on the headset and verify the A2DP status,
        for each value of the control sequence argument
        """
        # Wait for a while before sending the first command
        # This sleep is to be close to real user handling
        time.sleep(self.DELAY_AFTER_HS_CONNECTION)
        # Loop on audio command list
        for curr_cmd in self._audiocmd_list:
            # Remove leading and trailing whitespace
            curr_cmd = curr_cmd.strip().upper()

            self._raise_error_if_invalid_cmd(curr_cmd)

            # Send audio command to headset
            self._logger.info("HEADSET audio command: %s" % curr_cmd)
            if curr_cmd in [BtAudioCmd.PLAY, BtAudioCmd.PAUSE]:
                self._bt_headset.play_pause_music_toggle()
            elif curr_cmd == BtAudioCmd.STOP:
                self._bt_headset.stop_music()
            else:
                assert False, "This condition is impossible to happen"

            audiost1 = self._check_audio_state(curr_cmd)
            # Wait for a while
            time.sleep(self._duration)
            # Check audio state
            audiost2 = self._bt_api.get_bt_audio_state(self._hsaddr, BtProfile.A2DP)
            if audiost1 != audiost2:
                msg = "audio state changed from %d to %d" % (audiost1, audiost2)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    #------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        self._bt_api.unpair_bt_device(self._hsaddr)

        LiveBTBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # Power off headset
        self._bt_headset.set_power(False)

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------

    def _init_headset_off(self):
        """
        Initialize BT headset in power off state

        Use DUT to check if BT headset is discoverable
        Pre-requisite: DUT shall be ON and its BT also ON

        Return nothing is success otherwise raise DeviceException
        """

        self._logger.info("Initialize BT headset to power OFF state")

        if self._bt_api.get_bt_power_status_eot() == 'STATE_OFF':
            self._bt_api.set_bt_power("on")

        # Set HS in pairing mode
        self._bt_headset.set_discoverable()

        # DUT scan to discover remote headset
        if self._bt_api.bt_find_device(self._hsaddr):
            # HS visible then turn it OFF
            self._bt_headset.set_power(False)

    #------------------------------------------------------------------------------

    def _check_audio_state(self, audiocmd):
        """
        Check DUT has received successfully the audio command
        :type audiocmd: str from BtAudioCmd class
        :param audiocmd: current audiocmd sent to DUT

        :rtype: int
        :return: the audio state
                - BtAudioState.d[BtAudioState.STOPPED] for stopped
                - BtAudioState.d[BtAudioState.PLAYING] for playing
        """

        timeout = self.DEFAULT_MAX_RETRY

        statefound = False
        while timeout > 0 and not statefound:
            audiost = self._bt_api.get_bt_audio_state(self._hsaddr, BtProfile.A2DP)

            if ((audiocmd in [BtAudioCmd.PLAY, BtAudioCmd.FW, BtAudioCmd.RW]
                and audiost) or
                (audiocmd in [BtAudioCmd.PAUSE, BtAudioCmd.STOP]
                 and not audiost)):
                statefound = True

            time.sleep(self.DELAY_ONE_SECOND)
            timeout -= 1

        if statefound:
            msg = "Command %s received successfully by DUT" % audiocmd
            self._logger.info(msg)
        else:
            msg = "Command %s not received by DUT" % audiocmd
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return audiost

    #------------------------------------------------------------------------------

    def _check_connection_state(self, profile, expstate, timeout=None):
        """
        Check DUT has received successfully the audio command
        :type profile: str from BtProfile class
        :param profile: BT profile to get the connection state
        :type expstate: str from BtConState class
        :param expstate: expected connection state
        :type timeout: integer
        :param timeout: maximum duration to poll connection state, in second

        :rtype: int
        :return: the connection state
                - BtConState.d[BtConState.CONNECTED]
                - BtConState.d[BtConState.CONNECTING]
                - BtConState.d[BtConState.DISCONNECTED]
                - BtConState.d[BtConState.DISCONNECTING]
                - BtConState.d[BtConState.UNKNOWN]
        """

        if timeout is None or timeout < 1:
            timeout = self.DEFAULT_MAX_RETRY

        if profile not in (BtProfile.A2DP, BtProfile.HSP):
            msg = "unexpected profile <%s>" % profile
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        statefound = False
        while timeout > 0 and not statefound:
            connst = self._bt_api.get_bt_connection_state(self._hsaddr, profile)
            if connst == expstate:
                statefound = True

            time.sleep(self.DELAY_ONE_SECOND)
            timeout -= 1

        state_verbose = FINDKEY(BtConState.d, connst)

        if statefound:
            msg = "profile <%s> is successfully <%s>" % (profile, state_verbose)
            self._logger.info(msg)
        else:
            msg = "bad <%s> profile <%s> state with <%s>" % (profile, state_verbose, self._hsaddr)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return connst
