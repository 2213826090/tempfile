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
:summary: LocalConnectivity Utilities classes and function
:since: 22/10/2012
:author: lpastor
"""
import os
import random
import string  # pylint: disable=W0402
import time
import posixpath
from threading import Thread


from acs_test_scripts.Device.UECmd.UECmdTypes import BtProfile
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioCmd
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioState
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


NULL_ADDRESS = "00:00:00:00:00:00"


def get_bt_channel_from_freq(freq):
    """
    Get the BT channel ID from its frequency

    :type freq: int or str
    :param freq: The channel frequency

    :rtype: int
    :return: The channel ID
    """
    freq = int(freq)
    if freq < 2402:
        freq = 2402
    if freq > 2480:
        freq = 2480
    return int(freq - 2402)

def generate_random_string(string_size):
    """
    Generate a random str.
    :type string_size: int
    :param string_size: number of character contains in the generated str

    """
    random_string = str(''.
                        join(random.choice(string.ascii_uppercase + string.digits) for _x in range(string_size)))
    return random_string


def disconnect_tethering(who_disconnect, nap_api, panu_api, nap_addr, panu_addr):
    """
    Disconnect Tethering connection

    :type who_disconnect: String
    :param who_disconnect: "PAN-U" or "NAP" device to initiate disconnection
    :type nap_api: LocalConnectivity API Object
    :param nap_api: API to control BT on NAP device
    :type panu_api: LocalConnectivity API Object
    :param panu_api: API to control BT on PAN-User device
    :type nap_addr: String
    :param nap_addr: BT Address of the Network Access Point
    :type panu_addr: String
    :param panu_addr: BT Address of the Network PAN-User device
    """
    # Disconnect Bluetooth PAN profile
    if who_disconnect == "PAN-U":
        panu_api.disconnect_bt_device(nap_addr, BtProfile.PAN)
    elif who_disconnect == "NAP":
        nap_api.disconnect_bt_device(panu_addr, BtProfile.PAN)
    else:
        msg = "who_disconnect is not valid: " + str(who_disconnect)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)


def connect_to_nap(panu_api, nap_api, nap_addr, who_restarts_bt="NONE", wait_btwn_cmd=2):
    """
    Establish the BT Tethering connection

    :type panu_api: LocalConnectivity API Object
    :param panu_api: API to control BT on PAN-User PHONE device
    :type nap_api: LocalConnectivity API Object
    :param nap_api: API to control BT on NAP PHONE device
    :type nap_addr: String
    :param nap_addr: BT Address of the Network Access Point to connect to
    :type who_restarts_bt: String
    :param who_restarts_bt: "NAP", "PANU-U" or "NONE" to restart one of
                            the BT interface before connection
    :type wait_btwn_cmd: int
    :param wait_btwn_cmd: Time to wait between 2 UECmds
    """
    # Restart DUT (PHONE1) BT interface if required
    if who_restarts_bt == "NAP":
        nap_api.set_bt_tethering_power("off")
        time.sleep(wait_btwn_cmd)
        nap_api.set_bt_tethering_power("on")
        time.sleep(wait_btwn_cmd)
    elif who_restarts_bt == "PAN-U":
        panu_api.set_bt_power("off")
        time.sleep(wait_btwn_cmd)
        panu_api.set_bt_power("on")
        time.sleep(wait_btwn_cmd)

    # Establish connection
    if not panu_api.connect_bt_device(nap_addr, BtProfile.PAN):
        msg = "Connecting to %s through PAN profile failed" % nap_addr
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    time.sleep(wait_btwn_cmd * 4)


def establish_bt_pairing(requester_api, requester_addr, paired_api, paired_addr, wait_btwn_cmd=2):
    """
    Establishing Bluetooth pairing between requester and paired phones

    :type requester_api: LocalConnectivity API Object
    :param requester_api: API to control BT on the phone that will request the pairing
    :type requester_addr: String
    :param requester_addr: BT Address of the requester
    :type paired_addr: LocalConnectivity API Object
    :param paired_addr: API to control BT on the phone that will receive the pairing
    :type paired_addr: String
    :param paired_addr: BT Address of the paired device
    :type wait_btwn_cmd: int
    :param wait_btwn_cmd: Time to wait between 2 UECmds
    """
    # Set the phone to be paired as discoverable
    paired_api.set_bt_discoverable("both", 0)
    time.sleep(wait_btwn_cmd)

    # Do the pairing
    paired_api.wait_for_pairing(requester_addr, reconnect=1, replyval=1)
    requester_api.pair_to_device(paired_addr, 1)
    time.sleep(wait_btwn_cmd)

    # Unset discoverable mode
    paired_api.set_bt_discoverable("noscan", 0)

    # Control the pairing
    list_paired_devices = requester_api.list_paired_device()
    device_found = False
    for element in list_paired_devices:
        if str(element.address).upper() == str(paired_addr).upper():
            device_found = True
            break

    if not device_found:
        msg = "Pair to device %s failed" % paired_addr
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)


# ---- A2DP Functions ----
class ThreadA2DPCheckMusic(Thread):
    """
    This thread check if the music is still running.
    """
    def __init__(self, exceptions_queue, audio_state, duration, phone_bt, headset_bt_addr):
        """
        Constructor

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type audio_state : BtAudioState Object
        :param audio_state : state of music you want to check
        :type duration : integer
        :param duration : time of checking music
        :type phone_bt : Bluetooth API Object
        :param phone_bt : API for Bluetooth functions
        :type headset_bt_addr : str
        :param headset_bt_addr : address of the Bluetooth headset
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._audio_state = audio_state
        self._duration = int(duration)
        self._phone_bt = phone_bt
        self._headset_addr = headset_bt_addr

    def run(self):
        try:
            audio_old = self._phone_bt.get_bt_audio_state(self._headset_addr, BtProfile.A2DP)
            if audio_old != BtAudioState.d[self._audio_state]:
                msg = "Music state should be %s but is actually %s" % (BtAudioState.d[self._audio_state], audio_old)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            start = time.time()
            while (start + self._duration) > time.time():
                time.sleep(5.0)
                audio_current = self._phone_bt.get_bt_audio_state(self._headset_addr, BtProfile.A2DP)
                if audio_old != audio_current:
                    msg = str.format("Check music fail - music change {0} to {1}", str(audio_old), str(audio_current))
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                audio_old = audio_current

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadA2DPCheckMusic, self).join()
        return


class ThreadA2DPSwitchPlaying(Thread):
    """
    This thread switch audio between Pause and Resume state.
    """
    def __init__(self, exceptions_queue, initial_state, duration, delay, phone_bt, headset_bt):
        """
        Constructor

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type initial_state : BtAudioState Object
        :param initial_state : state of music at the beginning
        :type duration : integer
        :param duration : time of checking music
        :type delay : integer
        :param delay : delay between switches
        :type phone_bt : Bluetooth API Object
        :param phone_bt : API for Bluetooth functions
        :type headset_bt : Bluetooth Headset API Object
        :param headset_bt : API for Bluetooth Headset functions
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._initial_state = initial_state
        self._duration = int(duration)
        self._delay = int(delay)
        self._phone_bt = phone_bt
        self._headset_bt = headset_bt

    def run(self):
        try:
            if self._initial_state == BtAudioState.STOPPED:
                a2dp_switch_music_state(self._phone_bt, self._headset_bt, BtAudioCmd.PLAY)
                time.sleep(3.0)

            start = time.time()
            while(start + self._duration) > time.time():
                a2dp_switch_music_state(self._phone_bt, self._headset_bt, BtAudioCmd.PAUSE)
                time.sleep(self._delay)
                a2dp_switch_music_state(self._phone_bt, self._headset_bt, BtAudioCmd.PLAY)
                time.sleep(self._delay)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadA2DPSwitchPlaying, self).join()
        return


class ThreadA2DPSwitchVolume(Thread):
    """
    This thread switch audio volume. The thread only send commands, can't check for the moment that the
    command is correctly received and interpreted. Wait for AVRCP functions update.
    """
    def __init__(self, exceptions_queue, duration, phone_bt, headset_bt):
        """
        Constructor

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type duration : integer
        :param duration : time of checking music
        :type phone_bt : Bluetooth API Object
        :param phone_bt : API for Bluetooth functions
        :type headset_bt : Bluetooth Headset API Object
        :param headset_bt : API for Bluetooth Headset functions
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._duration = int(duration)
        self._phone_bt = phone_bt
        self._headset_bt = headset_bt

    def run(self):
        try:
            start = time.time()
            while(start + self._duration) > time.time():
                # Switch volume in this order : up, down, down, up, up, down
                self._headset_bt.set_volume(True)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.set_volume(False)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.set_volume(False)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.set_volume(True)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.set_volume(True)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.set_volume(False)
                time.sleep(2.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadA2DPSwitchVolume, self).join()
        return


class ThreadA2DPSwitchSong(Thread):
    """
    This thread switch song.The thread only send commands, can't check for the moment that the
    command is correctly received and interpreted. Wait for AVRCP functions update.
    """
    def __init__(self, exceptions_queue, duration, phone_bt, headset_bt):
        """
        Constructor

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type duration : integer
        :param duration : time of checking music
        :type phone_bt : Bluetooth API Object
        :param phone_bt : API for Bluetooth functions
        :type headset_bt : Bluetooth Headset API Object
        :param headset_bt : API for Bluetooth Headset functions
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._duration = int(duration)
        self._phone_bt = phone_bt
        self._headset_bt = headset_bt

    def run(self):
        try:
            start = time.time()
            while(start + self._duration) > time.time():
                # Switch song in this order : next, previous, previous, next
                self._headset_bt.next_audio_track()
                time.sleep(3.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.previous_audio_track()
                time.sleep(3.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.previous_audio_track()
                time.sleep(3.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)
                self._headset_bt.next_audio_track()
                time.sleep(3.0)
                _a2dp_check_audio_state(self._phone_bt, self._headset_bt, BtAudioState.PLAYING)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        super(ThreadA2DPSwitchSong, self).join()
        return


def a2dp_pair_connect_to_headset(phone_bt, headset_bt):
    """
    Pair and connect a device to a controlled BT headset

    :type phone_bt : Bluetooth API Object
    :param phone_bt : API to control device
    :type headset_bt : Bluetooth Headset API Object
    :param headset_bt : API to control BT headset
    """

    headset_bt_addr = headset_bt.get_bdaddress()

    # Initialize headset to off
    headset_bt.set_discoverable()
    if phone_bt.bt_find_device(headset_bt_addr):
        # HS visible then turn it OFF
        headset_bt.set_power(False)
        time.sleep(1.0)

    headset_bt.set_power(True)
    headset_bt.set_discoverable()
    phone_bt.flush_bt_scanned_devices()

    if phone_bt.bt_find_device(headset_bt_addr) is False:
        msg = "Can not found BT headset device"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    # pylint: disable=E1101
    pair_result = phone_bt.pair_to_device(headset_bt_addr, 1)
    if pair_result[0] != BT_BOND_STATE.BOND_BONDED:
        msg = "Can not pair device to headset"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    if phone_bt.connect_bt_device(headset_bt_addr, BtProfile.A2DP) is False:
        msg = "Can not connect device to headset"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)


def a2dp_unpair_headset(phone_bt, headset_bt):
    """
    Unpair a device to a controlled BT headset

    :type phone_bt : Bluetooth API Object
    :param phone_bt : API to control device
    :type headset_bt : Bluetooth Headset API Object
    :param headset_bt : API to control BT headset
    """

    headset_bt_addr = headset_bt.get_bdaddress()

    # Disconnect from Headset
    if phone_bt.disconnect_bt_device(headset_bt_addr, BtProfile.A2DP) is False:
        msg = "Can not disconnect device from headset"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    # Unpair device and headset
    phone_bt.unpair_bt_device(headset_bt_addr)
    headset_bt.set_power(0)


def _a2dp_check_audio_state(phone_bt, headset_bt, audio_state):
    """
    Check the current audio state.

    :type phone_bt : Bluetooth API Object
    :param phone_bt : API for Bluetooth functions
    :type headset_bt : Bluetooth Headset API Object
    :param headset_bt : API for Bluetooth Headset functions
    :type audio_state : BtAudioState.d
    :param audio_state : state required
    """

    headset_addr = headset_bt.get_bdaddress()
    audio_current = phone_bt.get_bt_audio_state(headset_addr, BtProfile.A2DP)
    if audio_current != BtAudioState.d[audio_state]:
        msg = "Check audio fail - audio must be %s but is %s" % (BtAudioState.d[audio_state], str(audio_current))
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)


def _a2dp_check_audio_state_change(phone_bt, headset_bt_addr, audiocmd, timeout=None):
    """
    Check the audio of the headset after an audio command

    :type phone_bt : Bluetooth API Object
    :param phone_bt : API for Bluetooth operations
    :type headset_bt_addr : str
    :param headset_bt_addr : Mac address of the BT headset
    :type audiocmd : BtAudioCmd Object
    :param audiocmd : command sent to the headset
    :type timeout : integer
    :param timeout : time before fail

    :return: current audio state
    """
    if timeout is None or timeout < 1:
        timeout = 5
    statefound = False

    start = time.time()
    while start + timeout > time.time() and not statefound:
        audiost = phone_bt.get_bt_audio_state(headset_bt_addr, BtProfile.A2DP)

        if ((audiocmd in [BtAudioCmd.PLAY, BtAudioCmd.FW, BtAudioCmd.RW]
            and audiost) or
            (audiocmd in [BtAudioCmd.PAUSE, BtAudioCmd.STOP]
             and not audiost)):
            statefound = True

    if not statefound:
        msg = "Check audio state failed - command not received"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    print "Command %s received successfully by DUT" % audiocmd
    return audiost


def a2dp_switch_music_state(bt_api, bt_headset_api, audio_cmd):
    """
    Launch music with headset using A2DP features.
    Require a minimum of 3 seconds to execute.

    :type bt_api : Bluetooth API Object
    :param bt_api : API used for Bluetooth functions
    :type bt_headset_api : Bluetooth Headset API Object
    :param bt_headset_api : API used for Bluetooth Headset functions
    :type aucio_cmd : BtAudioCmd Object
    :param audio_cmd : action on the headset (PLAY, PAUSE or STOP)
    """
    if audio_cmd not in [BtAudioCmd.PLAY, BtAudioCmd.PAUSE, BtAudioCmd.STOP]:
        msg = "Error parameter switch music state - %s" % str(audio_cmd)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    headset_bt_addr = bt_headset_api.get_bdaddress()

    initial_state = bt_api.get_bt_audio_state(headset_bt_addr, BtProfile.A2DP)
    if(initial_state == BtAudioState.d[BtAudioState.PLAYING] and audio_cmd == BtAudioCmd.PLAY) or \
        (initial_state == BtAudioState.d[BtAudioState.STOPPED] and audio_cmd == BtAudioCmd.PAUSE) or \
        (initial_state == BtAudioState.d[BtAudioState.STOPPED] and audio_cmd == BtAudioCmd.STOP):
        msg = "Can't switch music state, initial state is the state expected after the command"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    bt_headset_api.play_pause_music_toggle()
    audiost1 = _a2dp_check_audio_state_change(bt_api, headset_bt_addr, audio_cmd, 5)
    time.sleep(3.0)
    audiost2 = bt_api.get_bt_audio_state(headset_bt_addr, BtProfile.A2DP)
    if audiost1 != audiost2:
        msg = "Error switch music state"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

#------------------------------------------------------------------------------


class ThreadScanBT(Thread):
    """
    Thread used to do a Bluetooth scan and try to find the other phone give in parameter.
    This class raise an exception if the other BT device used as a reference is not found
    """

    SCANNING_WAITING_TIME = 5

    def __init__(self, exceptions_queue, phone_local_bt, phone_remote_bt, duration=1.0):
        """
        Initialize the thread

        :type exceptions_queue : Queue object
        :param exceptions_queue : used to send exceptions to the main thread
        :type phone_local_bt : Bluetooth API Object
        :param phone_local_bt : Bluetooth API of the phone to do the scan
        :type phone_remote_bt : Bluetooth API Object
        :param phone_remote_bt : Bluetooth API of the phone to receive the scan
        :type duration : integer
        :param duration : max time of scans
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_local_bt = phone_local_bt
        self._phone_remote_bt = phone_remote_bt
        self._duration = duration

    def run(self):
        """
        Execute the thread
        """

        try:
            start = time.time()
            while(start + self._duration) > time.time():
                self._phone_remote_bt.set_bt_discoverable("both", 0)
                self._phone_local_bt.flush_bt_scanned_devices()
                time.sleep(ThreadScanBT.SCANNING_WAITING_TIME)
                remote_addr = self._phone_remote_bt.get_bt_adapter_address()

                if not self._phone_local_bt.bt_find_device(remote_addr):
                    msg = "BT Scan - device %s not found" % remote_addr
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadScanBT, self).join()
        return


class ThreadOPPTransferFile(Thread):
    """
    Thread used to do a Bluetooth OPP transfer.
    """

    def __init__(self, exceptions_queue, device_local, device_remote, local_file, local_file_path,
                 iterations=1, delay=0, timeout=60, initialize=False):
        """
        Initialize the OPP file transfer thread.

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type device_local : device Object
        :param device_local : phone which send the file
        :type device_remote : device Object
        :param device_remote : phone which receive the file
        :type local_file : str
        :param local_file : name of the file which will be sent
        :type local_file_path : str
        :param local_file_path : path of the file which will be sent
        :type iterations : integer
        :param iterations : number of scans the thread must do
        :type delay : integer
        :param delay : time between iterations of send
        :type timeout : integer
        :param timeout : maximum send time
        :type initialize : boolean
        :param initialize : initialize OPP service if set to True
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._device_local = device_local
        self._device_remote = device_remote
        self._local_file = local_file
        self._local_file_path = local_file_path
        self._iterations = iterations
        self._delay = delay
        self._timeout = timeout
        self._initialize = initialize

    def run(self):
        """
        Execute the thread
        """
        count_loop = 0

        try:
            if self._initialize:
                phone_local_bt = self._device_local.get_uecmd("LocalConnectivity")
                phone_remote_bt = self._device_remote.get_uecmd("LocalConnectivity")
                phone_local_phone_system = self._device_local.get_uecmd("PhoneSystem")
                phone_remote_phone_system = self._device_remote.get_uecmd("PhoneSystem")
                _opp_init_configure(phone_local_phone_system, phone_remote_phone_system,
                                    phone_local_bt, phone_remote_bt)

            while count_loop < self._iterations:
                opp_transfer_file(self._device_local, self._device_remote,
                                  self._local_file, self._local_file_path, self._timeout)
                count_loop += 1

                time.sleep(self._delay)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadOPPTransferFile, self).join()
        return

#------------------------------------------------------------------------------


def _opp_init_configure(phone_local_phone_system, phone_remote_phone_system, phone_local_bt, phone_remote_bt):
    """
    Configure OPP service.

    :type phone_local_phone_system : PhoneSystem API Object
    :param phone_local_phone_system : API to control the screen on the phone that sends the OPP file
    :type phone_remote_phone_system : PhoneSystem API Object
    :param phone_remote_phone_system : API to control the screen on the phone that receives the OPP file
    :type phone_local_bt : Bluetooth API Object
    :param phone_local_bt : API to control Bluetooth on the phone that sends the OPP file
    :type phone_remote_bt : Bluetooth API Object
    :param phone_remote_bt : API to control Bluetooth on the phone that receives the OPP file
    """
    # Unlock screens
    phone_local_phone_system.display_on()
    phone_local_phone_system.set_phone_lock(0)
    phone_remote_phone_system.display_on()
    phone_remote_phone_system.set_phone_lock(0)

    time.sleep(2.0)

    phone_local_bt.set_bt_discoverable("both", 0)
    phone_remote_bt.set_bt_discoverable("both", 0)
    time.sleep(1.0)

    phone_local_bt.bt_opp_clean_notification_list()
    phone_remote_bt.bt_opp_clean_notification_list()
    time.sleep(1.0)


def opp_init_configure(device_local, device_remote):
    """
    Configure OPP service.

    :type device_local : device Object
    :param device_local : phone under tests
    :type device_remote : device Object
    :param device_remote : reference phone
    """
    phone_local_bt = device_local.get_uecmd("LocalConnectivity")
    phone_remote_bt = device_remote.get_uecmd("LocalConnectivity")
    phone_local_phone_system = device_local.get_uecmd("PhoneSystem")
    phone_remote_phone_system = device_remote.get_uecmd("PhoneSystem")

    _opp_init_configure(phone_local_phone_system, phone_remote_phone_system, phone_local_bt, phone_remote_bt)


def opp_terminate(device_local, device_remote):
    """
    Close opp service and clean notifications.

    :type device_local : device Object
    :param device_local : phone under tests
    :type device_remote : device Object
    :param device_remote : reference phone
    """
    phone_local_bt = device_local.get_uecmd("LocalConnectivity")
    phone_remote_bt = device_remote.get_uecmd("LocalConnectivity")
    phone_local_bt.bt_opp_clean_notification_list()
    phone_remote_bt.bt_opp_clean_notification_list()


def opp_accept_file(api, file_name, time_out):
    """
    Accept an incoming OPP file transfer
    :type filename: str
    :param filename: full path and filename of the file to transfer
    :type destination_address: str
    :param destination_address: BT address of the device to send the file to
    :type time_out: int
    :param time_out: after which the function returns an error
    :return: True if file accepted, false if not (or timed out)
    """
    expected = os.path.basename(file_name)
    t_0 = time.time()
    while True and (time.time() - t_0 < time_out):
        result = api.bt_opp_check_service()
        if len(result[0]) > 0:
            fname = result[2][0]
            status = result[5][0]
            if status == "downloading":
                return True
        time.sleep(1)
    return False


def opp_check_file_cancelled(api, file_name, time_out):
    """
    Check that an incoming OPP file transfer was rejected
    :type filename: str
    :param filename: full path and filename of the file to transfer
    :type destination_address: str
    :param destination_address: BT address of the device to send the file to
    :type time_out: int
    :param time_out: after which the function returns an error
    :return: True if file cancelled, false if not (or timed out)
    """
    expected = os.path.basename(file_name)
    t_0 = time.time()
    while True and (time.time() - t_0 < time_out):
        result = api.bt_opp_check_service()
        if len(result[0]) > 0:
            fname = result[2][0]
            status = result[5][0]
            if status == "cancelled":
                return True
        time.sleep(1)
    return False


def opp_check_file_waiting_accept(api, file_name, time_out):
    """
    Check that an incoming OPP file transfer was rejected
    :type filename: str
    :param filename: full path and filename of the file to transfer
    :type destination_address: str
    :param destination_address: BT address of the device to send the file to
    :type time_out: int
    :param time_out: after which the function returns an error
    :return: True if file is waiting to be accepted, false if not (or timed out)
    """
    expected = os.path.basename(file_name)
    t_0 = time.time()
    while True and (time.time() - t_0 < time_out):
        result = api.bt_opp_check_service()
        if len(result[0]) > 0:
            fname = result[2][0]
            status = result[5][0]
            if fname == 'Empty value.':
                if status == "waiting_accept":
                    return True
            else:
                if status == "waiting_accept":
                    return True

        time.sleep(1)
    return False


def opp_check_status_none(api, time_out):
    """
    Check that an incoming OPP file transfer was rejected
    :type filename: str
    :param filename: full path and filename of the file to transfer
    :type destination_address: str
    :param destination_address: BT address of the device to send the file to
    :type time_out: int
    :param time_out: after which the function returns an error
    :return: True if file is waiting to be accepted, false if not (or timed out)
    """
    t_0 = time.time()
    while True and (time.time() - t_0 < time_out):
        result = api.bt_opp_check_service()
        if not result[0]:
            return True
        time.sleep(1)
    return False


def opp_transfer_file(device_local, device_remote, local_file, local_file_path, timeout=60):
    """
    This class execute a file transfer using BT OPP profile.
    OPP service must be configured before launch this function.

    :type device_local : device Object
    :param device_local : phone which send the file
    :type device_remote : device Object
    :param device_remote : phone which receive the file
    :type local_file : str
    :param local_file : name of the file which will be sent
    :type local_file_path : str
    :param local_file_path : path of the file which will be sent
    :type timeout : integer
    :param timeout : maximum send time
    """
    complete_path = posixpath.join(local_file_path, local_file)

    phone_local_bt = device_local.get_uecmd("LocalConnectivity")
    phone_remote_bt = device_remote.get_uecmd("LocalConnectivity")

    phone_remote_bt.bt_opp_init(local_file)
    time.sleep(1.0)

    phone_remote_bt.bt_opp_clean_notification_list()

    phone_remote_address = phone_remote_bt.get_bt_adapter_address()
    phone_local_bt.bt_opp_send_file(complete_path, phone_remote_address)

    if not opp_accept_file(phone_remote_bt, complete_path, 10.0):
        msg = "remote device has not accepted the file on time"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    time.sleep(10.0)

    file_list = [local_file]
    local_file_size = phone_local_bt.bt_opp_get_files_checksum(local_file_path, file_list)
    start = time.time()

    while start + timeout > time.time():
        current_checksum = phone_remote_bt.bt_opp_get_files_checksum(None, file_list)
        if(local_file_size[local_file] == current_checksum[local_file]):
            break
        time.sleep(1.0)

    if local_file_size[local_file] != current_checksum[local_file]:
        msg = "error during opp transfer"
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)


class ThreadScanBTRemoteDevice(Thread):
    """
    Thread used to do a Bluetooth scan and try to find the bench BT device (or a specific address give in parameter).
    This class raise an exception if the other BT device used as a reference is not found
    """

    SCANNING_WAITING_TIME = 12.0

    def __init__(self, exceptions_queue, phone_local_bt, ref_bt_addr, duration):
        """
        Initialize the thread

        :type exceptions_queue : Queue object
        :param exceptions_queue : used to send exceptions to the main thread
        :type phone_local_bt : Bluetooth API Object
        :param phone_local_bt : Bluetooth API of the phone to do the scan
        :type ref_bt_addr : String
        :param ref_bt_addr : Remote BT device address
        :type duration : integer
        :param duration : time of execution
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_local_bt = phone_local_bt
        self._ref_bt_addr = ref_bt_addr
        self._duration = duration

    def run(self):
        """
        Execute the thread
        """

        try:
            start = time.time()
            while(start + self._duration) > time.time():
                self._phone_local_bt.flush_bt_scanned_devices()
                time.sleep(ThreadScanBTRemoteDevice.SCANNING_WAITING_TIME)

                if not self._phone_local_bt.bt_find_device(self._ref_bt_addr):
                    msg = "BT Scan - device %s not found" % self._ref_bt_addr
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)
