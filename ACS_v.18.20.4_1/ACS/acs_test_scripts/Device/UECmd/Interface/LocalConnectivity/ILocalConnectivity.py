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
:summary: This script implements the interfaces to unitary local
connectivity features
:since: 19/07/2010
:author: asebbane, vgombert
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class ILocalConnectivity():

    """
    Abstract class that defines the interface to be implemented
    by local connectivity handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    # pylint: disable=W0613

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def get_bt_scan_mode(self):
        """
        Retrieves the Bluetooth scan mode.

        :rtype: str
        :return: scan mode (both, noscan, inquiry, page)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_service_browsing(self, bd_address, class_to_browse=""):
        """
        Remote device service browsing.

        :type bd_address: str
        :param bd_address: Remote device BD address
                           format 00:00:00:00:00:00

        :type class_to_browse: str
        :param class_to_browse: type of service class

        :rtype: Boolean
        :return: Service search result
         True  = Service found or browsing succeeded
         False = Service not found or browsing failed

        :rtype: BluetoothDevice
        :return: an instance of BluetoothDevice with
         Name of remote device
         Address of remote device
         uuids = list of Services found
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_l2cap_ping(self, bd_address, packet_count="", packet_size=""):
        """
        Performs a l2ping on a remote Bluetooth device.

        :type bd_address: str
        :param bd_address: Bluetooth address to ping

        :type packet_size: int
        :param packet_size: Packet size in bytes

        :type packet_count: int
        :param packet_count: Number of packet to send

        :rtype: Measure Object (value,unit)
        :return: packet loss
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_reset_device(self):
        """
        Resets a Bluetooth adapter. Turn OFF then ON and check power state is successfully ON

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_scan_devices(self):
        """
        Scans remote Bluetooth devices.

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_fota(self, mac_address, files_to_flash, timeout):
        """
        Do a FOTA (Flash Over The Air) using Bluetooth.

        An exception is raised if an error occurs during the FOTA process (i.e. if
        this method ends, the FOTA has been done correctly).

        :type mac_address: str
        :param mac_address: MAC address of the device on which the FOTA shall be done
        :type files_to_flash: list
        :param files_to_flash: list of files to flash with FOTA
        :type timeout: int
        :param timeout: FOTA timeout value

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_find_device(self, remote_device_info):
        """
        Looks for a Bluetooth device.

        :type remote_device_info: str
        :param remote_device_info: Name or address of the device

        :rtype: boolean
        :return: True if found , False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_check_msg(self, msg):
        """
        check if file contain this message -- msg
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_receive_msg(self):
        """
        Receive message from another devices through BluetoothSocket

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_send_msg(self, remote_device_addr, msg):
        """
        Send message to another devices through BluetoothSocket

        :type remote_device_addr: str
        :param remote_device_addr: address of the device
        :type msg: str
        :param msg: message to send

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_init(self, file_to_be_received):
        """
        If already in filesystem, remove the file to be received.

        :type file_to_be_received: str
        :param file_to_be_received: filename of the file to be received soon

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_send_file(self, filename, destination_address):
        """
        Send a file to another devices through Bluetooth OPP transfer
        WARNING: Display shall be ON before using such method
                 Use phone_system.display_on() + phone_system.set_phone_lock(0)

        :type filename: str
        :param filename: full path and filename of the file to transfer
        :type destination_address: str
        :param destination_address: BT address of the device to send the file to

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_cancel_send(self):
        """
        Cancel sending files through OPP
        All the files being sent are cancelled

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_clean_notification_list(self):
        """
        Clean (Empty) the BT OPP notification list
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_check_service(self):
        """
        Retreive the entire BT OPP notifications information.

        :rtype: tuple
        :return: a tuple of notification list with following data:
        - id: The notification ID.
        - address: The address of remote device concerned by the transfer.
        - filename: The concerned file name.
        - filesize: The concerned file size.
        - downloadedsize: The file size already downloaded.
        - status: The notification status.
        - timeStamp: The timestamp (start time) of the file transfer, in ms.
        - curtime: The current time of the transfer, in ms.
        - dir: the direction of the transfer (UL=0, DL=1)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_opp_get_files_checksum(self, folder_name, file_list):
        """
        Returns a dict {file_name, checksum} for each file in file_list

        :type folder_name: str
        :param folder_name: the folder to search the files in
        :type file_list: str array
        :param file_list: list of file names

        :return: dict containing for each file its checksum
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def connect_bt_device(self, remote_device_addr, profile):
        """
        Connects to a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device
        :type profile: str
        :param profile: profile to connect with like hsp, a2dp...

        :rtype: boolean
        :return: connection status (true for connection succeeded)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def connect_bt_hid_device(self, remote_device_addr, reconnect="off"):
        """
        Connects Bluetooth hid device.

        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off',' 0', 0) to disable
        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_connection_state(self, remote_device_addr, profile):
        """
        Get the Bluetooth connection state to a specific profile.

        :type profile: str
        :param profile: profile to connect with like hsp, a2dp...
        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :rtype: int
        :return: the connection state
                - BtConState.d[BtConState.DISCONNECTED] for disconnected
                - BtConState.d[BtConState.CONNECTED] for connected
                - BtConState.d[BtConState.CONNECTING] for connecting
                - BtConState.d[BtConState.DISCONNECTING] for disconnecting
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_bt_device(self, remote_device_addr, profile):
        """
        Disconnects from a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device
        :type profile: str
        :param profile: profile to disconnect with like hsp, a2dp...

        :rtype: boolean
        :return: disconnection status (true for disconnection succeeded)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_audio_state(self, remote_device_addr, profile):
        """
        Get the Bluetooth audio state to a specific profile.

        :type profile: str
        :param profile: profile to use like hsp, a2dp...
        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :rtype: int
        :return: the audio state
                - BtAudioState.d[BtAudioState.STOPPED] for stopped
                - BtAudioState.d[BtAudioState.PLAYING] for playing
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_bt_audio_connected(self):
        """
        Check whether BT audio device is connected or not

        :rtype: boolean
        :return: return True if BT audio device exists.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_power(self, mode):
        """
        Sets the Bluetooth power.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_power_status(self):
        """
        Gets the Bluetooth power status.

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON', 'STATE_TURNING_OFF',
                     'STATE_TURNING_ON')
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_power_state_reached(self, expected_state, timeout=None):
        """
        Waits until the requested power state is reached (or timeout expires)

        :type expected_state: str
        :param expected_state: the state to wait for
        :type timeout: int
        :param timeout: maximum time to wait for end of transition (in seconds)

        :rtype: bool
        :return: True is state has been reached, False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_power_status_eot(self, expected_state=None, timeout=None):
        """
        Wait for the end of transition to get the Bluetooth power status.

        :type expected_state: str
        :param expected_state: the state to wait for
        :type timeout: int
        :param timeout: maximum time to wait for end of transition (in seconds)

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON')
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_pairable(self, mode):
        """
        Switchs the Bluetooth device to pairable mode.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_pairable_status(self):
        """
        Returns the Bluetooth device pairable status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_discoverable(self, mode, timeout):
        """
        Sets the device discoverable.

        :type mode: str or int
        :param mode: can be ('noscan', 'none', SCAN_MODE_NONE) for no scan
          ('inquiry') for disoverable not connectable. Not supported by android
          ('page', 'connectable', SCAN_MODE_CONNECTABLE) for connectable
          ('both', SCAN_MODE_CONNECTABLE_DISCOVERABLE) to discoverable
        :type timeout: int
        :param timeout: can be 0 for Never,
                               120 for 2 minutes
                               300 for 5 minutes
                               3600 for 1 hour

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_pairable_timeout(self):
        """
        Returns the timeout for the pairable mode.

        :rtype: int
        :return: pairable timeout value
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_discoverable_timeout(self):
        """
        Returns the timeout for the discoverable mode.

        :rtype: int
        :return: discoverable timeout value
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_autoconnect(self, mode):
        """
        Sets the Bluetooth autoconnect.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_autoconnect_status(self):
        """
        Gets the Bluetooth autoconnect state.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_scanning(self, mode):
        """
        Starts/stops the device discovery session.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def pair_to_device(self, remote_device_addr, reconnect="off",
                       replyval=1, pincode="0000", passkey=0):
        """
        Connects to a Bluetooth remote device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address
                                   format 00:00:00:00:00:00
        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off', '0', 0) to disable
        :type replyval: int
        :param replyval: pairing request reply value
                         1 = Positive reply
                         0 = Negative reply
        :type pincode: str
        :param pincode: Bluetooth device pincode for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: Bluetooth device passkey for pairing process
                       (6 digits Max)

        :rtype: list of 2 strings
        :return: (bond_state 'BOND_BONDED' or 'BOND_NONE') and
            (pinRes is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def unpair_bt_device(self, remote_device_addr):
        """
        Remove the remote Bluetooth device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_pairing(self, remote_device_addr, reconnect=0,
                         replyval=1, pincode="0000", passkey=0, timeout=20):
        """
        Wait for a pairing request from a Bluetooth remote device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address
                                   format 00:00:00:00:00:00
        :type replyval: int
        :param replyval: pairing request reply value
                         1 = Positive reply
                         0 = Negative reply
        :type pincode: str
        :param pincode: Bluetooth device pincode for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: Bluetooth device passkey for pairing process
                       (6 digits Max)

        :type: timeout: int
        :param: timeout: timeout for expecting a pairing request

        :rtype: str
        :return: bond_state 'BOND_BONDED' or 'BOND_NONE' or 'BONDING'
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_pairing_canceled(self):
        """
        Wait pairing cancelation due to timeout reached in embedded agent.

        :rtype: Boolean status of cancelation
        :return: True if pairing is successfully canceled False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_paired_device(self):
        """
        Lists all paired devices

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_connected_device(self):
        """
        Lists all connected devices

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_authentication(self, interface, mode):
        """
        Sets the Bluetooth authentication to enabled/disabled for a given device

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_agent_property(self, name, value):
        """
        Sets the properties of the simple agent file.
        available properties in Meego implementation are:
                 - "ReadFromFile" : "yes"|"no"
                 - "Authorize" : "yes"|"no"
                 - "PinCode" : str (can be characters and/or digits)
                 - "PassKey" : str composed of digits between 0-999999
                 - "Confirmation" : "yes"|"no"

        :type name: str
        :param name: name of the property
        :type value: str
        :param value: value of the property

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_default_link_policy(self, interface, mode):
        """
        Sets the Bluetooth default link policy for a given interface.

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...
        :type mode: str
        :param mode: "rswitch", "RSWITCH", "hold", "HOLD",
                     "sniff", "SNIFF", "park", "PARK"

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_bt_test_mode(self, interface):
        """
        Activates Bluetooth test mode.

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_ctrl_event_mask(self, interface, mask):
        """
        Sets BT controller Event Mask.

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...
        :type mask: str
        :param mask: list of events which are generated by the HCI for the host.
                 the mask is made of hex values
                 ex : default mask = 0x00001FFFFFFFFFFF

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_ctrl_event_filter(self, interface,
                                 filter_type,
                                 filter_condition_type="",
                                 condition=""):
        """
        Sets BT controller Event Filter.

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...
        :type filter_type: str
        :param filter_type: filter type eg : 0x02
        :type filter_condition_type: str
        :param filter_condition_type: For each Filter Type one or more Filter
        :type condition: list of str
        :param condition: For each Filter Condition Type defined for the Inquiry Result Filter
        and the Connection Setup Filter, zero or more Condition parameters are
        required depending on the filter condition type and filter type.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_adapter_address(self):
        """
        Gets Bluetooth address.

        :rtype: str
        :return: Bluetooth adapter address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_adapter_power(self, mode):
        """
        Sets the Bluetooth adapter power mode.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_a2dp_media_player(self, file_name, timeout):
        """
        Starts A2DP media player setting the file to be played

        :type file_name: str
        :param file_name: The name of the mp3 file to be played
        :type timeout: int
        :param timeout: maximum time for the player to be active.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_a2dp_media_player(self):
        """
        Stops A2DP media player
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_nfc_status(self):
        """
        Retreive the NFC Status

        :rtype: str
        :return: Return the NFC status "OFF", "TURNING_ON", "ON", "TURNING_OFF", "UNKNOWN"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_nfc_beam_status(self):
        """
        Retrieve the NFC BEAM Status

        :rtype: boolean
        :return: True if the NFC BEAM status is activated.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def nfc_enable(self):
        """
        Enable NFC interface

        :return: list of founded adapters (interface/address)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def nfc_disable(self):
        """
        Disable NFC interface

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def force_nfc_state(self, state):
        """
        After a PhoneSystem.set_phone_lock(0) NFC is still OFF
        This function is a workarround to re-enable NFC after a set_phone_lock(0)

        :type state: integer or str
        :param state: 1 or on to force re-enable NFC
                      0 or off to get back to a normal mode.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_nfc_crash(self, starttime):
        """
        Check the CrashInfo log to found "com.android.nfc" or "SIGSEGV" after
        the starttime

        :type starttime: float
        :param starttime: date after the one the check must be do
                            (get from time.time())

        :return: True if NFC Crash found, False if no NFC Crash found
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def select_secure_element(self, secure_element):
        """
        Select NFC secure element

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def read_nfc_tag(self):
        """
        Read data from NFC tag

        :rtype: String
        :return: read_data
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def read_nfc_tag_reader_mode(self):
        """
        Read data from NFC tag

        :rtype: String
        :return: read_data
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def write_nfc_tag(self, rtd_type, data):
        """
        Write data in tag

        :type rtd_type: str
        :param rtd_type: "RTD_TEXT", "RTD_SMARTPOSTER" or "RTD_URI"

        :type data:  str
        :param data

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def write_nfc_tag_reader_mode(self, rtd_type, data):
        """
        Write data in tag

        :type rtd_type: str
        :param rtd_type: "RTD_TEXT", "RTD_SMARTPOSTER" or "RTD_URI"
        :type data:  str
        :param data

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def exchange_apdu_using_scapi(self, channel, reader, apdu_case, data_length, loop):
        """
        Exchange APDUs

        :type channel: str
        :param channel: LOGICAL, BASIC
        :type reader:  str
        :param reader: UICC, SMX
        :type apdu_case: str
        :param apdu_case: ["1", "2", "3", "4", "ALL"]
        :type data_length: int
        :param data_length: [1..256]
        :type loop: int
        :param loop: number of commands sent during the test

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_default_nfc_p2p_configuration(self):
        """
        Set default P2P configuration
        Initiator : all passive and 424 active
        Target : all passive except 106, all active

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_nfc_p2p_configuration(self, mode, role, bitrate):
        """
        Set P2P configuration

        :type mode: str
        :param mode: active, passive
        :type role: str
        :param role: initiator, target
        :type bitrate: str
        :param bitrate: 106, 212, 424

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_tethering_power(self, state):
        """
        Enable / Disable the Bluetooth Tethering feature.

        :type state: str or int
        :param state: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_nfc_beam(self):
        """
        Enable NFC beam interface. Auto activation is performed in order to modify NFC beam status.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_nfc_beam(self):
        """
        Disable NFC beam interface. Auto activation is performed in order to modify NFC beam status.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def nfc_touch_to_beam(self, screen_size):
        """
        Start beam transfer touching the middle of the screen

        :type screen_size: str
        :param screen_size: 720x1184 format
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_bt_tethering_power(self):
        """
        Get Bluetooth Tethering feature state (Enable/disable).

        :rtype: boolean
        :return: True = enable
                 False = disable
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def flush_bt_scanned_devices(self):
        """
        Clean BT scanned devices list
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_default_addr(self):
        """
        Get the default BD address (persistant address)

        :rtype: str
        :return: default BD address. format: 12:34:56:78:9A:BC
                 otherwise return empty str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_mac_addr(self, domain, mac_addr):
        """
        Set the mac address for BT or WIFI interface
        :type domain:  str
        :param domain: can be 'wifi' or 'bt'
        :type mac_addr:  str
        :param mac_addr: MAC address to set (format 08:00:28:44:44:44)

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def avrcp_expect_buttons(self, sequence, timeout, file_name):
        """
        Expects the given buttons sequence to be happening.
        :type sequence:  str
        :param sequence: list of buttons semicolon separated
        :type timeout: int
        :param timeout: timeout for each button to be received
        :type file_name:  str
        :param file_name: name of the audio file to be played
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bluetooth_menu_settings(self):
        """
        Enter Bluetooth menu setting or exit from Bluetooth menu settings
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bt_name(self, name):
        """
        Sets the device Bluetooth name.
        :type name: str
        :param name: name of the Bluetooth device
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_gatt_read_characteristic(self, service_uuid, char_uuid):
        """
        Read a GATT Bluetooth characteristic on device
        :type service_uuid:  str
        :param service_uuid: Uuid of service, can use UBtServiceClass.GATT_SERVICE
        :type service_uuid:  str
        :param service_uuid: Uuid of service, can use UBtServiceClass.GATT_SERVICE
        :type: data_data_type: str
        :param data_type: "string', "int", "hexa", "hexa_endian", default "string"
                            if string: convert hexa to string before return value
                            if int: convert hexa to integer before return value
                            if hexa: no conversion will be operated, only concatenation.
                            if hexa_endian: will convert as little endian and concatenate before return value.

        :return: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_gatt_write_characteristic(self, service_name, char_name, data, data_type="hexa", acknowledgement=True):
        """
        Write data on GATT Bluetooth characteristic on device

        :type data: str
        :param data: Data to be send to device, as str format

        :type: data_data_type: str
        :param data_type: "hexa", "hexa_endian" or "string", default "hexa"
                            If "hexa_endian" will convert as little endian before writing

        :type service_name:  str
        :param service_name: Name of service, can use UBtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.

        :type char_name:  str
        :param char_name: Name of characteristic, can use UBtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.

        :type acknowledgement: bool
        :param acknowledgement: Whether as for acknowledgement on device or not

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_ssp_mode(self, mode):
        """
        Set Simple Secure Pairing mode to enable/disable it and force
        device to useLegacy Pairing mode instead.
        :type mode: string
        :param mode: SSP mode: "off" => disabled, "on" => enabled
        :return None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_service(self, enable, service):
        """
        Enable or disable bluetooth service
        :type enable: boolean
        :param enable: enable service if true, disable it if false
        :type service: string
        :param service: bluetooth service to enable or disable
                        service can be one of "nap", "gn" or "panu"
        :return None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_connection(self, remote_device_addr, profile):
        """
        Wait for bluetooth device connection
        :type remote_device_addr: str
        :param remote_device_addr:  bluetooth remote device to wait connection for
        :type profile: str
        :param profile: bluetooth profile to handle for device connection
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def trust_device(self, remote_device_addr, trust):
        """
        Trust/Untrust paired device
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth address of device to trust
        :type trust: boolean
        :param trust:
            - True to trust remote devie
            - False to untrust remote device
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_gatt_subscribe_notification(self, service_name, char_name):
        """
        Subscribe to a GATT Bluetooth notification related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :return: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_gatt_unsubscribe_notification(self, service_name, char_name):
        """
        Unsubscribe to a GATT Bluetooth notification related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :return: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def bt_gatt_get_notification(self, service_name, char_name, time_lap):
        """
        Get GATT Bluetooth notifications related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :type time_lap: int
        :param time_lap: Lap of time where notification should be backlogged (in minutes)

        :rtype: list
        :return: List of UECmdTypes.BtGattNotification
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)