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
:summary: Implements Networking methods used over the whole ACS solution
:since: 13/07/2012
:author: apairex
"""

import re
import posixpath
import threading
import math
import time
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Error, Global
from threading import Thread
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException


class AcsWifiFrequencies(object):

    """
    This class contains Wifi global variable which can be used everywhere in acs framework
    This class will help avoiding error on frequency and 5G Standard
    """

    # Channels <--> Frequencies
    WIFI_CHANNELS_FREQUENCIES_2G = {0: 'least-congested', 1: '2412', 2: '2417',
                                    3: '2422', 4: '2427', 5: '2432', 6: '2437',
                                    7: '2442', 8: '2447', 9: '2452', 10: '2457',
                                    11: '2462', 12: '2467', 13: '2472', 14: '2484'}

    WIFI_CHANNELS_FREQUENCIES_5G = {0: 'least-congested', 34: '5170', 36: '5180',
                                    38: '5190', 40: '5200', 42: '5210', 44: '5220',
                                    46: '5230', 48: '5240', 52: '5260', 56: '5280',
                                    60: '5300', 64: '5320', 100: '5500',
                                    104: '5520', 108: '5540', 112: '5560',
                                    116: '5580', 120: '5600', 124: '5620',
                                    128: '5640', 132: '5660', 136: '5680',
                                    140: '5700', 149: '5745', 153: '5765',
                                    157: '5785', 161: '5805', 165: '5825'}

    WIFI_STANDARD_5G = ['a', 'ac', 'an', 'n5G']

    @staticmethod
    def get_frequency(channel):
        """
        Retrieve the frequency from a channel.

        :type channel: str
        :param channel: the channel to retrieve the frequency from.

        :rtype: str
        :return: the frequency - or '0' if frequency not found
        """
        channel_int = int(channel)
        try:
            if channel_int in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G:
                return AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G[channel_int]
            elif channel_int in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
                return AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G[channel_int]
        except (ImportError, IndexError, AttributeError):
            msg = "get_frequency invalid channel " + channel
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return '0'

    @staticmethod
    def get_wifi_channel_from_frequency(frequency):
        """
        Retrieve the frequency from a channel.

        :type frequency: str
        :param frequency: the frequency to retrieve the channel from.

        :rtype: int
        :return: the frequency - or 0 if frequency not found
        """
        try:
            if frequency in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G.values():
                return [channel for channel, freq in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G.items() if freq == frequency][0]
            elif frequency in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G.values():
                return [channel for channel, freq in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G.items() if freq == frequency][0]
        except (ImportError, IndexError, AttributeError):
            msg = "get_wifi_channel_from_frequency invalid frequency " + frequency
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return '0'


class SupplicantState(object):

    """
    State machine for WiFi supplicant state
    Here, we only handle the significant (and common to all securities) states
    This class has been developed to analyze connection failure
    and connection success
    """
    UNDEFINED = "UNDEFINED"
    SCANNING = "SCANNING"
    AUTHENTICATING = "AUTHENTICATING"
    ASSOCIATING = "ASSOCIATING"
    ASSOCIATED = "ASSOCIATED"
    COMPLETED = "COMPLETED"
    DISCONNECTED = "DISCONNECTED"
    GROUP_HANDSHAKE = "GROUP_HANDSHAKE"
    FOUR_WAY_HANDSHAKE = "4WAY_HANDSHAKE"

    MAX_CONNECTION_TRIES = 3

    # Pattern used to parse logcat in order to get wpa_spplicant lines that correspond to
    # either a State change or a authentication/association start
    STATE_LOGCAT_FILTER = "wpa_supplicant.*((State:)|(Trying to (authenticate|associate) with))"

    def __init__(self, initial_value="", max_tries=MAX_CONNECTION_TRIES):
        """
        Initialize the SupplicantState object.

        :type initial_value: String
        :param initial_value: One of the available state values define as
            global attribute of this class.
            This can also be a long str that contains the state
        :type max_tries: int
        :param max_tries: max number of expected failures for a connection fail
        """
        if "->" in initial_value:
            initial_value = initial_value.split('>')[1]

        # The initial state can be included in the str passed as parameter
        # Useful to directly give a log line to the Object
        if SupplicantState.SCANNING in initial_value:
            self.__value = SupplicantState.SCANNING
        elif SupplicantState.AUTHENTICATING in initial_value:
            self.__value = SupplicantState.AUTHENTICATING
        elif SupplicantState.ASSOCIATING in initial_value:
            self.__value = SupplicantState.ASSOCIATING
        elif SupplicantState.ASSOCIATED in initial_value:
            self.__value = SupplicantState.ASSOCIATED
        elif SupplicantState.GROUP_HANDSHAKE in initial_value:
            self.__value = SupplicantState.GROUP_HANDSHAKE
        elif SupplicantState.FOUR_WAY_HANDSHAKE in initial_value:
            self.__value = SupplicantState.FOUR_WAY_HANDSHAKE
        elif SupplicantState.COMPLETED in initial_value:
            self.__value = SupplicantState.COMPLETED
        elif SupplicantState.DISCONNECTED in initial_value:
            self.__value = SupplicantState.DISCONNECTED
        else:
            self.__value = SupplicantState.UNDEFINED

        # Initialize the failure and success flags
        self.__tries = max_tries
        self.__success = False
        self.__state_always_undef = True

    def get_state(self):
        """
        Accessor to the state that represents this object

        :rtype: String
        :return: The current supplicant state stored in this object
        """
        return self.__value

    def set_state(self, state):
        """
        Set the supplicant state in the current object.
        The state is updated only if the given parameter is in the next
        allowed states list.

        :type state: String or SupplicantState
        :param state: state to set

        :rtype: boolean
        :return: True if the state has been updated, False otherwise
        """
        # Get the list of next allowed states
        if state in self.__next_allowed_states():

            if (self.__value == SupplicantState.AUTHENTICATING
                    or self.__value == SupplicantState.ASSOCIATED
                    or self.__value == SupplicantState.ASSOCIATING
                    or self.__value == SupplicantState.FOUR_WAY_HANDSHAKE
                    or self.__value == SupplicantState.GROUP_HANDSHAKE) \
                    and state == SupplicantState.DISCONNECTED:
                # Transition from (AUTHENTICATING or ASSOCIATED) to DISCONNECT counts as
                # a new connection tentative failure
                self.__tries -= 1

            if state == SupplicantState.COMPLETED:
                # Connection Success has been detected
                self.__success = True

            if self.__state_always_undef and state != SupplicantState.UNDEFINED:
                # Unset the flag "never been in any state but UNDEFINED"
                self.__state_always_undef = False

            self.__value = state
            return True

        return False

    def __next_allowed_states(self):
        """
        Returns the list of all allowed states after the current known state
        In case of connection success:
        SCANNING->AUTHENTICATING->ignored states->COMPLETED
        SCANNING->ignored states->ASSOCIATED->ignored states->COMPLETED
        In case of connection failure
        SCANNING->AUTHENTICATING->ignored states->DISCONNECTED->SCANNING
        SCANNING->ignored states->ASSOCIATED->ignored states->DISCONNECTED->SCANNING

        :rtype: list of strings
        :return: List of next available states
        """
        if self.__value == SupplicantState.UNDEFINED:
            return [SupplicantState.SCANNING,
                    SupplicantState.AUTHENTICATING,
                    SupplicantState.ASSOCIATING,
                    SupplicantState.ASSOCIATED,
                    SupplicantState.FOUR_WAY_HANDSHAKE,
                    SupplicantState.GROUP_HANDSHAKE,
                    SupplicantState.COMPLETED,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.SCANNING:
            return [SupplicantState.AUTHENTICATING,
                    SupplicantState.ASSOCIATING,
                    SupplicantState.ASSOCIATED,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.ASSOCIATING:
            return [SupplicantState.ASSOCIATED,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.AUTHENTICATING:
            return [SupplicantState.COMPLETED,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.ASSOCIATED:
            return [SupplicantState.COMPLETED,
                    SupplicantState.DISCONNECTED,
                    SupplicantState.FOUR_WAY_HANDSHAKE]
        elif self.__value == SupplicantState.FOUR_WAY_HANDSHAKE:
            return [SupplicantState.GROUP_HANDSHAKE,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.GROUP_HANDSHAKE:
            return [SupplicantState.COMPLETED,
                    SupplicantState.DISCONNECTED]
        elif self.__value == SupplicantState.COMPLETED:
            return []
        elif self.__value == SupplicantState.DISCONNECTED:
            return [SupplicantState.SCANNING]

    def is_connection_failure(self):
        """
        Returns True if the number of connection failures has reached the
        maximum number of tries

        :rtype: Boolean
        :return: True if the number of connection failures has reached the
            maximum number of tries
        """
        return self.__tries <= 0

    def is_connection_failed_once(self):
        """
        Returns True if the state has been DISCONNECTED once

        :rtype: Boolean
        :return: True if the connection fails at least once
        """
        return self.__tries < SupplicantState.MAX_CONNECTION_TRIES

    def is_connection_success(self):
        """
        Returns True is the connection succeeded

        :rtype: Boolean
        :return: True if the connection succeed
        """
        return self.__success

    def get_state_always_undef(self):
        """
        get the flag "never been in any state but UNDEFINED"

        :rtype: boolean
        :return: the flag "never been in any state but UNDEFINED"
        """
        return self.__state_always_undef

    def __str__(self, *args, **kwargs):
        """
        Overwrite the object conversion to str
        """
        return self.__value


def is_valid_ipv4_address(ip_addr):
    """
    Checks if the ip_addr parameter is a valid ipv4 address

    :type ip_addr: str
    :param ip_addr: IP address to check

    :rtype: bool
    :return: True if the ip_addr parameter is a valid ip_addr
    address, else returns false
    """
    parts = ip_addr.split(".")

    if len(parts) != 4:
        return False

    for item in parts:
        if not str(item).isdigit():
            return False

        if not 0 <= int(item) <= 255:
            return False

    return True


def get_dut_ipv4_address(network_api, network_interface=None):
    """
    This function will scan various network interface to find a valid IPV4 address for the DUT

    :type network_api: object
    :param network_api: instance of network api object to get IPV4 address

    :type network_interface: str
    :param network_interface: interface name (wlan0/wlan1 etc...)

    :rtype: str
    :return: interface ip address
    """
    cellular_interface_list = ["usb", "rmnet", "veth", "Cellular", "Mobile broadband", "inm"]
    ip_address = ""
    if network_interface not in (None, ""):
        cellular_interface_list.insert(0, network_interface)
    ip_address_dict = network_api.get_interface_ipv4_all_address()
    LOGGER_TEST_SCRIPT.info("Retrieved IP addresses: %s " % str(ip_address_dict))

    # scan ip address dictionnary to find if the interface is in dictionnary
    for interface in cellular_interface_list:
        interface = interface.lower()
        keys = ip_address_dict.keys()
        for key in keys:
            if interface in str(key).lower():
                # For instance "usb" can be in "usb2" so return ip address for usb2
                ip_address = str(ip_address_dict[key])
                LOGGER_TEST_SCRIPT.info("IPV4 address found for interface %s: %s" % (key, ip_address))
                return ip_address

    msg = "Cellular interface scan failed no IPV4 address found !!!"
    LOGGER_TEST_SCRIPT.error(msg)
    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)


def wait_for_dut_ipv4_address(timeout, network_api, network_interface=None):
    """
    This function will scan various network interface to find a valid IPV4 address for the DUT
    until timeout expiration

    :type timeout: int
    :param timeout: maximum time to wait for IPV4 address

    :type network_api: object
    :param network_api: instance of network api object to get IPV4 address

    :type network_interface: str
    :param network_interface: interface name (wlan0/wlan1 etc...)

    :rtype: str
    :return: interface ip address
    """
    # wait until DUT get an IP address from NW
    timeout = time.time() + timeout
    ip_address = None
    while time.time() < timeout:
        try:
            ip_address = get_dut_ipv4_address(network_api, network_interface)
            if ip_address is not None:
                return ip_address
        except:
            pass
    if ip_address is None:
        msg = "Could not get an IPV4 address from NW after %s seconds" % timeout
        LOGGER_TEST_SCRIPT.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
    return ip_address


def is_valid_ipv6_address(ip_addr):
    """
    Checks if the ip parameter is a valid ipv6 address

    :type ip_addr: str
    :param ip_addr: IPV6 address to check

    :rtype: bool
    :return: True if the ip parameter is a valid ipv6
    address, else returns false
    """
    pattern = re.compile(r"""
        ^
        \s*                         # Leading whitespace
        (?!.*::.*::)                # Only a single :: allowed
        (?:(?!:)|:(?=:))            # Only :: allowed at the beginning (: alone not allowed)
        (?:                         # Repeat 6 times:
            [0-9a-f]{0,4}           #   A group of at most four hexadecimal digits
            (?:(?<=::)|(?<!::):)    #   allow only : or ::
        ){6}                        #
        (?:                         # Either
            [0-9a-f]{0,4}           #   Another group
            (?:(?<=::)|(?<!::):)    #   allow only : or ::
            [0-9a-f]{0,4}           #   Last group
            (?: (?<=::)             #   Allow :: as final part of the address
             |  (?<!:)              #
             |  (?<=:) (?<!::) :    #
             )                      # OR
         |                          #   A v4 address with NO leading zeros (a.b.c.d)
            (?:25[0-4]|2[0-4]\d|1\d\d|[1-9]?\d)
            (?: \.
                (?:25[0-4]|2[0-4]\d|1\d\d|[1-9]?\d)
            ){3}
        )
        \s*                         # Trailing whitespace
        $
    """, re.VERBOSE | re.IGNORECASE | re.DOTALL)
    return pattern.match(ip_addr) is not None


def is_valid_mac_address(mac):
    """
    Checks if the mac parameter is a valid mac address

    :type ip: str
    :param ip: MAC address to check

    :rtype: bool
    :return: True if the mac parameter is a valid mac address,\
            else returns false
    """
    if re.match("^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$", mac.strip()) is None:
        return False
    else:
        return True


def check_connection_lost(networking_api, server_to_ping, packet_size, packet_count, logger=None):
    """
    Checks connection to server_to_ping is not possible.
    Raises an Exception in case of ping command can connect to the server

    :type networking_api: Networking API Object
    :param networking_api: Networking API of the phone to test the connection
    :type server_ip_address: str
    :param server_ip_address: IP address to ping
    :type packet_size: integer
    :param packet_size: Packet size in bytes
    :type packet_count: integer
    :param packet_count: Number of packet to send
    :type logger: Logger object
    :param logger: Logger object to log information
    """
    # Check if data connection is broken
    try:
        packet_loss = networking_api.ping(server_to_ping, packet_size, packet_count)
    except AcsBaseException as uecmdex:
        if "network is unreachable" in uecmdex.get_error_message().lower():
            # OK: Connection lost
            if logger is not None:
                logger.info("OK: Connection lost")
            return
        raise

    if logger is not None:
        msg = "Measured Packet Loss: %.0f%s (Target: 99.9%%)" \
            % (packet_loss.value, packet_loss.units)
        logger.info(msg)

    if packet_loss.value < 99.9:
        msg = "Connection is still active. Ping packet loss [%s]" \
            % str(packet_loss.value)
        if logger is not None:
            logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)


def ping(networking_api,
         server_to_ping,
         packet_size,
         packet_count,
         target_ping_packet_loss_rate,
         logger=None,
         blocking=True):
    """
    Ping the server

    :type networking_api: Networking API Object
    :param networking_api: Networking API of the phone to test the connection
    :type server_ip_address: str
    :param server_ip_address: IP address to ping
    :type packet_size: integer
    :param packet_size: Packet size in bytes
    :type packet_count: integer
    :param packet_count: Number of packet to send
    :type target_ping_packet_loss_rate: integer
    :param target_ping_packet_loss_rate: target packet loss
    :type logger: Logger object
    :param logger: Logger object to log information
    :type blocking: Boolean
    :param blocking: If blocking is set to True,
    a exception will be raised, if it is set to False
    only a warning message will be displayed
    """
    # Check if data connection is broken
    try:
        packet_loss = networking_api.ping(server_to_ping, packet_size, packet_count)
    except AcsBaseException as uecmdex:
        if "network is unreachable" in uecmdex.get_error_message().lower():
            if blocking:
                raise DeviceException(DeviceException.OPERATION_FAILED, uecmdex.get_error_message())
            else:
                if logger is not None:
                    logger.warning("Connection lost: %s" % uecmdex.get_error_message())
        else:
            raise
    else:
        if logger is not None:
            msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)" \
                % (packet_loss.value,
                   packet_loss.units,
                   target_ping_packet_loss_rate,
                   packet_loss.units)
            logger.info(msg)

        # Compute verdict depending on % of packet loss
        if packet_loss.value <= target_ping_packet_loss_rate:
            msg = "Ping is successfull with Ping packet loss [%s]" \
                % str(packet_loss.value)
            if logger is not None:
                logger.info(msg)
        else:
            msg = "Ping as failed with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   target_ping_packet_loss_rate,
                   packet_loss.units)
            if blocking:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                if logger is not None:
                    logger.warning(msg)


def compute_verdict(expected_value, tolerance, mean_measure, std_dev, logger=None):
    """
    Compute the verdict based on the mean and standard deviation value
    :type expected_value: integer
    :param expected_value: the expected wifi connection time
    :type tolerance: integer
    :param tolerance: tolerance for the expected wifi connection time
    :type mean_measure: float
    :param mean_measure: the mean wifi connection time measured
    :type std_dev: float
    :param std_dev: the computed standard deviation
    :type logger: Logger object
    :param logger: Logger object to log information
    """
    if mean_measure < 0 or math.isnan(mean_measure):
        msg = "Invalid WiFi Connection Time Observed"
        if logger is not None:
            logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    if mean_measure > expected_value:
        msg = "Mean connection time (%f) != expected value [0, %f]" % (mean_measure, expected_value)
        if logger is not None:
            logger.error(msg)
        raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)

    threshold = expected_value * tolerance
    if std_dev > threshold:
        msg = "Connection time measure is not relevant. "
        msg += "Standard Deviation = %f. " % std_dev
        msg += "Expected value is lower than %f" % threshold
        if logger is not None:
            logger.error(msg)
        raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)

    msg = "Mean Connection time: %f " % mean_measure
    msg += "Std deviation: %f " % std_dev
    msg += "(expected value: %d)" % expected_value
    if logger is not None:
        logger.info(msg)

# ----------------------------------------------------------------------------


class ThreadWebBrowsing(threading.Thread):
    """
    Thread used to do a web browsing.
    This class raise an exception if there is any problem, and return logs status
    """

    WAIT_BETWEEN_PAGES = 10.0

    def __init__(self, exceptions_queue, phone_local_wifi, duration, *website_url):
        """
        Initialize the thread

        :type exceptions_queue : Queue object
        :param exceptions_queue : used to send exceptions to the main thread
        :type phone_local_wifi : Networking API Object
        :param phone_local_wifi : Networking API of the phone to do the scan
        :type duration : integer
        :param duration : duration of the activitys
        :type web_site_url : String list
        :param web_site_url : List of all URL web sites used for the test
        """
        threading.Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_local_wifi = phone_local_wifi
        self._duration = duration
        self._website_url = website_url
        self._error = Error()

    def run(self):
        """
        Execute the thread
        """
        try:
            start = time.time()
            while(start + self._duration) > time.time():
                for element in self._website_url:
                    time.sleep(self.WAIT_BETWEEN_PAGES)
                    (self._error.Code, self._error.Msg) = \
                        self._phone_local_wifi.open_web_browser(element, "acs_agent", 30)
                    if self._error.Code != Global.SUCCESS:
                        self._phone_local_wifi.close_web_browser("acs_agent")
                        msg = str.format("Web browsing fail on {0} - {1}", str(element), self._error.Msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self._phone_local_wifi.close_web_browser("acs_agent")

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadWebBrowsing, self).join()
        return self._error.Code, self._error.Msg


class ThreadDownloadFileFromFTP(threading.Thread):
    """
    Thread used to do a web browsing.
    This class raise an exception if there is any problem, and return logs status
    """
    def __init__(self, exceptions_queue, phone_networking, ftp_address, ftp_login, ftp_password,
                 ftp_remote_file, ftp_path_remote_file, ftp_local_folder, timeout):
        """
        Initialize the thread

        :type exceptions_queue : Queue object
        :param exceptions_queue : used to send exceptions to the main thread
        :type phone_networking : Networking API Object
        :param phone_networking : Networking API of the phone to do the scan
        :type ftp_address : String
        :param ftp_address : Address of the FTP where is located the file
        :type ftp_login : String
        :param ftp_login : Login to connect to the FTP
        :type ftp_password : String
        :param ftp_password : Password to connect to the FTP
        :type ftp_remote_file : String
        :param ftp_remote_file : Name of the file which will be downloaded
        :type ftp_path_remote_file : String
        :param ftp_path_remote_file : Folder where is located the file on the FTP
        :type ftp_local_folder : String
        :param ftp_local_folder : Folder where the file will be stored locally
        :type timeout : integer
        :param timeout : time before return an error
        """
        threading.Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_networking = phone_networking
        self._ftp_address = ftp_address
        self._ftp_login = ftp_login
        self._ftp_password = ftp_password
        self._ftp_remote_file = ftp_remote_file
        self._ftp_path_remote_file = ftp_path_remote_file
        self._ftp_local_folder = ftp_local_folder
        self._timeout = timeout

        # Build complete path of FTP file
        self._ftp_complete_path = posixpath.join(ftp_path_remote_file, ftp_remote_file)

    def run(self):
        """
        Execute the thread
        """
        try:
            ftp_transfer_id = self._phone_networking.start_ftp_xfer("DL", self._ftp_address, self._ftp_login,
                                                                    self._ftp_password, self._ftp_complete_path,
                                                                    self._ftp_local_folder)
            start = time.time()
            while start + self._timeout > time.time():
                if self._phone_networking.get_ftp_xfer_status() != 'transferring':
                    break
                time.sleep(2.5)

            if not self._phone_networking.is_ftp_xfer_success(self._ftp_remote_file, "DL", ftp_transfer_id):
                self._phone_networking.stop_ftp_xfer(ftp_transfer_id)
                msg = str.format("Timeout downloading file from FTP")
                raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)
        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.

        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadDownloadFileFromFTP, self).join()
        return


# ----------------------------------------------------------------------------
class ThreadScanWiFi(Thread):
    """
    Thread used to do a WiFi scan.
    This class raise an exception if the WiFi ssid of reference is not found
    """

    WAIT_BETWEEN_COMMANDS = 15.0

    def __init__(self, exceptions_queue, phone_local_wifi, phone_wifi_ref_ssid, duration=1.0):
        """
        Initialize the thread

        :type exceptions_queue : Queue object
        :param exceptions_queue : used to send exceptions to the main thread
        :type phone_local_wifi : Networking API Object
        :param phone_local_wifi : Networking API of the phone to do the scan
        :type phone_wifi_ref_ssid : String
        :param phone_wifi_ref_ssid : Reference SSID network the phone need to find
        :type duration : integer
        :param duration : max time of scans
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._phone_local_wifi = phone_local_wifi
        self._phone_wifi_ref_ssid = phone_wifi_ref_ssid
        self._duration = duration

    def run(self):
        """
        Execute the thread
        """

        try:
            start = time.time()
            while (start + self._duration) > time.time():
                self._phone_local_wifi.request_wifi_scan()
                time.sleep(ThreadScanWiFi.WAIT_BETWEEN_COMMANDS)
                list_scanned_wifi = self._phone_local_wifi.list_ssids("wifi", "all")

                device_found = False
                for element in list_scanned_wifi:
                    if str(element) == str(self._phone_wifi_ref_ssid):
                        device_found = True
                        break
                if not device_found:
                    msg = "WiFi Scan - device %s not found" % self._phone_wifi_ref_ssid
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)

    def join(self, timeout=180):
        """
        Wait the thread finish.
        :type timeout : integer
        :param timeout : maximum time of wait
        """
        super(ThreadScanWiFi, self).join()
        return
