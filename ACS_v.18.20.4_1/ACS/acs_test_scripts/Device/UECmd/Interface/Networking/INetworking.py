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
:summary: This file implements INetworking UECmds
:since: 19/07/2010
:author: sfusilie
"""
from ErrorHandling.DeviceException import DeviceException


class INetworking():
    """
    Abstract class that defines the interface to be implemented
    by network handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    # pylint: disable=W0613

    # Constant values for Wifi Sleep Policies.
    # To be overloaded in Platform specific classes
    WIFI_SLEEP_POLICY = {}

    # Constant for FTP transfer status
    # To be overloaded in Platform specific classes
    FTP_TRANSFERRING = ""

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def set_wificonfiguration(self, ssid, pass_phrase, security,
                              ip_method="", address="", netmask="",
                              gateway="", dns1="", dns2="", proxy_config="NONE",
                              proxy_address="", proxy_port="", proxy_bypass=None):
        """
        Sets passphrase for secured WIFI network

        :type ssid: str
        :param ssid: WIFI network SSID

        :type pass_phrase: str
        :param pass_phrase: Passphrase

        :type security: str
        :param security : wifi security name. Can be NONE, OPEN, WEP, WPA, WPA2,
         WEP64-OPEN, WEP128-OPEN, WEP64, WEP128, WPA2-PSK-AES, WPA-PSK-TKIP,
         WPA2-PSK-TKIP, WPA-PSK-AES, WPA-PSK, WPA-PSK-TKIP-AES,
         WPA2-PSK, WPA2-PSK-TKIP-AES, WPA-WPA2-PSK, WPA-WPA2-PSK-TKIP-AES,
         WPA-WPA2-PSK-TKIP, "WPA-WPA2-PSK-AES

        :type ip_method: str
        :param ip_method: Possible values are "dhcp" or "static"

        :type address: str
        :param address: The current configured IPv4 address.

        :type netmask: str
        :param netmask: The current configured IPv4 netmask.

        :type gateway: str
        :param gateway: The current configured IPv4 gateway.

        :type dns1: str
        :param dns1: The current IPV4 dns1 to configure.

        :type dns2: str
        :param dns2: The current IPV4 dns2 to configure.

        :type proxy_config: str
        :param proxy_config: Possible values are "NONE", "MANUAL" or "AUTO"

        :type proxy_address: str
        :param proxy_address: Address of the proxy

        :type proxy_port: str
        :param proxy_port: Port of the proxy

        :type proxy_bypass: str
        :param proxy_bypass: Addresses bypassed by the proxy separated by a comma

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_autoconnect_mode(self, interface, state):
        """
        Sets the autoconnect mode to on/off for a specific interface or all

        :type interface: str
        :param interface: interface to modify or 'all' for all interfaces

        :type state: str or int
        :param state: Can be ("on", "1" , 1) to enable autoconnect
                             ("0", "off" , 0) to disable autoconnect

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clean_all_data_connections(self):
        """
        Disable PDP context and remove all known Wifi networks

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_connect(self, ssid, check_connection=True, poor_connection_check=False):
        """
        Connects to a WIFI network using its SSID

        :type ssid: str
        :param ssid: WIFI network SSID
        :type check_connection: boolean
        :param check_connection: if connection is checked
        :type poor_connection_check: Boolean
        :param poor_connection_check: Disables (False) or enables (True) the poor connection check (JB or later).

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_disconnect(self, ssid):
        """
        Disconnects from a WIFI network using its SSID

        :type ssid: str
        :param ssid: WIFI network SSID

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_disconnect_all(self):
        """
        Disconnects from all WIFI networks

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_apn(self, interface, apn, user, password, protocol, mmsc, apn_type, set_default, clear_apn):
        """
        Set the APN for a PDP Context using its interface

        :type interface: str
        :param interface: Data Cellular interface

        :type apn: str
        :param apn: Cellular Access Point Name

        :type user: str
        :param user: The APN user

        :type password: str
        :param password: The APN user

        :type protocol: str
        :param protocol: The APN protocol ("IP", "IPV6", "IPV4V6")

        :type mmsc: str
        :param mmsc: The APN MMSC
        (ex : http://10.102.161.47:8080/8960=8960)

        :type apn_type: str
        :param apn_type: The APN Type (ex : default,mms)

        :type set_default: bool
        :param set_default: Boolean determining if new APN is set as default APN

        :type clear_apn: bool
        :param clear_apn: Boolean determining if existing APN(s) is/are keep or remove

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_apn(self):
        """
        Clear the current APN configuration on the DUT
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reset_apn(self):
        """
        Restores the default APN configuration of the DUT

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_pdp_context(self, interface, check=True):
        """
        Activates a Packet Data Protocol (PDP) Context using its interface

        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reactivate_pdp_context(self, interface, check=True):
        """
        Check if PDP is activated, if not activate it

        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def deactivate_pdp_context(self, interface, check=True):
        """
        Deactivates a Packet Data Protocol (PDP) context using its interface

        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context deactivation is checked (no check for GSM)

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_flight_mode(self, mode):
        """
        Sets the flight mode to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_flight_mode(self):
        """
        Returns the flight mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_ftp_xfer(self,
                       direction,
                       server_ip_address,
                       username,
                       password,
                       filename,
                       local_path,
                       loop,
                       client_ip_address):
        """
        Start a transfer of a file via FTP.

        :type direction: str
        :param direction: Transfer direction (UL/DL)
        :type server_ip_address: str
        :param server_ip_address: IP address of the FTP server
        :type username: str
        :param username: FTP account username
        :type password: str
        :param password: FTP account password
        :type filename: str
        :param filename: File name to be transfered
        :type local_path: str
        :param local_path: path where to read/save the FTP file (sdcard)
        :type loop: str
        :param loop: True to loop the transfer , False to not used
        :type client_ip_address: str
        :param client_ip_address: IP address of the DUT networking interface we want to use for FTP

        :rtype: operation status & output log & task id
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_ftp_xfer_success(self, filename, direction, task_id):
        """
        Returns FTP transfer success or failure

        :type filename: str
        :param filename: File name to be transfered
        :type direction: str
        :param direction: Transfer direction (UL/DL)
        :type task_id: str
        :param task_id: Transfer to be finished
        :rtype: boolean
        :return: xfer successfull
                - True (success)
                - False (failure)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_ftp_xfer(self, task_id, cancel_request=False):
        """
        Stop a transfer of a file via FTP.

        :type task_id: str
        :param task_id: Transfer to be stopped
        .. note:: this parameter is not used on Android

        :type cancel_request: bool
        :param cancel_request: Indicate if the stop result form a cancel request
                                (ie timeout) or normal behavior

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_ftp_xfer_status(self):
        """
        get the xfer status.

        :rtype: str
        :return: xfer status
                - "connecting"
                - "connected"
                - "transferring"
                - "disconnected"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def iperf(self, settings):
        """
        Measures throughputs using IPERF

        :type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration...
        Refer to Utilities/IPerfUtilities to get list of possible settings

        :rtype: measured throughput value
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_iperf(self):
        """
        Kill all iperf instance on embedded side
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def ping(self,
             server_ip_address,
             packet_size,
             packet_count,
             interval=1,
             flood_mode=False,
             blocking=True,
             source_address=None):
        """
        Pings from the DUT a server on the bench. the protocol to use (IPv4 or IPv6) is
        computed automatically.

        If you have several interfaces connected (ethernet, wifi, ...), you should want
        to specify on which interface the ping come from. You can do this by specifying
        a source IP address, thanks to source_address input parameter.

        :type server_ip_address: str
        :param server_ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packet to send

        :type interval: float
        :param interval: Interval in seconds between pings (only for IPv4)

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False otherwise.

        :type blocking: Boolean
        :param blocking: True if you want to throw an error when network is UNREACHABLE.
        The ping will silently fail if this parameter is set to False.

        :type source_address: str
        :param source_address: source IP address to use

        :rtype: Measure object (value,unit)
        :return: packet loss
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def ping6(self,
              ip_address,
              packet_size,
              packet_count,
              flood_mode=False,
              blocking=True,
              source_address=None):
        """
        Pings IPV6 addresses from the DUT a server on the bench

        If you have several interfaces connected (ethernet, wifi, ...), you should want
        to specify on which interface the ping come from. You can do this by specifying
        a source IP address, thanks to source_address input parameter.

        :type ip_address: str
        :param ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packet to send

        :type interval: float
        :param interval: Interval in seconds between pings

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False otherwise.

        :type blocking: Boolean
        :param blocking: True if you want to throw an error when network is UNREACHABLE.
        The ping will silently fail if this parameter is set to False.

        :type source_address: str
        :param source_address: source IP address to use

        :rtype: Measure object
        :return: packet loss
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_continuous_ping(self, ip_address):
        """
        Start continuous ping to a given address

        :type ip_address: str
        :param ip_address: IP address to ping

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_continuous_ping(self):
        """
        Stop the continuous ping ongoing

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_multiple_http_transfer(self, url, sync_interval, duration, agent="acs", filename="test_dl.data"):
        """
        Start http download to a given address every X millis during X seconds

        :type url: str
        :param url: the url to the file to download

        :type sync_interval: int
        :param sync_interval: the download interval

        :type duration: int
        :param duration: the duration until transfers will stop in seconds

        :type filename: str
        :param filename: optional parameter to specified the filename of downloaded file

        :type agent: str
        :param agent: agent to use to process the download

        :return: status & output
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_status_multiple_http_transfer(self):
        """
        Get status of transfers

        :return: downloading status, transfers count and transfers error
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_multiple_http_transfer(self, url, agent="acs"):
        """
        Stop the http transfers

        :type url: str
        :param url: the url of the ongoing transfer

        :type agent: str
        :param agent: agent to use to process the download

        :return: status, transfers count and transfers error
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_current_continuous_ping(self):
        """
        Get the current ping result of the ongoing continuous ping

        :rtype: (bool, float)
        :return: (ping success, ping rtt)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_continuous_ping_success(self, duration):
        """
        Checks that ongoing continuous ping is successful

        :type duration: float
        :param duration: duration of the check in seconds

        :rtype: (bool, float)
        :return: (result of the check, average rtt)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_continuous_ping_failure(self, duration):
        """
        Checks that ongoing continuous ping is failing

        :type duration: float
        :param duration: duration of the check in seconds

        :rtype: bool
        :return: result of the check
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def ftp_xfer(self,
                 direction,
                 server_ip_address,
                 username,
                 password,
                 filename,
                 timeout,
                 local_path,
                 target_throughput,
                 client_ip_address=""):
        """
        Transfers a file via FTP

        :type direction: str
        :param direction: Transfer direction (UL/DL)

        :type server_ip_address: str
        :param server_ip_address: IP address of the FTP server

        :type username: str
        :param username: FTP account username

        :type password: str
        :param password: FTP account password

        :type filename: str
        :param filename: File name to be transfered

        :type timeout: integer
        :param timeout: script execution timeout

        :type local_path: str
        :param local_path: path where to read/save the FTP file (sdcard)

        :type target_throughput: float
        :param target_throughput: target throughput for data transfer in kbps

        :type client_ip_address: str
        :param client_ip_address: IP address of the DUT networking interface we want to use for FTP

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_available_technologies(self):
        """
        Returns an array of available technologies.

        :rtype: list
        :return: List of technologies to activate if available
                                - cellular
                                - wifi
                                - bluetooth
                                - gps
                                - fm
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_power(self, mode):
        """
        Sets the WIFI power to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_power_status(self):
        """
        Gets the WIFI power.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_scanning(self, mode):
        """
        Sets the WIFI scanning to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_scanning_status(self):
        """
        Returns the WIFI scanning status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_set_scan_interval(self, scan_interval):
        """
        Sets the WIFI scan interval in seconds.

        :type scan_interval: int
        :param scan_interval: scan interval in seconds

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_connected_wifi(self):
        """
        Lists the connected WIFI.

        :rtype: list
        :return: The list connected wifi ssid(s)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_power_level(self):
        """
        Returns the Wifi TX power level in dBm.

        :rtype: float
        :return: wifi power in dBm
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_power_saving_mode(self, mode, interface):
        """
        Sets the power saving mode of the given wlan interface to the given value.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :type interface: str
        :param interface: the wlan interface name

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_mac_addr(self, interface="wifi"):
        """
        Returns the MAC address of the given interface.

        :type interface: str
        :param interface: the interface name.

        :rtype: str
        :return: The dut mac address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def request_wifi_scan(self):
        """
        Trigger a Wifi scan.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_ssids(self, technology="wifi", state="all"):
        """
        Lists the ssids for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: list
        :return: a list of ssids
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_ssids_and_capabilities(self, technology="wifi", state="all"):
        """
        Lists the ssids and capabilities for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: tuple
        :return: a tuple of lists ssids and capabilities
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def list_ssids_and_frequency(self, technology="wifi", state="all"):
        """
        Lists the ssids and frequency for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: tuple
        :return: a tuple of lists ssids and frequency
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_ssid_bfore_timeout(self, ssid, timeout, technology="all"):
        """
        Try to find a given ssid before timeout.

        :type ssid: str
        :param ssid: ssid to check

        :type timeout: int
        :param timeout: operation timeout

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :rtype: boolean
        :return: research result
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_ip_address(self, IPV6=False):
        """
        Returns the Wifi ip address.

        :type IPV6 Boolean
        :param IPV6 False to get IPV4 address, True to get the IPV6 one

        :rtype: str
        :return: wifi ip address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv4_address(self, interface):
        """
        Returns the ipv4 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv4_all_address(self):
        """
        Returns ipv4 addresses for all active network interface.

        :rtype: dict
        :return: {active interface name: ip address}
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_hotspot(self, state, hotspot_ssid="", hotspot_security="", hotspot_passphrase="", hotspot_standard="", hotspot_channel="", hotspot_hidden=""):
        """
        Set the SSID, security and password for the Configurable wifi hotspot.

        :type state: str
        :param state: wifi hotspot enable or disable: on|off
        :type hotspot_ssid: str
        :param hotspot_ssid: ssid of hotspot (required only if state=on)
        :type hotspot_security: str
        :param hotspot_security: security of hotspot, can be
                         OPEN|WPA-PSK|WPA2-PSK (required only if state=on)
        :type hotspot_passphrase: str
        :param hotspot_passphrase: password of hotspot,
                         (required only if state=on and hotspot_security!=OPEN)
        :type hotspot_standard: str
        :param hotspot_standard: standard of hotspot, optional, can be (2_4GHZ_20MHZ;5GHZ_20MHZ;5GHZ_40MHZ;5GHZ_80MHZ)
        :type hotspot_channel: str
        :param hotspot_channel: channel of hotspot, value "AUTO" by default (only if standard is used, optional)
        :type hotspot_hidden: str
        :param hotspot_hidden: hidden SSID
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_hotspot_status(self):
        """
        Get the status of the Wifi hotspot feature.

        :rtype: int
        :return: 1 if enabled, 0 if disabled
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_hotspot_parameters(self):
        """
        Get all parameters of the Wifi hotspot feature.

        :type state: str
        :param state: wifi hotspot parameter, can be SSID;SECURITY;PASSPHRASE;STANDARD;CHANNEL;HIDDEN

        :rtype: dict
        :return: all currents parameters : SSID, SECURITY, PASSPHRASE, STANDARD, CHANNEL, HIDDEN
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_remove_config(self, ssid):
        """
        Remove a wifi configuration for the device.

        :type ssid: str
        :param ssid: the ssid of the wifi configuration to be removed or "all"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def open_web_browser(self,
                         website_url,
                         browser_type="native",
                         timeout=None,
                         skip_eula=False):
        """
        Open the Web Browser on the web page
        passed as parameter.

        :type website_url: str
        :param website_url: URL to open

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :type timeout: int
        :param timeout: timeout to open the page

        :type skip_eula: boolean
        :param skip_eula: skip EULA on 1st start

        :rtype: tuple
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def close_web_browser(self, browser_type="native"):
        """
        Close the Android Browser.

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def push_wpa_certificate(self, certificate_file):
        """
        Push given certificate file from host to device.

        :param certificate_file: str
        :param certificate_file: Certificate file to push onto target
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def load_wpa_certificate(self, certificate_name=None,
                             eap_password=None,
                             credential_password=None):
        """
        Load the WPA certificate file from the SDCARD
        Prerequisite: A certificate file ".p12" should have been pushed into
        the folder /sdcard/.
        Warning, only 1 .p12 file must be present on the SDCARD
        Warning 2: if a credential password is set, it should be the PIN code
                    specified in the benchConfig (Credential_password)

        :type certificate_name: str
        :param certificate_name: Name to give to the certificate after loading

        :type eap_password: str
        :param eap_password: password to open the certificate file

        :type credential_password: str
        :param credential_password: password to set for the Android credential
                                    passwords

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_sleep_policy(self, policy):
        """
        Set the wifi sleep policy

        :type policy: WIFI_SLEEP_POLICY enum
        :param policy: policy to set
        .. seealso:: http://developer.android.com/reference/android/provider/Settings.System.html#WIFI_SLEEP_POLICY
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_sleep_policy(self):
        """
        get the wifi sleep policy

        :return: the wifi sleep policy
        .. seealso:: http://developer.android.com/reference/android/provider/Settings.System.html#WIFI_SLEEP_POLICY
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_menu_settings(self, displayed=True):
        """
        Enter WiFi menu setting or exit from WiFi menu settings
        This UEcmd is useful for WiFi policy tests

        :type displayed: bool
        :param displayed: True to enter into WiFi menu setting\
                            and False to exit (goes to idle screen)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_menu_advanced_settings(self):
        """
        Enter WiFi menu advanced setting or exit from WiFi menu advanced settings
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_connection_state(self, ssid, timeout=0):
        """
        Checks that the connected SSID is the one passed as parameter

        :type ssid: str
        :param ssid: SSID name of the Wifi network the DUT should be connected
        :type timeout: int
        :param timeout: Customize the timeout
                        (when timeout = 0 --> default value)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def redirect_log_on_dut(self):
        """
        Start logging logcat into a file on DUT
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_log_on_dut(self):
        """
        Stop logging logcat into a file on DUT
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_dhcp_renewal_interval_on_dut(self, nb_hits):
        """
        Computes the average dhcp renewal interval for nb_hits renewals

        :type nb_hits: int
        :param nb_hits: number of renewals for wich the average has to be computed

        :rtype: int or None
        :return: average renewal interval for nb_hits renewals or None if operation failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_dhcp_renewal_interval(self, nb_hits, timeout):
        """
        Computes the average dhcp renewal interval for nb_hits renewals

        :type nb_hits: int
        :param nb_hits: number of renewals for wich the average has to be computed

        :type timeout: int
        :param timeout: max number of seconds between 2 renewals

        :rtype: int or None
        :return: average renewal interval for nb_hits renewals or None if operation failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_dhcp_state(self):
        """
        Returns the Wifi DHCP state according to android NetworkInfo.State.

        :rtype: str
        :return: wifi DHCP state
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_wifi_dhcp_connected(self, timeout=None):
        """
        Polls until the IP address has been acquired.
        Then, DHCP state will be "CONNECTED".
        At the end of the default timeout, an exception will be raised

        :type timeout: int
        :param timeout: max number of seconds to wait for IP address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_regulatorydomain(self, regulatory_domain, interface="wlan0"):
        """
        Set the Wifi Regulatory Domain

        :type regulatory_domain: String
        :param regulatory_domain: the regulatory domain to set (FR, GB, US...)

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_regulatorydomain(self):
        """
        Get the Wifi Regulatory Domain

        :rtype: String
        :return: the regulatory domain (FR, GB, US... or "none")
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_frequency_band(self, freq_band, silent_mode=False, interface="wlan0"):
        """
        Set the Wifi Frequency Band

        :type freq_band: String
        :param freq_band: Frequency Band to set (auto, 2.4GHz, 5GHz)

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception
                            if the device does not support this method

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_wifi_connection_log(self):
        """
        Start logging information about Wifi networks connection
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_connection_status_log(self, ssid, timeout=120):
        """
        Wait for the next wifi connection failure or success

        :type ssid: str
        :param ssid: ssid to control the connection
        :type timeout: int
        :param timeout: time to wait until no response arrives (optional)

        :rtype: str
        :return: "SUCCESS" for a connection success
                "FAILURE" for a connection failure
                "TIMEOUT" for no information trigged in the limited time
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wifi_setkeyexchange(self, ssid, key_exchange_mode, key_exchange_pin=None,
                            simulate_faulty_connection=False, interface="wlan0"):
        """
        Sets key exchange mode (WPS for instance) for secured WIFI network.

        :type key_exchange_mode: str
        :param key_exchange_mode: Key exchange mode

        :type key_exchange_pin: str
        :param key_exchange_pin: PIN used by some key exchange modes (WPS_PIN_FROM_AP
        for instance)

        :type ssid: str
        :param ssid: WIFI network SSID

        :type simulate_faulty_connection: bool
        :param simulate_faulty_connection: If True, corrupts the data or wait for the
        AP's timeout to simulate a faulty simulation request

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_frequency_band(self, silent_mode=False, interface="wlan0"):
        """
        Gets the band selection (bands of frequencies)
        0 means dual
        1 means 5Ghz
        2 means 2.4Ghz

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception if the device does not support this method
        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: String
        :return: The band text (JB or later).
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_connection_rate(self, wifi_interface):
        """
        Get the Wifi connection rate

        :type wifi_interface: str
        :param wifi_interface: WiFi interface name (wlan0)

        :rtype: int
        :return: the connection rate in MByte/s. -1 in case of parsing error
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_usb_tethering(self, delay=(-1), unplug=False):
        """
        start the tethering connection over USB

        :type delay: int
        :param delay: activate the delayed command.
        :type unplug: boolean
        :param unplug: Unplug USB cable after delayed command in order for the ADB connection not to be lost.
        This option is usefull because USB tethering activation causes a short USB cable disconnection.
        If this parameter is set to True then delay will be force to 10 seconds at least.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_usb_tethering(self, delay=(-1), unplug=False):
        """
        stop the tethering connection over USB

        :type delay: int
        :param delay: activate the delayed command.
        :type unplug: boolean
        :param unplug: Unplug USB cable after delayed command in order for the ADB connection not to be lost.
        This option is usefull because USB tethering activation causes a short USB cable disconnection.
        If this parameter is set to True then delay will be force to 10 seconds at least.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_usb_tethering_state(self, expected_state, timeout=10):
        """
        Control that the USB Tethering feature is in the expected_state
        :type expected_state: boolean
        :param expected_state: the expected state to control
        :type timeout: int
        :param timeout: optional. Time during which to check the USB tethering status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def usb_tether(self, wifi_off, unplug_usb, use_flight_mode):
        """
        push and run a script that will execute the test in USB disconnected mode

        :type wifi_off: int
        :param wifi_off: 1 to turn wifi off during usb tethering

        :type unplug_usb: int
        :param unplug_usb: 1 to unplug usb during usb tethering

        :type use_flight_mode: int
        :param use_flight_mode: 1 to turn on flight mode during usb tethering

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv6_address(self, interface, timeout=10):
        """
        Returns the 1st ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,
        get_interface_ipv6_all_address will return all the addresses.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: str
        :return: interface ip address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv6_scopelink_address(self, interface="", timeout=10):
        """
        Returns the scope link ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: str
        :return: interface ip address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv6_global_address(self, interface="", timeout=10):
        """
        Returns the scope link ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: str
        :return: interface ip address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_ip_protocol(self, protocol):
        """
        Check IP protocol used

        :type protocol: str
        :param silent_mode: IP protocol selected on the DUT for data registration

        :rtype: String
        :return: The message containing the IP address and the protocol

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_no_ip_address(self):
        """
        Check that no IP address is given to the DUT (if data is not activated)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_ipv6_all_address(self, interface=""):
        """
        Returns all the ipv6 addresses of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: list of strings
        :return: interface ip addresses
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def copy_dhcp6_dns(self, interface='wlan0', dns_nr='1', is_rdnssd=False):
        """

        Copy the DNS of the given interface from dhcp6.<interface>.dns to
        net.<interface>.dns

        :type interface: String
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dns_nr: int
        :param dns_nr: dns server number.

        :type is_rdnssd: Boolean
        :param is_rdnssd: If true, the dns will be taken via the rdnssd property,
        If False, it will be done via DHCP.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_dhcp6_client(self, dhcp6_mode, interface):
        """
        Launched the ipv6 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dhcp6_mode: str
        :param dhcp6_mode: stateless or stateful
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_rdnssd_client(self, interface):
        """
        Launches the Recursive DNS Server on the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_ipv6_consistency(self, interface, ipv6_prefix=""):
        """
        Check that the local IPV6 address is compliant with the EUI-64 format.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type ipv6_prefix: String
        :param ipv6_prefix: If you want to check the IPV6 prefix in addition to
                            the loopback one, provide this parameter.

        :rtype: bool
        :return: True if the check is good, False else.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_ip_filtering(self, phy):
        """
        Disables all IP filtering

        :type: phy
        :param: phy the physical interface to disable the filter to.
        """
        pass

    def enable_ip_filtering(self, phy):
        """
        Enables all IP filtering

        :type: phy
        :param: phy the physical interface to enable the filter to.
        """
        pass

    def measure_wifi_turns_off_duration(self, mode, wlaninterface,
                                        duration=(-1), period=(-1),
                                        expected_duration=(-1), tolerance=(-1)):
        """
        Measure the time for WiFi interface to turn OFF.
        For test that should be performed unplugged, this function can be used
        with asynchronous mode.

        :type mode: str
        :param mode: Can be "sync", "async_init", "async_result"
        - "sync" -> Synchrone mode.
            The time to turn off WiFi is returned by the function
            An exception is raised measured value too far from
            expected_duration passed as parameter
        - "async_init" -> Asynchronous mode, intialisation.
            An embedded script to watch WiFi interface activation is launched
            on the DUT.
        - "async_result" -> Asynchronous mode, obtains the result
            Retrieves the result from the DUT and returns it.
            An exception is raised measured value too far from
            expected_duration passed as parameter
        :type wlaninterface: str
        :param wlaninterface: Wireless LAN interface of the DUT (ie: wlan0)
        :type duration: int
        :param duration: duration time while watching the WiFi interface (in s)
        :type period: int
        :param period: waiting time between occurences (in s)
                default is automatic value (duration / 100)
        :type expected_duration: int
        :param expected_duration: Expected time for WiFi to turn OFF in sec.
        :type tolerance: float
        :param tolerance: tolerance for expected_duration in percentage < 1

        :rtype: int or None
        :return: with sync and async_result modes, returns the time for WiFi to
            turn OFF in seconds. It returns None in async_init mode.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_roaming_mode(self):
        """
        Returns the roaming mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_roaming_mode(self, mode):
        """
        Sets the roaming mode to off or on.

        :type mode: str
        :param mode: can be 'on' to enable
                            'off' to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_preferred_network_type(self):
        """
        Returns the Preferred Network Type.

        :rtype: str
        :return:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_preferred_network_type(self, preferred_type):
        """
        Sets the Preferred Network Type .

        :type preferred_type: str
        :param preferred_type: can be:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def _get_wifi_chipset_manufacturer(self):
        """
        Get Wifi chipset manufacturer: CHIPSET_BROADCOM or CHIPSET_TI
        List of possible values are attributes of this class

        :rtype: str
        :return: The Wifi chipset used on the PHONE
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reset_mobile_broadband_mode(self, mode):
        """
        Sets MobileBroadband to off or on.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_output_traffic(self):
        """
        Prevent all traffic including icmp

        :param: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_output_traffic(self):
        """
        Allow UE traffic to go out

        :param: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_output_traffic_but_ping(self):
        """
        Prevent all traffic excluding ping

        :param: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_default_regulatory_domain(self):
        """
        Get default regulatory domain

        :rtype: str
        :return: The default regulatory domain
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_direct(self, mode):
        """
        Sets the WIFI direct feature ON or OFF.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_direct_status(self, mode):
        """
        Gets the WIFI direct status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_widi(self, mode):
        """
        Sets the WIDI feature ON or OFF.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_widi_status(self, mode):
        """
        Gets the WIDI status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_serving_cell(self):
        """
        Returns the current serving cell of the device.

        :rtype: acs_test_scripts.Utilities.RegistrationUtilities.CellInfo
        :return: the serving cell instance
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_neighboring_cells(self):
        """
        Returns the list of all neighboring cells.

        :rtype: list
        :return: a list of C{NeighboringCellInfo} instances
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def simulate_phone_pin_unlock(self, pin_lock_code=""):
        """
        Simulates the entry of the PHONE pin code by the user.
        This is intended to unlock the certificate for enterprise authentication if
        it protected by one.

        :type pin_lock_code: String
        :param pin_lock_code: The code used to lock the phone
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def lost_packets_iperf(self, queue):
        """
        Looks for indication of lost packet in the queue containing the output
        of an iperf client or server.
        :type queue: queue
        :param queue: queue of iperf command
        :rtype: boolean
        :return: True if lost packets are detected, False if not.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_rssi(self, ssid):
        """
        Returns the WiFi RSSI for a given SSID.

        :type ssid : str
        :param ssid: name of SSID to check

        :rtype: int
        :return: value of the RSSI in dBm
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_network_notification(self, mode):
        """
        Set the Network Notification option. Available in Advanced Wi-Fi settings menu.

        @type mode: string or int
        @param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_network_notification(self):
        """
        Get the Network Notification option. Available in Advanced Wi-Fi settings menu.

        @rtype: boolean
        @return: state of network notification (0 for disabled, 1 for enabled)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def clean_tcp_records_device(self):
        """
        Clean TCP records on Board
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_ims_registration_status(self):
        """
        Returns an integer description the IMS registration
        state of the device.
        :return: an integer describing the IMS registration status
            (-1 if a non-integer value has been returned by the API).
        :rtype: int
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def answerIMS(self):
        """
        Answers all voice calls.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def cellular_signal_properties(self):
        '''
           Returns a dictionary of properties about the cellular signal.
        '''
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_ims_registration_before_timeout(self, timeout):
        """
        Waits a maximum of the given timeout (in seconds) for IMS registration.

        If the IMS registration does not occur before the timeout, a
        DeviceException is raised.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_bandwidth(self, interface="wlan0"):
        """
        Get the Wifi current bandwidth

        :type interface: str
        :param interface: interface to check the bandwidth

        :rtype: String
        :return: the current bandwidth ("20", "40" or "80") in MHz
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_channel(self, interface="wlan0"):
        """
        Get the Wifi current channel

        :type interface: str
        :param interface: interface to check the channel

        :rtype: String
        :return: the current channel
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_wifi_scan_always_available(self, mode):
        """
        Set the WiFi Scan always available option in WiFi settings menu.
        WARNING : this function use UI !

        :type mode: str
        :param mode: mode to set. Possible values : "ON", "OFF"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_scan_always_available(self):
        """
        Get the WiFi Scan always available option in WiFi settings menu.

        :rtype: str
        :return: WiFi Scan always available option state. Possible values are "Enable" or "Disable".
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_preferred_network_mode(self, preferred_mode):
        """
        Sets the Preferred Network Type on user menu .

        :type preferred_mode:  str
        :param preferred_mode: can be:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remove_wpa_certificates(self, credential_password=None, pin_code_removal=False):
        """
        Remove wpa certificates already installed on device.
        Possibility to remove pin code already set.

        :type pin_code_removal: boolean
        :param pin_code_removal: Remove Pin Code

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
    def get_available_networks(self):
        """
        Returns the list of all available networks.

        :rtype: list
        :return: a list of [available networks]
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
