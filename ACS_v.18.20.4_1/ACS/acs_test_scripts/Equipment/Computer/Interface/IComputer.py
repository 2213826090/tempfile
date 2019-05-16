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
:summary: virtual interface for Computer
:since:24/04/2012
:author: apairex
.. note:: BZ2832
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IComputer(object):

    """
    Virtual interface for Computer
    """

    def init(self, force_init=False):
        """
        Initializes the Computer software equipment
        For Remote Computer, the connection to the host through ssh, is done in this method

        :type force_init: boolean
        :param force_init: Force the initialization or do nothing in case of the initialization as already been done.
                            This parameter is useful to recover from computer connection crash
        :rtype: boolean
        :return: for Remote computers, True means that the connection was required
                                        False means that the connection was already active

        .. warning:: ACS account ssh public key must be installed into the user account
                  of the remote host in .ssh/authorized_keys, for Remote Computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Release the Computer software equipment
        For Remote Computer, Disconnects the ssh connection
        For Local Computer, Terminate asynchronous process
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def read(self, timeout=1):
        """
        Reads the stdout. Can be used to flush the stdout.
        For local computer this function is used to retrieve stdout for asynchronous process.

        :type timeout: int
        :param timeout: timeout to wait for stdout in seconds

        :rtype: str
        :return: stdout data
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def run_cmd(self, cmd, timeout=1):
        """
        Execute a command on the computer (Local or Remote)

        :type cmd: str
        :param cmd: command to execute on the remote host

        :type timeout: int
        :param timeout: timeout to wait for stdout (in sec.)
                        0  -> stdout and stderr are not read
                        -1 -> On LOCAL Computer the cmd is launched asynchronously
                        (Only 1 cmd can be launched asynchronously in the same time)
        :type stdout: str
        :param stdout: Used with Popen.  Valid values are PIPE, an existing file descriptor (a positive integer),
         an existing file object, and None.
        :type stderr: str
        :param stderr: Used with Popen.  Valid values are PIPE, an existing file descriptor (a positive integer),
         an existing file object, STDOUT and None.

        :rtype: String, String, Int
        :return: stdout and stderr data reply from command but will return string saying "timeout" if a timeout > 0
         has expired or None if timeout is 0. return the cmd return code too
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ssh_exec(self, host, login, cmd):
        """
        Connect to a host through ssh and execute a command synchronously

        :type host: str
        :param host: IP address to connect to

        :type login: str
        :param login: ssh account to log in

        :type cmd: str
        :param cmd: shell command to execute

        :rtype: output from stdout
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ping(self,
             ip_address,
             packet_size,
             packet_count,
             interval=1,
             flood_mode=False):
        """
        Pings a destination address

        :type ip_address: str
        :param ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packets to send

        :type interval: float
        :param interval: Interval in seconds between pings

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False else.

        :rtype: Measure object
        :return: Packet loss in %
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def dhclient(self, interface):
        """
        Configure a network interface using DHCP

        :type interface: str
        :param interface: interface to configure (wlan1/ra0/usb0)

        :rtype: tuple of strings
        :return: IP addresses of station and gateway
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def change_route(self, network, gateway):
        """
        Change the routing tables

        :type network: str
        :param network: network whose route is to replace

        :type gateway: str
        :param gateway: new gateway address

        :rtype: tuple of strings
        :return: destination, netmask and interface
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def restore_route(self, network, gateway, destination, netmask, iface):
        """
        Set the routing tables back to their original values

        :type network: str
        :param network: network whose route is to replace

        :type gateway: str
        :param gateway: new gateway address

        :type destination: str
        :param destination: original route destination

        :type netmask: str
        :param netmask: original netmask

        :type iface: str
        :param iface: original interface
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def copy_file_in_local_path(self, source, destination):
        """
        Copy a file located on the COMPUTER (local or remote) into a local path.
        For Remote computer, it downloads the file from the remote host using scp.

        :type source: str
        :param source: path name of source file

        :type destination: str
        :param destination: path name of destination file

        :rtype: str
        :return: stdout/stderr reply for the command

        .. warning:: For RemoteComputer usage, ACS account ssh public key must be installed
                  into the user account of the remote host in .ssh/authorized_keys
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def copy_file_in_remote_path(self, source, destination):
        """
        Copy a file located on the host into the COMPUTER (local or remote) path.
        For Remote computer, it uploads the file from the local host using scp.

        :type source: str
        :param source: path name of source file

        :type destination: str
        :param destination: path name of destination file

        :rtype: str
        :return: stdout/stderr reply for the command
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def wifi_connect(self, interface, standard, ssid, security, passphrase):
        """
        Configure and connect a remote wifi interface to a given profile

        :type interface: str
        :param interface: wifi interface to configure (wlan1/ra0)

        :type standard: str
        :param standard: wifi standard to use (a/b/g/n)

        :type ssid: str
        :param ssid: SSID of AP to connect to

        :type security: str
        :param security: type of security (OPEN/WPA2-PSK-xxx)

        :type passphrase: str
        :param passphrase: wifi passphrase for WPA2

        :rtype: str
        :return: ip address of the wifi interface
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def wifi_disconnect(self, interface):
        """
        Disconnect a remote wifi interface

        :type interface: str
        :param interface: wifi interface to configure (wlan1/ra0)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_iperf_server(self, settings):
        """
        Execute iperf server command on host

        :type settings: dictionary
        :param settings: Settings to launch iperf as server
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_iperf_server(self):
        """
        Terminate iperf server command on host
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def iperf_client(self, settings):
        """
        Execute iperf client command on host

        :type settings: dictionary
        :param settings: Dictionary containing all settings to launch iperf as server

        :rtype: str
        :return: Command result
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ipv6_addresses(self):
        """
        Returns the ipv6 address of the given interface.

        :rtype: array of strings
        :return: List of interface's IP addresses
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ipv6_prefix(self):
        """
        Returns the IPV6 network prefix.

        :rtype: String
        :return: The IPV6 network prefix.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_stop_service(self, service_name, mode):
        """
        Launches or stops a service on the computer.

        :type service_name: String
        :param service_name: The name of the service you want to start or stop.

        :type mode: String
        :param mode: start or stop
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_host_on_test_network(self):
        """
        return configured host from Bench_Config
        This host name or IP is the one accessible from DUT point of view.
        This is only used on Conformance Bench,
        where the Test network is different from the Control Network

        :rtype: str
        :return: host name or IP, from DUT point of view (Test Network)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_interface_mac_addr(self, interface):
        """
        Get the MAC address of the interface specified as parameter

        :type interface: str
        :param interface: wifi interface to get MAC address from (wlan1/ra0)

        :rtype: String
        :return: MAC address of the interface given as parameter,
                 characters are in lower case for hexadecimal values
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_wifi_interface(self):
        """
        return configured wifi interface from Bench_Config

        :rtype: str
        :return: Wifi interface name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_usb_interface(self):
        """
        return configured usb interface from Bench_Config

        :rtype: str
        :return: usb interface name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_tethering_interface(self):
        """
        return configured tethering interface from Bench_Config

        :rtype: str
        :return: usb interface name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_os(self):
        """
        Returns the Operating System installed on the COMPUTER
        For REMOTE_COMPUTER it always returns LINUX

        :rtype: str
        :return: the OS installed on the computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_command(self, binary_to_check):
        """
        Check that binary_to_check command is installed on the host computer.
        This function is used to check SSH and SCP command are available on the COMPUTER

        :type binary_to_check: str
        :param binary_to_check: Binary cmd to check if available on the Computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def web_browsing_test(self, url, timeout):
        """
        Simulate a web browsing and connect to the given URL

        :type url: String
        :param url: The URL to connect to
        :type timeout: int
        :param timeout: the timeout to abandon the connection tentative
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_from_bench_network(self):
        """
        Disable network interface used to connect to the bench network
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_from_bench_network(self):
        """
        Enable network interface used to connect to the bench network
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ftp_xfer(self, direction, server_ip_address, username, password, filename, timeout):
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
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def clean_tcp_records_computer(self):
        """
        Clean TCP records on computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def mount_host_partition(self, partition, mount_point):
        """
        Mount a partition on computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def unmount_host_partition(self, mount_point):
        """
        Unmount a partition on computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def delete_host_file(self, file_name):
        """
        Delete a file on the host computer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_date_time(self):
        """
        Get current date and time of a given computer.

        :rtype: dict
        :return: all currents parameters : DAY, MONTH, YEAR, HOURS, MINUTES, SECONDS
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

