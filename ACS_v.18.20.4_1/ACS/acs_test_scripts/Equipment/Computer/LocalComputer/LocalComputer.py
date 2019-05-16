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
:summary: Class to provide utilities to run from a Local Computer connected
to the IP network of the bench
:since:24/04/2012
:author: apairex BZ2832
"""

import sys
import subprocess
import platform
import re
import shlex
import time
import telnetlib
from re import match
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure, XFER_DIRECTIONS
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.Computer.Common.Common import GenericComputer
from Queue import Queue
from threading import Thread
from UtilitiesFWK.Utilities import BenchConfigParameters
from UtilitiesFWK.AcsSubprocess.AcsSubprocess import enqueue_output
from Queue import Empty
from UtilitiesFWK.Utilities import run_local_command
from ErrorHandling.AcsConfigException import AcsConfigException
from tempfile import gettempdir
import os
from ErrorHandling.DeviceException import DeviceException
ON_POSIX = 'posix' in sys.builtin_module_names
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
import unicodedata
from ftplib import FTP


class LocalComputer(GenericComputer):

    """
    Common class to provide utilities
    to run from a Local Computer connected to the IP network of the bench
    """

    def __init__(self, name, model, eqt_params):
        """
        Constructor
        """
        GenericComputer.__init__(self, name, model, eqt_params)

        # Retrieve the OS name
        self._os = platform.system()
        if self._os != self.WINDOWS and self._os != self.LINUX:
            msg = "Operating System not recognized [%s]" % self._os
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

        self._usb_interface = str(eqt_params.get_param_value("USBInterface", ""))
        self._bench_network_if = str(eqt_params.get_param_value("BenchNetInterface", ""))

        self._tethering_interface = str(eqt_params.get_param_value("TetherInterface", "RNDIS"))
        self._default_net_interface = "eth0"

        # For Asynchronous process to run
        self._async_process = None
        self._queue = None

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
        return False

    def release(self):
        """
        Release the Computer software equipment
        For Remote Computer, Disconnects the ssh connection
        For Local Computer, Terminate asynchronous process
        """
        if self._async_process is not None:
            self._async_process.terminate()  # pylint: disable=E1101
            self._async_process = None
            self._queue = None

    def run_cmd(self, cmd, timeout=1, stdout=subprocess.PIPE, stderr=subprocess.PIPE):
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
        output = ''
        std_err = ''
        ret_code = None

        # callable for call with timeout
        def timed_process(process):
            process.wait()

        self._logger.debug("RUN CMD: %s", cmd)
        shell_option = False

        if shell_option:
            # It is recommended to give the command as a str
            if isinstance(cmd, type([])):
                cmd = " ".join(cmd)
        else:
            # It is recommended to give the command as a list
            if isinstance(cmd, type("")) or isinstance(cmd, type(u"")):
                cmd = shlex.split(cmd)

        # first create th process
        process = subprocess.Popen(cmd,
                                   shell=shell_option,
                                   stdout=stdout,
                                   stderr=stderr,
                                   bufsize=1,
                                   close_fds=ON_POSIX)
        if timeout < 0:
            # execute the process asynchronously
            if self._async_process is not None:
                # Only 1 async process can be run
                self._async_process.terminate()

            self._queue = Queue()
            self._async_process = process
            # Need to avoid enqueue_output when not using PIPE because it is a utility function that will consume
            # stdout stream and put the content in a queue.
            if stdout == subprocess.PIPE:
                thread = Thread(target=enqueue_output, args=(process.stdout, self._queue))
                thread.daemon = True  # thread dies with program
                thread.start()
        elif timeout == 0:
            # no timeout. just wait end of process
            out = process.communicate()
            output = out[0]
            std_err = out[1]
        elif timeout > 0:
            # timed execution, start a timer
            thread = Thread(target=timed_process, args=(process,))
            thread.start()
            thread.join(timeout)
            if thread.is_alive():
                process.terminate()
                # Using join() will help us make sure the thread has terminated as expected.
                thread.join()
                output = "timeout"
                std_err = "timeout"
            else:
                out = process.communicate()
                output = out[0]
                std_err = out[1]

        ret_code = process.returncode
        self._logger.debug("RUN CMD std out: %s", output)
        self._logger.debug("RUN CMD std err: %s", std_err)
        self._logger.debug("RUN CMD process return code %s", ret_code)
        return {"std": output, "err": std_err, "ret": ret_code}

    def ping(self,
             ip_address,
             packet_size,
             packet_count,
             interval=1,
             flood_mode=False,
             unreach_loss=False):
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

        :type unreach_loss: Boolean
        :param unreach_loss: True if you want to count Destination host unreachable reply as ping loss, False else.

        :rtype: Measure object
        :return: packet loss
        """

        if int(packet_count) > 0:
            if int(interval) > 0:
                ping_timeout = int(packet_count * interval) + 20
            else:
                ping_timeout = int(packet_count) + 20
        else:
            self._logger.error("'packet count' must be an Integer > 0")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "'packet count' must be an Integer > 0")

        if not flood_mode:
            flood_option = ""
            flood_txt = ""
        else:
            flood_option = " -f"
            flood_txt = "(flood mode) "

        self._logger.info("Ping address " + flood_txt + str(ip_address) +
                          " with " + str(packet_count) + " packets of " +
                          str(packet_size) + " bytes...")

        # Compute whether ip_address is V4 or V6 format.
        if NetworkingUtil.is_valid_ipv4_address(str(ip_address)):
            linux_cmd = "ping"
            win_cmd = "ping -4"
        elif NetworkingUtil.is_valid_ipv6_address(str(ip_address)):
            linux_cmd = "ping6"
            win_cmd = "ping -6"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "%s is not a valid IP address" % str(ip_address))

        if self._os == self.WINDOWS:
            if packet_count > 1 and interval != 1:
                # no interval option on Windows ping, loop
                temp_ret = 0
                for _count in range(packet_count):
                    start_time = time.time()
                    ret = self.ping(ip_address,
                                    packet_size,
                                    1,
                                    1,
                                    flood_mode)
                    temp_ret += ret.value
                    end_time = time.time()
                    if end_time - start_time < interval:
                        time.sleep(interval - (end_time - start_time))
                ret.value = temp_ret / packet_count
                return ret
            cmd = str(win_cmd) + " -l " + str(packet_size) + \
                " -n " + str(packet_count) + " " + str(ip_address)
        else:
            cmd = str(linux_cmd) + " -s " + str(packet_size) + flood_option + \
                " -c " + str(packet_count) + " -i " + str(interval) + \
                " " + str(ip_address)

        output = self.run_cmd(cmd, ping_timeout)
        output = output["std"]

        output_lower = output.lower()
        packet_loss = Measure()

        # Search the loss rate
        if unreach_loss:
            # Get number of sent packet in output (here nb sent packet = 4):
            # Packets: Sent = 4, Received = 4, Lost = 0 (0% loss)
            search1 = re.search("packets: sent = ([0-9]+)", output_lower)
            # Get number of valid ping replies, a valid replies looks like:
            # Reply from 127.0.0.1: bytes=32 time<1ms TTL=128
            search2 = re.findall("ttl=([0-9]+)", output_lower)
            if search1 is not None:
                nb_sent_ping = int(search1.group(1))
                nb_good_ping = len(search2)
                packet_loss.units = "%"
                packet_loss.value = (100 * (nb_sent_ping - nb_good_ping)) / nb_sent_ping
            else:
                error = "Ping command returned an error message (Output = %s)" \
                    % output
                self._logger.error(error)
                raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, error)
        else:
            search = re.search("([0-9]+\.?[0-9]*)%[a-zA-Z ]*loss", output_lower)
            if search is not None:
                packet_loss.value = float(search.group(1))
                packet_loss.units = "%"
            else:
                error = "Ping command returned an error message (Output = %s)" \
                    % output
                self._logger.error(error)
                raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, error)
        return packet_loss

    def dhclient(self, interface):
        """
        Configure a network interface using DHCP

        :type interface: str
        :param interface: interface to configure (wlan1/ra0/usb0)

        :rtype: tuple of strings
        :return: IP addresses of station and gateway
        """
        self.get_logger().info("dhclient %s" % interface)

        if self._os == self.WINDOWS:
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

        # On Ubuntu OS starting from release 12, we need to add the -v option else nothing is displayed on the screen
        # and therefore no parsing can be done.
        verbose_option = ""
        os_maj_rel = self._get_os_major_release()
        if os_maj_rel >= 12:
            verbose_option = "-v "

        nbr_retry = 3
        found = None
        while found is None and nbr_retry > 0:
            # MUST be root to kill the dhclient
            self.ssh_exec("localhost", "root", "killall dhclient")
            output = self.ssh_exec("localhost", "root", "dhclient %s%s" % (verbose_option, interface))
            self._logger.debug(output)
            # look for "DHCPACK of 192.168.42.246 from 192.168.42.129"
            found = re.search(r"DHCPACK of ([0-9.]+) from ([0-9.]+)", output)
            nbr_retry -= 1
            if found is None and nbr_retry > 0:
                time.sleep(2)

        if found is None:
            msg = "Could not get IP address of interface %s" % interface
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        return found.groups()[0], found.groups()[1]

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
        if self._os == self.WINDOWS:
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

        destination = None
        netmask = None
        iface = None

        # MUST be root to configure a network route
        output = self.ssh_exec("localhost", "root", "route -n")
        routes = output.split("\n")
        for route in routes:
            pattern = r"([0-9.]+)\s+[0-9.]+\s+([0-9.]+)\s+[A-Z!]+\s+(?:[0-9]+\s+){3}([A-Za-z0-9]+)"
            res = re.search(pattern, route)
            if res is not None and res.groups()[0] == network:
                destination = res.groups()[0]
                netmask = res.groups()[1]
                iface = res.groups()[2]
                self.ssh_exec("localhost", "root", "route del -net %s netmask %s dev %s" % (destination, netmask, iface))

        if destination is None or netmask is None or iface is None:
            self._logger.warning("Can't find original IP route to delete.")
        # change route for our Access Point
        self.ssh_exec("localhost", "root", "route add -net %s netmask 255.255.255.0 gw %s" % (network, gateway))
        return destination, netmask, iface

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

        :return: None
        """
        if self._os == self.WINDOWS:
            msg = "Restore route function is not available on Windows OS"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)

        # MUST be root to configure a network route
        self.ssh_exec("localhost", "root",
                      "route del -net %s netmask 255.255.255.0 gw %s" % (network, gateway))
        if destination is not None and netmask is not None and iface is not None:
            self.ssh_exec("localhost", "root",
                          "route add -net %s netmask %s dev %s" % (destination, netmask, iface))

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
        if self._os == self.WINDOWS:
            cmd = "copy %s %s" % (source, destination)
        else:
            cmd = "cp %s %s" % (source, destination)

        out = self.run_cmd(cmd)

        return out["std"]

    def get_host_on_test_network(self):
        """
        return configured host from Bench_Config
        This host name or IP is the one accessible from DUT point of view.
        This is only used on Conformance Bench,
        where the Test network is different from the Control Network

        :rtype: str
        :return: host name or IP, from DUT point of view (Test Network)
        """
        return self._get_current_ipv4()

    def _get_current_ipv4(self):
        """
        Return the current IPv4 used on the default network interface
        Since the local computer is never used as a network bridge,
        it only has 1 network interface then 1 IP.

        :rtype: str
        :return: IP on the Test Network
        """
        if self._os == self.LINUX:
            cmd = "ifconfig %s" % self._default_net_interface
            pattern = r'inet addr:((?:[0-9]{1,3}\.){3}[0-9]{1,3})'

        else:
            # OS Windows
            cmd = "ipconfig"
            pattern = r'IPv4 Address[ \.]*: *((?:[0-9]{1,3}\.){3}[0-9]{1,3})'

        out = self.run_cmd(cmd)
        out = out["std"]
        ip_address = re.findall(pattern, out, re.MULTILINE)
        if len(ip_address) == 0:
            msg = "Unable to parse IP address from ifconfig command. Ouput: " + out
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        if len(ip_address) > 1:
            msg = "Several IPs have been found. The first one is used: " + str(ip_address)
            self._logger.warning(msg)

        return ip_address[0]

    def web_browsing_test(self, url, timeout=None):
        """
        Simulate a web browsing and connect to the given URL

        :type url: String
        :param url: The URL to connect to
        :type timeout: int
        :param timeout: the timeout to abandon the connection tentative
        """
        # This function opens a telnet connection to the given port (80 by default) and send an http GET request to
        # the given HTTP path

        # default values:
        port = 80
        http_path = "/"

        # So, 1st, let's parse the URL to extract IP, port and HTTP path
        parse = re.search(r'(?:https?://)?([^:/$]+)(?::([0-9]+))?(/.*)?', url)
        if parse is None:
            msg = "Unable to extract host, port and path from URL: %s" % url
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        host = parse.group(1)
        if parse.group(2):
            port = parse.group(2)
        if parse.group(3):
            http_path = parse.group(3)

        self._logger.debug("HOST=%s PORT=%s PATH=%s" % (host, port, http_path))

        # Initialize telnet session
        telnet_session = telnetlib.Telnet()
        # debug level: 0->disable / 1->enable
        telnet_session.set_debuglevel(0)

        try:
            if timeout is None:
                telnet_session.open(host, port)
            else:
                telnet_session.open(host, port, timeout)

            out = telnet_session.read_lazy()
            if out != "":
                msg = "Unable to connect to " + host
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
            telnet_session.write("GET %s\n" % http_path)
            out = telnet_session.read_until("</html>", timeout)
            self._logger.debug("HTTP response:\n" + out)

            # Analyze output and search for an XML type output
            if re.search(r'<[^>]+>.*</[^>]+>', out, re.MULTILINE) is None:
                msg = "Unable to send GET request to the WEB server. out=" + out
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)
        finally:
            telnet_session.close()

    def disconnect_from_bench_network(self):
        """
        Disable network interface used to connect to the bench network
        """
        if not self._bench_network_if:
            msg = "BenchNetInterface is not configured in BenchConfig for the COMPUTER1 Equipment"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._logger.info("Disconnect from bench network (disable '%s')" % self._bench_network_if)
        cmd = "ifconfig %s down" % self._bench_network_if
        self.ssh_exec("localhost", "root", cmd)
        time.sleep(2)

    def connect_from_bench_network(self):
        """
        Enable network interface used to connect to the bench network
        """
        if not self._bench_network_if:
            msg = "BenchNetInterface is not configured in BenchConfig for the COMPUTER1 Equipment"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._logger.info("Connect to bench network (enable '%s')" % self._bench_network_if)
        cmd = "ifconfig %s up" % self._bench_network_if
        self.ssh_exec("localhost", "root", cmd)
        time.sleep(2)

    def get_interface_from_list(self, if_name):
        """
        Parse Window command: "route print" return,
        and return searched Interface
        :type if_name: String
        :param if_name: name of the interface to search

        :rtype: dict
        :return: if_list is a of dict(if_number,mac_address)
        """
        # Check OS platform
        os_str = platform.system().upper()
        if os_str == 'LINUX':
            # Linux:
            msg = "get_interface_list: Not implemented for Linux platform"
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
            # Prepare command
        args = shlex.split("route print")
        try:
            p, q = run_local_command(args, False)
            data = p.communicate()
            # Use code page 860 to convert read bytes from windows console,
            # then normalize chars and convert them to utf-8 (ignore unknown symbols)
            strdata = unicodedata.normalize('NFKD', data[0].decode('cp860')).encode('ascii', 'ignore')
            # Parse following str data to extract IF numbers and its MAC addresses (ex: out_tupple=(13,9c-8e-99-dd-d5-97))
            # "===========================================================================\n"
            # "Interface List\n"
            # " 13...9c 8e 99 dd d5 97 ......Intel(R) 82579LM Gigabit Network Connection\n"
            if_strdata = [elem.strip(" ") for elem in
                          strdata.split("===========================================================================")[1].split("\n")
                          if match("^[0-9]", elem.strip(" "))]
            if_list = {}
            for line in if_strdata:
                data = [str(elem.strip(" ")) for elem in line.split("...")][:2]
                # Stop parsing when get " 1..........."
                if data[1] != "":
                    if if_name in line:
                        if_list[data[0]] = data[1].replace(" ", "-").upper()
                else:
                    break
                    # Debug
            msg = "Interface found: %s" % str(if_list)
            self.get_logger().debug(msg)
            return if_list
        except Exception as error:
            msg = "get_interface_from_list: %s" % str(error)
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

    def get_net_info_list(self):
        """
        Parse Window command: "ipconfig /all" return,
        and return a list of Network connection informations
        :rtype: list
        :return: net_connection_list is a list of dict(network_attribute, values)
        """
        # Check OS platform
        os_str = platform.system().upper()
        if os_str == 'LINUX':
            # Linux:
            msg = "get_net_info_list: Not implemented for Linux platform"
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        # Prepare command
        args = shlex.split("ipconfig /all")
        try:
            p, q = run_local_command(args, False)
            output = ""
            # Poll command q during 5 seconds to get all command std_out
            if q is not None:
                start = time.time()
                while time.time() < start + 5:
                    try:
                        line = q.get(timeout=5)
                    except Empty:
                        break
                    output += line
            data = p.communicate()
            # Use code page 860 to convert read bytes from windows console,
            # then normalize chars and convert them to utf-8 (ignore unknown symbols)
            strdata = unicodedata.normalize('NFKD', data[0].decode('cp860')).encode('ascii', 'ignore')
            strdata += output
            mac_strdata = [str(elem.strip(" ")) for elem in strdata.split("\n")]
            # Init variables
            net_connection_list = []
            net_connection = {}
            # Process each line
            for i in mac_strdata:
                # Empty line
                if i == "":
                    pass
                # New network connection (""blabla:") or new connection param ("blabla :")
                elif ":" in i:
                    entry = i.split(":")
                    if entry[1] == "":
                        # New connection
                        if entry[0][-1] != " ":
                            net_connection_list.append(net_connection)
                            net_connection = {}
                        # New "empty" param in current network connection
                        else:
                            net_connection[entry[0].split(".")[0].strip()] = entry[1].strip()
                    # New param in current network connection
                    else:
                        net_connection[entry[0].split(".")[0].strip()] = entry[1].strip()
            # Save last Net connection entries
            net_connection_list.append(net_connection)
            # Return the list of connection info
            return net_connection_list
        except Exception as error:
            msg = "get_net_info_list: %s" % str(error)
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

    def search_for_interface(self, win_network_timeout=150):
        """
        Parse Window Interface list (IF reference, MAC Address), and
        detect interface IF from a reference set of Interface, then
        wait for this interface to get a valid IP adress, and returns its
        IF number, IP address and Gateway address.
        :type win_network_timeout: int
        :param win_network_timeout: Timeout to process all the actions
                                    (default value: 120 s)
        :rtype: str list
        :return: if_number, if_ip_addr, if_gateway_addr
        """
        # Init variable
        if_number = None
        if_mac_addr = None
        if_ip_addr = None
        ip_addr_found = False
        if_gateway_addr = None
        ip_is_valid = False
        dns_server = ""
        if_list = {}
        if_name = self._tethering_interface

        start_time = time.time()
        end_time = start_time + win_network_timeout
        while time.time() <= end_time:
            # Step 1:
            # - search USB tethering interface (use "route print")
            # - get its ref (IF) and MAC address
            # Get new IF list
            if_list = self.get_interface_from_list(if_name)
            interface = set(if_list.keys())
            if interface:
                # Define USB tethering IF number and MAC address
                if_number = list(interface)[0]
                if_mac_addr = if_list[list(interface)[0]]
                # Debug log
                msg = "%s Interface detected (in %s s): IF=%s MAC=%s" \
                      % (if_name, time.time() - start_time, if_number, if_mac_addr)
                self.get_logger().debug(msg)
                # Init start time for the next debug log
                start_time = time.time()
            else:
                # Loop if no new interface detected
                time.sleep(1)
                continue
            # Step 2:
            # - Found MAC address in Network connection info (use: "ipconfig /all")
            # - Match this MAC address with a valid IPv4 address
            # Get Network connection info
            Net_list = self.get_net_info_list()
            self.get_logger().debug("Net list: %s" % Net_list)
            # Search the IF in connection info
            n = 0
            for i in Net_list:
                n += 1
                # Match MAC address with one connection
                if "Physical Address" in i.keys() and "DNS Servers" in i and i["Physical Address"] == if_mac_addr:
                    # Get "DNS Servers" from this connection
                    dns_server = i["DNS Servers"]
                    # Get "IPv4 Address" from this connection
                    IPv4Key = [key for key in i.keys() if "IPv4 Address" in key]
                    self.get_logger().debug("IPv4 adress: %s" % IPv4Key)
                    if len(IPv4Key) == 1:
                        # Process IPv4 IP Address "Default Gateway"
                        if_ip_addr = str(i[IPv4Key[0]].split("(")[0])
                        # Process DNS IP domain
                        dns_ip_domain = ".".join(dns_server.split(".")[:-1])
                        # Check if IP Address is in DNS IP domain
                        ip_is_valid = dns_ip_domain != "" and dns_ip_domain in if_ip_addr
                        # Debug log
                        msg = "*** Found IF=%s with MAC=%s in Connection %d " \
                              % (if_number, if_mac_addr, n)
                        # Check if IP address is valid, by comparison with DNS IP address
                        if ip_is_valid:
                            # Debug log
                            msg += "with valid IP=%s " % if_ip_addr
                            # Get "Default Gateway" address
                            gateway_key = [key for key in i.keys() if "Default Gateway" in key]
                            if len(gateway_key) == 1:
                                # "Default Gateway"
                                if_gateway_addr = str(i[gateway_key[0]])
                                msg += "and Gateway=%s (in %s s)***" \
                                       % (if_gateway_addr, time.time() - start_time)
                                self.get_logger().debug(msg)
                                ip_addr_found = True
                                break
                            else:
                                # Debug log
                                msg += "not a valid IP=%s*** wait for a valid IP address" % if_ip_addr
                                self.get_logger().debug(msg)
                        # No IPv4 IP address found yet
                        else:
                            if_ip_addr = None
                        break
            if ip_addr_found:
                break
            else:
                time.sleep(1)
        return if_number, if_ip_addr, if_gateway_addr

    def add_route(self, net_ip_addr, ip_mask, gateway_ip_addr, if_number):
        """
        Add an IP route.
        :type net_ip_addr: str
        :param net_ip_addr: IP address of the route
        :type ip_mask: str
        :param ip_mask: IP Mask of the route (ex: "255.255.255.0")
        :type gateway_ip_addr: str
        :param gateway_ip_addr: IP address of the Gateway
        :type if_number: str
        :param if_number: Network Interface number of the Gateway (ex: "17")
        :rtype: str
        :return: The command return
        """
        # Check OS platform
        os_str = platform.system().upper()
        if os_str == 'LINUX':
            # Linux:
            msg = "get_net_info_list: Not implemented for Linux platform"
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        # Prepare command
        args = shlex.split("route ADD %s MASK %s %s IF %s"
                           % (net_ip_addr, ip_mask, gateway_ip_addr, if_number))
        try:
            # Debug log
            msg = "route ADD %s MASK %s %s IF %s" \
                  % (net_ip_addr, ip_mask, gateway_ip_addr, if_number)
            self.get_logger().debug(msg)
            p, q = run_local_command(args, False)
            data = p.communicate()
            # Use code page 860 to convert read bytes from windows console,
            # then normalize chars and convert them to utf-8 (ignore unknown symbols)
            strdata = unicodedata.normalize('NFKD', data[0].decode('cp860')).encode('ascii', 'ignore')
            return strdata
        except Exception as error:
            msg = "add_route: %s" % str(error)
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

    def delete_route(self, net_ip_addr):
        """
        Delete an IP route
        :type net_ip_addr: str
        :param net_ip_addr: IP address of the route
        :rtype: str
        :return: The command return
        """
        # Check OS platform
        os_str = platform.system().upper()
        if os_str == 'LINUX':
            # Linux:
            msg = "get_net_info_list: Not implemented for Linux platform"
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        # Prepare command
        args = shlex.split("route DELETE %s" % net_ip_addr)
        try:
            # Debug log
            msg = "route DELETE %s" % net_ip_addr
            self.get_logger().debug(msg)
            p, q = run_local_command(args, False)
            data = p.communicate()
            # Use code page 860 to convert read bytes from windows console,
            # then normalize chars and convert them to utf-8 (ignore unknown symbols)
            strdata = unicodedata.normalize('NFKD', data[0].decode('cp860')).encode('ascii', 'ignore')
            return strdata
        except Exception as error:
            msg = "delete_route: %s" % str(error)
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

    def get_ip_mask(self, ip_address):
        """
        Create a mask from an IP address, by replacing last digit with "0"
        :type ip_address: str
        :param ip_address: IP address to use to create the mask
        :rtype: str
        :return: The IP mask
        """
        ip_mask = ip_address.split(".")
        ip_mask[-1] = "0"
        ip_mask = ".".join(ip_mask)
        return ip_mask

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
        filename = os.path.normpath(filename).replace("\\", "/")
        cwd = None
        fullfilename = ""
        ftp = None

        try:
            ftp = FTP(server_ip_address, username, password, timeout=timeout)

            # Enter in local passive mode
            ftp.set_pasv(True)

            if direction == XFER_DIRECTIONS.UL:  # pylint: disable=E1101
                # Remove the potential path included in the filename
                filename = os.path.basename(filename)
                # Create the local temporary filename
                fullfilename = self._create_tmp_ftp_uplink_file(filename)

                # Open the FTP connection
                filename_fd = open(fullfilename, 'rb')
                ftp.storbinary('STOR ' + filename, filename_fd)
                filename_fd.close()

                # Control that the file exists on the server
                self._ftp_does_remote_file_exist(ftp, filename)

                # Remove the file on the server
                ftp.delete(filename)

            elif direction == XFER_DIRECTIONS.DL:  # pylint: disable=E1101
                # Extract the ftp path and ftp file
                folder = os.path.dirname(filename)
                filename = os.path.basename(filename)

                # Change the remote working directory
                if folder:
                    ftp.cwd(folder)

                # Check the existance of the remote file:
                self._ftp_does_remote_file_exist(ftp, filename)

                # Change the working directory into the temp folder
                cwd = os.getcwd()
                os.chdir(gettempdir())

                # Remove the file is already exists
                if os.path.isfile(filename):
                    os.unlink(filename)

                # Get the file
                ftp.retrbinary('RETR ' + filename, open(filename, 'wb').write)

                # Check the existance of the DL file
                if not os.path.isfile(filename):
                    msg = "FTP DL fails"
                    self._logger.error(msg)
                    # This should generate a test FAIL!
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

                # FTP Download success. Remove the downloaded file
                os.unlink(filename)

            else:
                msg = "ftp_xfer: invalide direction parameter"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        finally:
            # Come back into the ACS working directory
            if cwd:
                os.chdir(cwd)

            # Remove TMP file used for Uplink test
            if direction == XFER_DIRECTIONS.UL and os.path.isfile(fullfilename):  # pylint: disable=E1101
                os.unlink(fullfilename)

            # Close the FTP Connection
            if ftp:
                ftp.quit()

    def iperf_client(self, settings):
        """
        Execute iperf client command on host

        :type settings: dictionary
        :param settings: settings to launch iperf as server

        :rtype: str
        :return: Command result
        """
        GenericComputer.iperf_client(self, settings)
        # Launch IPERF client

        self.run_cmd(self._iperf_cmd, -1)

    def start_iperf_server(self, settings):
        """
        Execute iperf server command on host

        :type settings: dictionary
        :param settings: Settings to launch iperf as server
        """
        GenericComputer.start_iperf_server(self, settings)

        # Build the iperf command
        args = self._build_iperf_command(settings, "server")

        # Launch IPERF Server
        self.run_cmd(args, -1)

    def _create_tmp_ftp_uplink_file(self, filename):
        """
        Create a temporary file. The file size of the new temp file created should be written in the filename

        :type filename: str
        :param filename: the name of the temp file to create
        """
        # Extract the file size from the filename
        filesize_extract = re.search(r'([0-9]+)[^okMG]*([okMG])', filename, re.IGNORECASE)
        if filesize_extract is None:
            msg = "Upload file name is not correct, please set 'xxxMo' or 'xxxGo' as UL_FILE TC parameter"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        count = int(filesize_extract.group(1))
        if filesize_extract.group(2).lower() == "o":
            dd_bs = 1
        elif filesize_extract.group(2).lower() == "k":
            dd_bs = 1024
        elif filesize_extract.group(2).upper() == "M":
            dd_bs = 1048576
        elif filesize_extract.group(2).upper() == "G":
            dd_bs = 1073741824

        # Create the temporary file.
        tmp_dir = gettempdir()
        tmp_file = filename.replace(' ', '_')
        tmp_file = os.path.join(tmp_dir, tmp_file)
        create_file_cmd = "dd if=/dev/zero of=%s bs=%d count=%d" % (tmp_file, dd_bs, count)
        self.run_cmd(create_file_cmd)

        # Control that the tmp file has well been created and has the expected size
        if not os.path.isfile(tmp_file) or dd_bs * count != os.path.getsize(tmp_file):
            if os.path.isfile(tmp_file):
                msg = "Wrong tmp file size created. "
                msg += "Expected size: %d - Tmp filesize: %d" % (dd_bs * count, os.path.getsize(tmp_file))
                # Remove the file
                os.unlink(tmp_file)
            else:
                msg = "Tmp file creation failed."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        return tmp_file

    def _ftp_does_remote_file_exist(self, ftp, filename):
        """
        Check the existance of the file on the remote FTP server

        :type ftp: FTP object
        :param ftp: the ftp object to use to list the remote files list
        :type filename: str
        :param filename: the filename to control the existance
        """
        output = list()
        found = False
        ftp.retrlines("LIST", output.append)
        for line in output:
            self._logger.debug("FTP output: " + line)
            listed_file = line.split()[-1:]
            if filename in listed_file:
                found = True
                break
        if not found:
            msg = "File to DL is missing on the FTP server (%s)" % filename
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def mount_host_partition(self, partition, mount_point):
        """
        Mount a partition on the host computer. This can apply to external media storage
        (plugged on USB for example)

        :type partition: str
        :param partition: path of the device to mount , eg : /dev/sdb1
        :type mount_point: str
        :param mount_point: folder where to mount the partition , eg : /media/storage
        """
        # Check OS platform
        os_str = platform.system().upper()

        if os_str != 'LINUX':
            msg = "mount_host_partition : Not implemented for OS %s" % os_str
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        else:
            # Linux OS
            # Check if mount point exists
            if not os.path.isfile(mount_point) and mount_point != "":
                msg = "Mount point %s does not exist, create it before mounting ..." % mount_point
                self._logger.warning(msg)
                cmd = "mkdir -p %s" % mount_point
                self.run_cmd(cmd)

            # Now mount partition
            cmd = "mount %s %s" % (partition, mount_point)
            output = self.run_cmd(cmd)
            std_output = output["std"]
            err_output = output["err"]
            return_code = output["ret"]

            if return_code != 0:
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, err_output)

        output = "%s %s" % (std_output, err_output) if (
            err_output and err_output) else (err_output or err_output)

        return output

    def unmount_host_partition(self, mount_point):
        """
        Unmount a partition on the host computer. This can apply to external media storage
        (plugged on USB for example)

        :type mount_point: str
        :param mount_point: mount point of the partition , eg : /media/storage
        """
        # Check OS platform
        os_str = platform.system().upper()

        if os_str != 'LINUX':
            msg = "unmount_host_partition : Not implemented for OS %s" % os_str
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        else:
            # Linux OS
            # Now unmount partition
            self._logger.debug("Unmount partition {0}".format(mount_point))
            cmd = "umount {0}".format(mount_point)
            output = self.run_cmd(cmd)
            std_output = output["std"]
            err_output = output["err"]
            return_code = output["ret"]

            if return_code != 0:
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, err_output)
            else:
                # Remove the mount point but don't raise exception (sudo needed by exemple)
                self._logger.debug("Remove mount point {0}".format(mount_point))
                cmd = "rm -rf {0}".format(mount_point)
                self.run_cmd(cmd)

        output = "%s %s" % (std_output, err_output) if (
            err_output and err_output) else (err_output or err_output)

        return output

    def delete_host_file(self, file_name):
        """
        Delete a file on the host computer. This can apply to external media storage
        (plugged on USB for example)

        :type file_name: str
        :param file_name: file name on host, eg : /media/storage/file.bin
        """
        # Check OS platform
        os_str = platform.system().upper()

        if os_str != 'LINUX':
            msg = "delete file on host : Not implemented for OS %s" % os_str
            raise AcsBaseException(AcsBaseException.FEATURE_NOT_IMPLEMENTED, msg)
        else:
            # Now remove file
            cmd = "rm -rf %s" % str(file_name)
            output = self.run_cmd(cmd)
            std_output = output["std"]
            err_output = output["err"]

            if err_output != "" and std_output == "":
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, err_output)

        output = "%s %s" % (std_output, err_output) if (
            err_output and err_output) else (err_output or err_output)

        return output


# Unit test for ftp_xfer function
if __name__ == "__main__":
    equipment_dict = {u'LOCAL_COMPUTER': {}, u'REMOTE_COMPUTER': {}}

    bench_config = BenchConfigParameters(equipment_dict)

    computer = LocalComputer("COMPUTER1", "LOCAL_COMPUTER", bench_config)

    computer.ftp_xfer("UL", "192.168.0.150", "acs", "intelacs", "put100M", 20)
    computer.ftp_xfer("DL", "192.168.0.150", "acs", "intelacs", "acs_test/get100M", 20)


