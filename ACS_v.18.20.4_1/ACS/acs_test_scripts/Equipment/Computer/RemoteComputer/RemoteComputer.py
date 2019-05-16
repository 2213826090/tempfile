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
:summary: Class to provide utilities to run from a Remote Computer connected
to the IP network of the bench
:since:24/04/2012
:author: apairex BZ2832
"""

import subprocess
import sys
import platform
import re
import time

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.Computer.Common.Common import GenericComputer
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure

import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil

# Non blocking readline
from Queue import Queue, Empty
ON_POSIX = 'posix' in sys.builtin_module_names
from threading import Thread
from threading import Event
import paramiko
import socket
import random


class RemoteComputer(GenericComputer):

    """
    Common class to provide utilities
    to run from a remote Computer connected to the IP network of the bench
    """

    def __init__(self, name, model, eqt_params):
        """
        Constructor
        """
        GenericComputer.__init__(self, name, model, eqt_params)
        self._eqt_params = eqt_params
        self._host = str(self._eqt_params.get_param_value("IP"))
        self._host_on_test_network = str(self._eqt_params.get_param_value("IP_TEST", self._host))
        if self._host_on_test_network == "":
            self._host_on_test_network = self._host
        self._wifi_interface = str(self._eqt_params.get_param_value("Interface"))
        self._local_os = platform.system()
        self._ipv6prefix = str(self._eqt_params.get_param_value("IPv6Prefix", ""))
        if not self._eqt_params.has_parameter("SshPort"):
            self._port = 22
        else:
            self._port = int(self._eqt_params.get_param_value("SshPort", "22"))
        self._std_in = None
        self._std_out = None
        self._std_err = None

        self._queue = None
        self._event_cmd_ended = None
        self._client = None
        self._iperf_uid = ""

        self._os = self.LINUX

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
        # To force the initialization, run a 'release' before the initialization
        if force_init:
            self.get_logger().debug("Force release connection")
            self.release()

        if self._client is None:
            self.get_logger().info("ssh_connect to %s@%s password: %s" % (self._login, self._host, self._password))
            try:
                self._client = paramiko.SSHClient()
                self._client.load_system_host_keys()
                self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                self._client.connect(hostname=self._host,
                                     port=self._port,
                                     username=self._login,
                                     password=self._password,
                                     key_filename=self._key)
            except Exception as e:
                output = "An error occurred during ssh connection with Remote computer (%s)" % e
                self._logger.error(output)
                if self._client is not None:
                    self._client.close()
                    self._client = None
                    self._queue = None
                    self._event_cmd_ended = None
                raise
            return True

        # Connection was already active
        return False

    def release(self):
        """
        Release the Computer software equipment
        For Remote Computer, Disconnects the ssh connection
        For Local Computer, Terminate asynchronous process
        """
        self.get_logger().info("SSH release connection")
        if self._queue is None and self._client is None:
            self._logger.debug("Already released")
            return

        try:
            if self._std_in is not None:
                self._std_in.close()
            if self._client is not None:
                self._client.close()  # pylint: disable=E1101
            if self._event_cmd_ended is not None:
                self._event_cmd_ended.clear()
        except:
            self._logger.warning("An exception occurs when trying to release SSH connection")
        finally:
            self._queue = None
            self._client = None
            self._event_cmd_ended = None

    def run_cmd(self, cmd, timeout=1):
        """
        Execute a command on the computer (Local or Remote)

        :type cmd: str
        :param cmd: command to execute on the remote host

        :type timeout: int
        :param timeout: timeout to wait for stdout (in sec.)
                        0  -> stdout is not read
                        -1 -> On LOCAL Computer the cmd is launched asynchronously
                        (Only 1 cmd can be launched asynchronously in the same time)

        :rtype: String, String
        :return: stdout and stderr data reply from command
        """
        self._logger.debug("run cmd: %s" % cmd)
        # Launch command on SSH client
        self._std_in, self._std_out, self._std_err = self._client.exec_command(cmd)

        if self._queue is None:
            # Enqueue all command output in self._queue using a daemon thread
            self._queue = Queue()
            self._event_cmd_ended = Event()

        # pylint: disable=E1101
        thread = Thread(target=enqueue_output,
                        args=(self._std_out, self._queue,
                              self._logger, self._event_cmd_ended))
        thread.daemon = True  # thread dies with the program
        thread.start()

        out = ''
        if timeout > 0:
            out = self.read(timeout)

        return {"std": out, "err": ""}

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
        self.get_logger().debug("copy_file_in_local_path from %s@%s:%s -> %s"
                                % (self._login, self._host, source, destination))

        try:
            sftp = self._client.open_sftp()
            sftp.get(source, destination)
            sftp.close()
        except Exception as e:
            output = "An error occurred during ssh connection with Remote computer (%s)" % e
            self._logger.error(output)
            if self._client is not None:
                self._client.close()
                self._client = None
                self._queue = None
            raise
        return True

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
        self.get_logger().debug("copy_file_in_remote_path from %s -> %s@%s:%s" % (source, self._login, self._host, destination))
        try:
            sftp = self._client.open_sftp()
            sftp.put(source, destination)
            sftp.close()
        except Exception as e:
            output = "An error occurred during ssh connection with Remote computer (%s)" % e
            self._logger.error(output)
            if self._client is not None:
                self._client.close()
                self._client = None
                self._queue = None
            raise
        return True

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
        :param flood_mode: True if you want to use the ping in flood mode

        :rtype: Measure object
        :return: Packet loss in %
        """
        # Compute whether ip_address is V4 or V6 format.
        if NetworkingUtil.is_valid_ipv4_address(str(ip_address)):
            linux_cmd = "ping"
        elif NetworkingUtil.is_valid_ipv6_address(str(ip_address)):
            linux_cmd = "ping6"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "%s is not a valid IP address" % str(ip_address))

        if not flood_mode:
            flood_option = ""
            flood_txt = ""
        else:
            flood_option = " -f"
            flood_txt = "(flood mode) "

        cmd = "%s -s " % linux_cmd + str(packet_size) + flood_option + \
            " -c " + str(packet_count) + " -i " + str(interval) + \
            " " + str(ip_address)

        self._logger.info("Ping address " + flood_txt + str(ip_address) +
                          " with " + str(packet_count) + " packets of " +
                          str(packet_size) + " bytes...")
        output = self.ssh_exec(self._host, self._login, cmd)

        output_lower = output.lower()
        packet_loss = Measure()

        # Search the loss rate
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

    def get_ipv6_addresses(self):
        """
        Returns the ipv6 address computer.

        :rtype: array of strings
        :return: List of interface's IP addresses
        """
        cmd = "ip -6 addr show"
        tries = 5
        self._logger.debug("Getting IPv6 addresses: ")
        while tries > 0:
            output = self.ssh_exec(self._host, self._login, cmd)

            interface_ips = re.findall(r"\s*INET6\s*(%s[^/]+)/.*" %
                                       self._ipv6prefix.upper(), output.upper())
            if not interface_ips:
                tries -= 1
                if tries == 0:
                    msg = "Could not get IPv6 address"
                    self._logger.error(msg)
                    self._logger.debug("output is: %s" % output)
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
                else:
                    time.sleep(2)
            else:
                self._logger.debug("Got IPs: ")
                self._logger.debug(interface_ips)
                break
        return interface_ips

    def start_stop_service(self, service_name, mode):
        """
        Launches or stops a service on the computer.

        :type service_name: String
        :param service_name: The name of the service you want to start or stop.

        :type mode: String
        :param mode: start or stop
        """
        mode = mode.lower()
        if mode not in ["start", "stop"]:
            msg = "%s is not a valid mode" % mode
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        cmd = "service %s %s" % (service_name, mode)

        self._logger.info("%s %s server" % (mode.title(), service_name))
        self._logger.debug(cmd)

        # Open ssh connection
        local_connection = self.init()

        # Launch the request through ssh
        self.run_cmd(cmd, 5)

        if local_connection:
            self.release()

    def get_ipv6_prefix(self):
        """
        Returns the IPV6 network prefix.

        :rtype: String
        :return: The IPV6 network prefix.
        """
        return self._ipv6prefix

    def get_host_on_test_network(self):
        """
        return configured host from Bench_Config
        This host name or IP is the one accessible from DUT point of view.
        This is only used on Conformance Bench,
        where the Test network is different from the Control Network

        :rtype: str
        :return: host name or IP, from DUT point of view (Test Network)
        """
        return self._host_on_test_network

    def get_interface_mac_addr(self, interface):
        """
        Get the MAC address of the interface specified as parameter

        :type interface: str
        :param interface: wifi interface to get MAC address from (wlan1/ra0)

        :rtype: String
        :return: MAC address of the interface given as parameter,
                 characters are in lower case for hexadecimal values
        """
        output = self.ssh_exec(self._host, self._login, "ifconfig %s" % interface)

        re_macaddress = re.search(r"HWaddr\s((?:[0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2})", output)

        if re_macaddress is None:
            msg = "Unable to parse MAC address from output command. Ouput=" + str(output)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        macaddress = re_macaddress.group(1).lower()
        # '-' is used if the WiFi interface is in Monitor mode
        macaddress = macaddress.replace('-', ':')

        return macaddress

    def clean_tcp_records_computer(self):
        """
        Clean TCP records on computer. Extracted from set_prop_local.sh script (PnPLab repository, WiFi KPIs).
        """
        # Enable Rx Buffer size auto tuning
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_moderate_rcvbuf=1")
        # increase TCP max buffer size settable using setsockopt()
        self.ssh_exec(self._host, self._login, "sysctl -w net.core.rmem_max=16777216")
        self.ssh_exec(self._host, self._login, "sysctl -w net.core.wmem_max=16777216")
        # Values below are in Bytes, not in pages
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_rmem='4096 87380 16777216'")
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_wmem='4096 87380 16777216'")
        # increase the length of the processor input queue
        self.ssh_exec(self._host, self._login, "sysctl -w net.core.netdev_max_backlog=30000")
        # recommended default congestion control is cubic
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_congestion_control=reno")
        # tcp_abc -> allow increase cwd by 2
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_abc=2")
        # recommended for hosts with jumbo frames enabled
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_mtu_probing=1")
        # Slow Start after idle is disabled
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_slow_start_after_idle=0")
        # No metrics saved
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_no_metrics_save=1")
        # Enable F-RTO
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_frto=2")
        # TCP keep alive
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_keepalive_time=7200")
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_keepalive_intvl=75")
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_keepalive_probes=9")
        # TCP timestamps
        # Enabled for better RTT estimate
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_timestamps=1")
        # TCP window scaling
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_window_scaling=1")
        # TCP selective ack
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_sack=1")
        # TCP forward ack
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_fack=0")
        # TCP duplicate selective ack
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_dsack=1")
        # TCP adv win scale
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.tcp_adv_win_scale=1")
        # Flush TCP param cache
        self.ssh_exec(self._host, self._login, "sysctl -w net.ipv4.route.flush=1")
        # MTU size
        self.ssh_exec(self._host, self._login, "ifconfig eth0 mtu 1500")
        # self.run_cmd("ethtool -K eth0 sg on tso off gso off")

    def iperf_client(self, settings):
        """
        Execute iperf client command on host

        :type settings: dictionary
        :param settings: settings to launch iperf as server

        :rtype: str
        :return: Command result
        """
        GenericComputer.iperf_client(self, settings)
        try:
            output = self.ssh_exec(self._host, self._login, self._iperf_cmd)
        except:
            # Disconnection
            self._logger.error("iperf client disconnected: \n" + output)
            raise
        self._logger.debug("iperf client output:\n" + output)

        return output

    def start_iperf_server(self, settings):
        """
        Execute iperf server command on host

        :type settings: dictionary
        :param settings: Settings to launch iperf as server
        """
        self._logger.info("Start iperf server on %s" % type(self).__name__)

        # Build the iperf command
        self._build_iperf_command(settings, "server")

        # Open ssh connection
        self.init()

        # in order to kill it at the end of test
        # This UID is added to iperf server command through the -t parameter
        # which is not used by iperf server
        # The iperf command will look something like that:
        # iperf -u -p 4821 -s -t 1423411334
        # with 1423411334 as iperf UID
        self._iperf_uid = str(random.getrandbits(32))
        self._iperf_cmd += " -t %s" % self._iperf_uid

        return self.run_cmd(self._iperf_cmd, -1)

    def stop_iperf_server(self):
        """
        Terminate iperf server command on host
        @rtype: string
        @return: Command result
        """
        self._logger.info("Stop iperf server")

        # Terminate IPERF server
        serveroutput = self.read(2)
        self.release()
        # Kill iperf server on computer
        # First retrieve Process ID of server thanks to iperf UID stored at server start
        # ps ax | grep iperf | grep "t 1423411334" | grep -v grep return for instance:
        # 20584 ?        Ssl    0:00 iperf -u -p 4821 -s -t 1423411334
        # We can see the iperf command with iperf UID (1423411334) and the first digits (20584) are the iperf PID
        # awk '{print $1}' allows to retrieve directly the PID
        args = "ps ax | grep iperf | grep \"t %s\" | grep -v grep | awk '{print $1}'" % self._iperf_uid
        pid = self.ssh_exec(self._host, self._login, args).rstrip("\n")
        pid_list = pid.split("\n")
        # If several process are found with the same iperf uid (nearly impossible with a probability of 1/2^32)
        # raise exception
        if len(pid_list) > 1:
            msg = "Can't kill iperf server on computer because there are too many iperf server PIDs (pids: %s)" % pid_list
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        # Kill iperf server on computer
        self._logger.info("Kill iperf server on computer (pid: %s)" % pid)
        args = "kill -9 %s" % pid
        out = self.ssh_exec(self._host, self._login, args)
        self._logger.debug("iperf server killed: %s" % out)

        return serveroutput


def enqueue_output(out, queue, logger, event):
    """
    Write SSH out stream into a queue and print it on the fly

    :type out: output stream
    :param out: SSH out stream to write in queue

    :type queue: Queue
    :param queue: the queue that need to be filled

    :type logger: logger
    :param logger: logger instance

    """
    for line in out:
        queue.put(''.join([i if ord(i) < 128 else ' ' for i in line]))

    event.set()
    out.close()
