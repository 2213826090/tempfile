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
:summary: Common class to provide utilities to run from a Computer connected
          to the IP network of the bench
:since:24/04/2012
:author: apairex BZ2832
"""

import re
import time
import subprocess
import sys
import os

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.Computer.Interface.IComputer import IComputer
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Queue import Empty
from acs_test_scripts.Utilities.IPerfUtilities import Iperf
import paramiko
import socket
from UtilitiesFWK.Utilities import str_to_bool_ex
ON_POSIX = 'posix' in sys.builtin_module_names


class GenericComputer(EquipmentBase, IComputer, Iperf):

    """
    Common class to provide utilities
    to run from a Computer connected to the IP network of the bench
    """
    WINDOWS = "Windows"
    LINUX = "Linux"

    def __init__(self, name, model, eqt_params):
        """
        Constructor
        """
        # Initialize class parent
        IComputer.__init__(self)
        Iperf.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)

        self._os = None

        # WiFi interface name
        self._wifi_interface = None

        self._usb_interface = None

        self._tethering_interface = "RNDIS"

        self._queue = None
        self._event_cmd_ended = None
        self._host = "localhost"
        self._login = eqt_params.get_param_value("username", "root")
        self._password = eqt_params.get_param_value("password", "")
        self._key = eqt_params.get_param_value("ssh_key", "")
        if self._key == "":
            self._key = None

        # Check ssh and scp full path name
        self._ssh_bin = str(eqt_params.get_param_value("SshPath", ""))
        self._scp_bin = str(eqt_params.get_param_value("ScpPath", ""))
        self._ssh_bin = self.__join_exe_if_missing(self._ssh_bin, "ssh")
        self._scp_bin = self.__join_exe_if_missing(self._scp_bin, "scp")

        self._ssh_checked = False
        self._scp_checked = False

    def get_wifi_interface(self):
        """
        return configured wifi interface from Bench_Config

        :rtype: str
        :return: Wifi interface name
        """
        return self._wifi_interface

    def get_usb_interface(self):
        """
        return configured usb interface from Bench_Config

        :rtype: str
        :return: usb interface name
        """
        return self._usb_interface

    def get_tethering_interface(self):
        """
        return configured tethering interface from Bench_Config

        :rtype: str
        :return: tethering interface name
        """
        return self._tethering_interface

    def start_iperf_server(self, settings):
        """
        Execute iperf server command on host

        :type settings: dictionary
        :param settings: Settings to launch iperf as server
        """
        self._logger.info("Start iperf server on %s" % type(self).__name__)

    def stop_iperf_server(self):
        """
        Terminate iperf server command on host
        @rtype: string
        @return: Command result
        """
        self._logger.info("Stop iperf server")

    def start_iperf_client(self, settings):
        self._logger.info("Start iperf client on %s" % type(self).__name__)
        return self.iperf_client(settings)

    def iperf_client(self, settings):
        """
        Execute iperf client command on host

        :type settings: dictionary
        :param settings: settings to launch iperf as server

        :rtype: str
        :return: Command result
        """
        # Retrieve iperf settings
        protocol = settings.get('protocol', 'tcp')
        port_number = settings.get('port_number')
        duration = settings.get('duration', '10')
        direction = settings.get('direction', 'both')
        msg = "IPERF %s " % protocol.upper()
        if direction == 'both':
            msg += "UL & DL"
        else:
            msg += direction.upper()
        msg += " measurement on port %s for %s seconds" % (str(port_number), str(duration))
        self._logger.info(msg)
        # Build the iperf command
        self._build_iperf_command(settings, "client")
        self._logger.debug("Run command : " + self._iperf_cmd)

    def stop_iperf_client(self):
        pass

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
        output = ""
        std_out = None
        self.get_logger().debug("ssh_exec %s@%s %s" % (login, host, cmd))
        # Open SSH session
        try:
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(hostname=host,
                           username=login,
                           password=self._password,
                           key_filename=self._key)
            # Perform command
            std_in, std_out, std_err = client.exec_command(cmd)
            # Get command output
            if std_out is not None:
                output += "".join(std_out.readlines())
                output += "".join(std_err.readlines())
        except socket.timeout as ex:
            msg = "Unable to establish ssh connection with computer (%s)" % ex
            self._logger.error(msg)
        except Exception as e:
            msg = "An error occurred during ssh connection with computer (%s)" % e
            self._logger.error(msg)
        finally:
            if client is not None:
                client.close()
        return output

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
        self.get_logger().info("wifi_connect to %s" % ssid)

        if standard == 'b':
            rate = "11M"
            mode = 1  # legacy 11B only
        else:
            rate = "54M"
            if standard in ['bg', 'gb']:
                mode = 0  # legacy 11b/g mixed
            elif standard == 'g':
                mode = 4  # legacy 11G only
            elif standard == 'a':
                mode = 2  # legacy 11A only
            elif standard == 'n':
                mode = 6  # 11N only
            elif standard == 'gn':
                mode = 7  # 11GN mixed
            elif standard in ['bgn', 'ngb']:
                mode = 9  # 11BGN mixed
            elif standard in ['an', 'na']:
                mode = 8  # 11AN mixed
            elif standard in ['abgn']:
                mode = 5  # 11ABGN mixed
            else:
                msg = "%s is not a valid HOTSPOT_STANDARD" % standard
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Configure Wifi USB Device
        # Check security type: WPA vs Open
        if security.startswith("WPA2-PSK"):
            if interface.startswith("ra"):
                # RaLink wifi adapter requires iwpriv
                cmds = [
                    ["ifconfig %s down" % interface, "", 1],
                    ["ifconfig %s up" % interface, "", 1],
                    ["iwpriv %s set NetworkType=Infra" % interface, "", 1],
                    ["iwpriv %s set WirelessMode=%d" % (interface, mode), "", 1],
                    ["iwpriv %s set AuthMode=WPA2PSK" % interface, "", 1],
                    ["iwpriv %s set EncrypType=AES" % interface, "", 1],
                    ["iwpriv %s set SSID=%s" % (interface, ssid), "", 1],
                    ["iwpriv %s set WPAPSK=%s" % (interface, passphrase), "", 1],
                    ["iwpriv %s set SSID=%s" % (interface, ssid), "", 1]
                ]
            else:
                # DLink wifi adapter requires wpa_cli
                cmds = [
                    ["rfkill unblock wifi", "", 1],
                    ["rfkill unblock all", "", 1],
                    ["ifconfig %s down" % interface, "", 1],
                    ["iwconfig %s rate %s" % (interface, rate), "", 1],
                    ["ifconfig %s up" % interface, "", 1],
                    # edit /usr/share/dbus-1/system-services/fi.epitest.hostap.WPASupplicant.service
                    # and comment all the lines to disable dbus access
                    ["killall wpa_supplicant", "", 1],
                    ["sleep 1", "", 1],
                    ["wpa_supplicant -Dnl80211 -c/etc/wpa_supplicant.conf -i%s -t -B" % interface, "", 1],
                    ["wpa_cli remove_network all", "OK", 2],
                    ["wpa_cli add_network", "0", 2],
                    ["wpa_cli set_network 0 ssid '\"%s\"'" % ssid, "OK", 2],
                    ["wpa_cli set_network 0 key_mgmt WPA-PSK", "OK", 2],
                    ["wpa_cli set_network 0 psk '\"%s\"'" % passphrase, "OK", 2],
                    ["wpa_cli enable_network 0", "OK", 2],
                    # cannot rely on CTRL-EVENT-CONNECTED not always provided by wpa_cli
                    # ["status 0", "CTRL-EVENT-CONNECTED", 20],
                    ["ifconfig %s up" % interface, "", 3]
                ]
        else:
            if interface.startswith("ra"):
                # RaLink wifi adapter requires iwpriv
                cmds = [
                    ["ifconfig %s down" % interface, "", 1],
                    ["ifconfig %s up" % interface, "", 1],
                    ["iwpriv %s set NetworkType=Infra" % interface, "", 1],
                    ["iwpriv %s set WirelessMode=%d" % (interface, mode), "", 1],
                    ["iwpriv %s set AuthMode=OPEN" % interface, "", 1],
                    ["iwpriv %s set EncrypType=NONE" % interface, "", 1],
                    ["iwpriv %s set SSID=%s" % (interface, ssid), "", 1]
                ]
            else:
                # DLink wifi adapter requires wpa_cli
                cmds = [
                    ["ifconfig %s down" % interface, "", 1],
                    ["iwconfig %s rate %s" % (interface, rate), "", 1],
                    ["ifconfig %s up" % interface, "", 1],
                    # edit /usr/share/dbus-1/system-services/fi.epitest.hostap.WPASupplicant.service
                    # and comment all the lines to disable dbus access
                    ["killall wpa_supplicant", "", 1],
                    ["sleep 1", "", 1],
                    ["wpa_supplicant -Dnl80211 -c/etc/wpa_supplicant.conf -i%s -t -B" % interface, "", 1],
                    ["wpa_cli remove_network all", "OK", 2],
                    ["wpa_cli add_network", "0", 2],
                    ["wpa_cli set_network 0 ssid '\"%s\"'" % ssid, "OK", 2],
                    ["wpa_cli set_network 0 key_mgmt NONE", "OK", 2],
                    ["wpa_cli enable_network 0", "OK", 2],
                    ["ifconfig %s up" % interface, "", 3]
                ]

        # Open ssh connection
        local_connection = self.init()

        for cmd in cmds:
            self.get_logger().debug("cmd: " + cmd[0])
            self.run_cmd(cmd[0], 0)
            wait = cmd[2]
            found = False
            while wait > 0 and not found:
                wait -= 1
                output = self.read(1)
                if len(output):
                    self.get_logger().debug("out: " + output)
                if len(cmd[1]) == 0:
                    found = True  # just read stdout once
                elif re.search(cmd[1], output) is not None:
                    found = True
            if not found:
                # error
                self.run_cmd("quit", 1)
                msg = "Could not associate device %s with DUT" % interface
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # request IP address from DHCP server
        self.run_cmd("killall dhclient", 5)
        self.run_cmd("dhclient %s\n" % interface, 5)

        # wait at most 30 seconds to get IP address
        wait = 30
        res = None
        while wait > 0 and res is None:
            wait -= 1
            time.sleep(1)
            # extract IP address
            output = self.run_cmd("ifconfig %s\n" % interface, 1)["std"]
            res = re.search(r"inet [^12]*([0-9\.]+) ", output)

        if local_connection:
            self.release()

        if res is None:
            msg = "Could not get IP address of device %s" % interface
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        interface_ip = res.group(1)
        return str(interface_ip)

    def wifi_disconnect(self, interface):
        """
        Disconnect a remote wifi interface

        :type interface: str
        :param interface: wifi interface to configure (wlan1/ra0)
        """
        self.get_logger().info("wifi_disconnect")

        # Open ssh connection
        local_connection = self.init()

        self.run_cmd("ifconfig %s down" % interface, 0)

        if local_connection:
            self.release()

    def read(self, timeout=1):
        """
        Reads the stdout. Can be used to flush the stdout.
        For local computer this function is used to retrieve stdout for asynchronous process.

        :type timeout: int
        :param timeout: timeout to wait for stdout in seconds

        :rtype: str
        :return: stdout data
        """
        output = ''

        if self._queue is None or self._event_cmd_ended is None:
            raise TestEquipmentException(TestEquipmentException.INSTANTIATION_ERROR,
                                         "No queue instance to read output")

        start = time.time()
        while not self._event_cmd_ended.is_set():
            try:
                line = self._queue.get(timeout=1)
                self._queue.task_done()
                output += line
            except Empty:
                # Lets timeout occurs to raise exception
                pass
            finally:
                if time.time() > start + timeout:
                    raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED,
                                                 "Timeout for RemoteComputer command")

        return output

    def get_os(self):
        """
        Returns the Operating System installed on the COMPUTER
        For REMOTE_COMPUTER it always returns LINUX

        :rtype: str
        :return: the OS installed on the computer
        """
        return self._os

    def check_command(self, binary_to_check):
        """
        Check that binary_to_check command is installed on the host computer.
        This function is used to check SSH and SCP command are available on the COMPUTER

        :type binary_to_check: str
        :param binary_to_check: Binary cmd to check if available on the Computer
        """
        if (binary_to_check == self._ssh_bin and not self._ssh_checked) \
                or (binary_to_check == self._scp_bin and not self._scp_checked):
            # Try to run the binary tool and check usage message
            process = subprocess.Popen(binary_to_check,
                                       shell=True,
                                       stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT,
                                       bufsize=1,
                                       close_fds=ON_POSIX)
            output = process.communicate()
            if "is not recognized" in output[0] or "command not found" in output[0] \
                    or "commande introuvable" in output[0]:
                msg = "%s is not available on computer" % binary_to_check
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)

            if binary_to_check == self._ssh_bin:
                self._ssh_checked = True
            elif binary_to_check == self._scp_bin:
                self._scp_checked = True

    def __join_exe_if_missing(self, path, exe):
        """
        Checks that the str 'path' ends with 'exe'.
        If not, add 'exe' at the end with a folder separation if required.
        If path is empty "", it returns 'exe'

        :type path: str
        :param path: filepath to check
        :type exe: str
        :param exe: the file to check the presence at the end of path
        :rtype: String
        :return: the full path for exe file
        """
        path = str(path)
        exe = str(exe)
        full_path = path
        if not path.endswith(exe) and not path.endswith(exe + ".exe"):
            # ssh full path is not ending with 'exe' or 'exe'.exe
            # then add the executable name "ssh"
            full_path = os.path.join(full_path, exe)
        return full_path

    def _get_os_major_release(self):
        """
        Gets the major release of the OS.
        For instance, if you use Ubuntu 12.04, you'll have 12.

        :rtype: int
        :return: OS Major release version.
        """
        if self._os == self.LINUX:
            cmd = 'lsb_release -a'
            output = self.run_cmd(cmd)
            result = output["std"]
            result_version = re.search(r'Release:\s+([0-9]+)', result, re.MULTILINE)
            if result_version:
                version = result_version.group(1)
                version = int(version)
                self._logger.debug("_get_os_major_release result: %s" % version)
                return version
            else:
                msg = "_get_os_major_release failed"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        else:
            msg = "Operating System not supported [%s]" % self._os
            self.get_logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

    def get_date_time(self):
        """
        Get current date and time of a given computer.

        :rtype: dict
        :return: all currents parameters : DAY, MONTH, YEAR, HOURS, MINUTES, SECONDS
        """
        # Check OS platform
        if self._os != self.LINUX:
            msg = "get_date_time : Not implemented for OS %s" % self._os
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)
        else:
            result_parameters = {}

            cmd = "date +'%d-%m-%Y-%H-%M-%S'"
            output = self.run_cmd(cmd)
            std_output = output["std"]
            std_output = std_output.replace('\n', '')
            std_output = std_output.split('-')
            if len(std_output) < 6:
                msg = "get_date_time : Can't get correct date and time"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

            result_parameters["DAY"] = std_output[0]
            result_parameters["MONTH"] = std_output[1]
            result_parameters["YEAR"] = std_output[2]
            result_parameters["HOURS"] = std_output[3]
            result_parameters["MINUTES"] = std_output[4]
            result_parameters["SECONDS"] = std_output[5]
        return result_parameters

    def set_bt_visibility(self, hci_interface, enableVisibility):
        """
        Set on/off the visibility of a given computer.

        :rtype: none
        """
        if enableVisibility == True:
            cmd = "sudo hciconfig " + hci_interface + " piscan"
        else:
            cmd = "sudo hciconfig " + hci_interface + " noscan"

        output = self.run_cmd(cmd)
        return not str_to_bool_ex(output["ret"])

    def check_bt_interface_availability(self, hci_interface):
        """
        Check if the hci_interface is available a given computer.

        :rtype: none
        """

        cmd = "hcitool dev"
        output = self.run_cmd(cmd)
        output = output["std"]
        if hci_interface in output:
            return True
        else:
            return False
