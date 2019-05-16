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
:summary: This file implements WPA P2P Client
:since: 04/05/2013
:author: smaurel
"""

import subprocess
import sys
import re

from acs_test_scripts.Equipment.WifiP2p.Interface.IP2pClient import IP2pClient
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.TestEquipmentException import TestEquipmentException


# Non blocking readline
from Queue import Queue, Empty
import time
ON_POSIX = 'posix' in sys.builtin_module_names
from threading import Thread


def enqueue_output(out, queue):
    """
    read the queue.
    """
    for line in iter(out.readline, ''):
        queue.put(line)
    out.close()


def run_process(args):
    """
    create a new process to manage connection
    """
    process = subprocess.Popen(args,
                               shell=False,
                               stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT,
                               bufsize=1,
                               close_fds=ON_POSIX)
    queue = Queue()
    thread = Thread(target=enqueue_output, args=(process.stdout, queue))  # pylint: disable=E1101
    thread.daemon = True  # thread dies with the program
    thread.start()
    return process, queue


class WifiP2PClientCLI(BaseV2, IP2pClient):
    """
    Implementation of P2P Client interface
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)

        self._p2pclient_ongoing = False
        self._wpa_cli_checked = False

        self._p2p_interface = ""

        self._cli_process = None
        self._cli_queue = None

        self._old_reg = ""

        self.init()

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self._logger.info("init")

        # start wpa_cli
        if self._device.get_serial_number():
            args = ['adb', '-s', self._device.get_serial_number(), 'shell']
        else:
            args = ['adb', 'shell']
        (self._cli_process, self._cli_queue) = run_process(args)
        self.__read_stdout(1)

        self.__write_stdin('killall wpa_cli\n')
        self.__read_stdout(1)

        # Check WPA_Cli existence
        self.__check_wpa_cli()

    def release(self):
        """
        Releases DUT resources and close connection.
        """
        self._logger.info("P2P release")

        self.__write_stdin('killall wpa_cli\n')
        self._p2pclient_ongoing = False

        if self._cli_queue is not None:
            self._cli_process.stdin.close()  # pylint: disable=E1101
            self._cli_process.terminate()  # pylint: disable=E1101

        self._cli_queue = None
        self._cli_process = None

    def set_lan_interface_ip_address(self, ip_address, timeout):
        """
        Set the interface ip address.
        :type ip_address: str
        :param ip_address: the ip address to set on DUT interface
        :type timeout: int
        :param timeout: time (in seconds) to wait for the reply
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        # Empty buffer
        self.__read_stdout(2)

        clioutput = ""
        result = None
        self.__write_stdin('interface\n')
        while timeout >= 0 and result is None:
            clioutput += self.__read_stdout(1)
            if not clioutput:
                self._logger.warning("P2P interface output " + clioutput)

            # Regex resulting group will be 1 or more number
            result = re.search(r"(p2p-p2p0-\d+)", clioutput)
            if result is not None:
                break
            timeout -= 1

        if result is None:
            msg = "Could not get device interface p2p-p2p0-x"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        self._p2p_interface = result.groups()[0]
        self._logger.debug("cliaddress: " + self._p2p_interface)

        # format the ifconfig command
        self._exec("adb shell busybox ifconfig " + self._p2p_interface + " " + ip_address, 1)
        self._logger.debug("set ip to " + ip_address)

    def start(self, lan_interface):
        """
        Start the P2P Client
        :type lan_interface: str
        :param lan_interface: the interface to connect on (not use)
        :rtype: str
        :return: the P2P Client Mac address
        """
        if self._p2pclient_ongoing:
            msg = "Cannot start 2 P2P Client in the same time"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self.__write_stdin('killall wpa_cli\n')

        self._logger.debug("start_P2P client")

        self.__write_stdin("wpa_cli IFNAME=" + lan_interface + "\n")
        self._p2pclient_ongoing = True
        clioutput = self.__read_stdout(1)

        if clioutput:
            self._logger.debug("P2P Client start output " + clioutput)

        if "Could not connect to wpa_supplicant" in clioutput:
            msg = "Could not connect to wpa_supplicant"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self.__write_stdin('status\n')
        clioutput = self.__read_stdout(1)
        result = re.search(r"p2p_device_address=(([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})", clioutput, re.MULTILINE)
        if result is None:
            msg = "Could not get device1 address. " + clioutput
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        client_address = result.groups()[0]
        self._logger.debug("cli2address: " + client_address)

        return client_address

    def stop(self):
        """
        Stop the P2P Client
        """
        self._logger.debug("Stop P2P client")

        self.__write_stdin('quit\n')

        self._p2pclient_ongoing = False

    def remove_all_network(self):
        """
        remove the remained p2p groups networks
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p remove all network...")

        self.__write_stdin("remove_network all\n")
        self.__write_stdin("save\n")

    def find_network(self):
        """
        Find available p2p network
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p find...")

        self.__write_stdin('p2p_find\n')

    def find_dut(self, dut_mac, timeout):
        """
        Find and listen a DUT in the network list

        :type dut_mac: str
        :param dut_mac: DUT Mac address to find
        :type timeout: int
        :param timeout: maximum delay (in seconds) to search
        :rtype: str
        :return: None if not found. The DUT name if found
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return None

        # search device address and name in p2pfind replies
        self._logger.debug("search device address and name in p2pfind replies...")

        clioutput = ""

        regex_string = r"P2P-DEVICE-FOUND.* name='([^']+)' "
        regex = re.compile(regex_string)
        p2p_dest = None

        start = time.time()
        while start + timeout > time.time():
            clioutput += self.__read_stdout(1)
            if clioutput:
                lines = clioutput.split('\n')
                for line in lines:
                    if dut_mac in line:
                        p2p_dest = regex.search(line)
                        if p2p_dest is not None:
                            p2p_dest = p2p_dest.groups()[0]
            if p2p_dest is not None:
                break

        if p2p_dest is None:
            self.__write_stdin('p2p_stop_find\n')
            msg = "Could not find device address in p2p_find before timeout"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._logger.debug("p2p destination: " + p2p_dest)

        return p2p_dest

    def stop_find(self):
        """
        Stop the P2P find
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p stop find...")

        self.__write_stdin('p2p_stop_find\n')

    def listen(self):
        """
        Enables the P2p listen mode
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p listen...")

        self.__write_stdin('p2p_listen\n')

    def set_listen_channel(self, channel):
        """
        Set the P2p listen channel
        :type channel: str
        :param channel: channel to set
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("set p2p listen channel: " + str(channel))

        self.__write_stdin("set p2p_listen_channel " + str(channel) + "\n")

    def set_device_name(self, name):
        """
        Set the P2p device name
        :type channel: str
        :param channel: name to set
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("set p2p device name: " + name)

        self.__write_stdin("set device_name " + name + "\n")

    def connect_pbc(self, dut_mac, go_intent, persistent=False, frequency=2412):
        """
        Launch a connection to the DUT or accept a connection from the DUT on PBC mode.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        if go_intent < 0 or go_intent > 15:
            msg = "GO intent not in range (0-15)"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._logger.debug("p2p connect_pbc...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        self.__write_stdin("p2p_connect " + dut_mac + " pbc " + persistence + \
                           "go_intent=" + str(go_intent) + " freq=" + str(frequency) + "\n")

    def connect_pin_display(self, dut_mac, go_intent, persistent=False, frequency=2412):
        """
        Launch a connection to the DUT with PIN request mode.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        :rtype: str
        :return: The Pin code
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        if go_intent < 0 or go_intent > 15:
            msg = "GO intent not in range (0-15)"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._logger.debug("p2p connect_pin_display...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        self.__write_stdin("p2p_connect " + dut_mac + " pin display " + persistence + \
                           "go_intent=" + str(go_intent) + " freq=" + str(frequency) + "\n")
        clioutput = self.__read_stdout(1)
        if not clioutput:
            msg = "could not get pin code from device"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        pincode = clioutput.split('\n')[1].strip()
        self._logger.debug("client pin code: " + pincode)
        if not pincode:
            msg = "could not get pin code from device"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        return pincode

    def connect_pin_enter(self, dut_mac, pin_code, go_intent, persistent=False, frequency=2412):
        """
        Accept the connect_pin_display request.

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type pin_code: str
        :param pin_code: The pin code to connect
        :type go_intent: integer
        :param go_intent: the local WPA Cli Go value (0 to 15)
        :type persistent: boolean
        :param persistent: persistent connection
        :type frequency: integer
        :param frequency: connection frequency
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        if go_intent < 0 or go_intent > 15:
            msg = "GO intent not in range (0-15)"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self._logger.debug("p2p connect_pin_enter...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        self.__write_stdin("p2p_connect " + dut_mac + " " + pin_code + " keypad " + persistence + \
                           "go_intent=" + str(go_intent) + " freq=" + str(frequency) + "\n")

    def disconnect(self):
        """
        disconnect the other DUT
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p p2p_group_remove " + self._p2p_interface)

        self.__write_stdin("p2p_group_remove " + self._p2p_interface + "\n")

    def __check_wpa_cli(self):
        """
        Check that P2P Client binary tool is installed on the host computer
        """
        if not self._wpa_cli_checked:
            self.__write_stdin('which wpa_cli\n')
            out = self.__read_stdout(1)
            if "/wpa_cli" not in out:
                msg = "wpa_cli is not available on Host"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)

            self._wpa_cli_checked = True

    def __read_stdout(self, timeout):
        """
        read the queue.

        :type Queue: Queue
        :param Queue: the queue
        :type timeout: int
        :param timeout: timeout value
        """
        output = ''

        if self._cli_queue is not None:
            start = time.time()
            while time.time() < start + timeout:
                try:
                    line = self._cli_queue.get(timeout=timeout)
                except Empty:
                    break
                else:
                    output += line

        return output

    def __write_stdin(self, cmd):
        """
        Write a command in the stdin
        """
        if self._cli_process is None:
            msg = "Terminal is not running. Unable to send a command"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        return self._cli_process.stdin.write(cmd)  # pylint: disable=E1101
