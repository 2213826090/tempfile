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

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.WifiP2p.Interface.IP2pClient import IP2pClient
from ErrorHandling.TestEquipmentException import TestEquipmentException
import re
import time
import posixpath


class P2pClient(EquipmentBase, IP2pClient):
    """
    Implementation of P2P Client interface
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        IP2pClient.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        computer = str(bench_params.get_param_value("Computer"))
        # NOTE: import here to avoid circular dependency on
        # EquipmentManager if imported at top level
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        self._em = EquipmentManager()
        self._computer = self._em.get_computer(computer)
        self._lan_interface = ""
        self._p2pclient_ongoing = False

        # Get binary file path
        self._wpa_cli_bin = posixpath.realpath(eqt_params[model]["Binary"])

        self._logger = self.get_logger()

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self._logger.info("init")

        self._computer.init()

        # Open ssh connection
        self._computer.run_cmd("killall wpa_cli")

        # Check WPA_Cli existence
        self._computer.check_command("wpa_cli")

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        self._logger.info("P2P release")

        self._computer.release()

    def set_lan_interface_ip_address(self, ip_address, timeout):
        """
        Set the interface ip address.
        If the WPA Cli is not run, the SSH connection is use,
        else, a new SSH connection is opened and close just after
        :type ip_address: str
        :param ip_address: the ip address to set on interface
        :type timeout: int
        :param timeout: UNUSED here
        """
        self.stop()
        self._computer.run_cmd("ifconfig " + self._lan_interface + " " + ip_address, 1)

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

        self._logger.debug("start_P2P client")

        self._lan_interface = ""

        cli2output = self._computer.run_cmd(self._wpa_cli_bin, 2)
        cli2output = cli2output['std']

        if cli2output:
            self._logger.debug("P2P Client start output " + cli2output)

        # Retrieve the LAN interface
        result = re.search(r"Selected interface '(.+)'", cli2output)
        if result is not None:
            self._lan_interface = result.groups()[0]
        self._logger.debug("cli2lan: " + self._lan_interface)
        msg = "Could not connect to wpa_supplicant"
        if msg in cli2output:
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # Retrieve the used WPA P2P Client Mac address
        cli2output = self._computer.run_cmd("status", 2)
        cli2output = cli2output['std']
        if cli2output:
            self._logger.debug("wpa_cli2 status output: " + cli2output)

        result = re.search(r"p2p_device_address=(([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})", cli2output, re.MULTILINE)
        if result is None:
            msg = "Could not get P2P Client device address"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        client_address = result.groups()[0]
        self._logger.debug("cli2address: " + client_address)

        self._p2pclient_ongoing = True
        return client_address

    def stop(self):
        """
        Stop the P2P Client
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("Stop P2P client")

        # Stop the WPA Client
        self._computer.run_cmd("quit")

        # Kill wpa_cli
        self._computer.run_cmd("killall wpa_cli")

        self._p2pclient_ongoing = False

    def remove_all_network(self):
        """
        remove the remaining p2p groups networks
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p remove all network...")

        self._computer.run_cmd("remove_network all")
        self._computer.run_cmd("save")

    def find_network(self):
        """
        Find available p2p network
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p find...")
        self._computer.run_cmd("p2p_find")

    def find_dut(self, dut_mac, timeout):
        """
        Find and listen a DUT in the network list

        :type dut_mac: str
        :param dut_mac: DUT Mac adress to found
        :type timeout: int
        :param timeout: maximum delay (in seconds) to search
        :rtype: str
        :return: None if not found. The Dut name if found
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        # search device address and name in p2pfind replies
        self._logger.debug("search device address and name in p2pfind replies...")

        regex_string = "P2P-DEVICE-FOUND.* name='([^']+)' "
        regex = re.compile(regex_string)
        p2p_dest = None

        start = time.time()
        while start + timeout > time.time():
            cli1output = self._computer.read(1)
            if cli1output:
                lines = cli1output.split('\n')
                for line in lines:
                    if dut_mac in line:
                        p2p_dest = regex.search(line)
                        if p2p_dest is not None:
                            p2p_dest = p2p_dest.groups()[0]
                            break
            if p2p_dest is not None:
                break

        if p2p_dest is None:
            self._computer.run_cmd("p2p_stop_find")
            msg = "Could not find device address in p2p_find"
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

        self._computer.run_cmd("p2p_stop_find")

    def listen(self):
        """
        Enable the P2p listen mode
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("p2p listen...")

        self._computer.run_cmd("p2p_listen")

    def set_listen_channel(self, channel):
        """
        Set the P2p listen channel
        :type channel: int
        :param channel: channel to set
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("set p2p listen channel: " + str(channel))

        self._computer.run_cmd("set p2p_listen_channel " + str(channel))

    def set_device_name(self, name):
        """
        Set the P2p device name
        :type name: str
        :param name: the device name
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        self._logger.debug("set p2p device name: " + name)

        self._computer.run_cmd("set device_name " + name)

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

        self._logger.debug("p2p connect_pbc...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        self._computer.run_cmd("p2p_connect " + dut_mac + " pbc " + persistence + \
                               " go_intent=" + str(go_intent) + " freq=" + str(frequency))

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

        self._logger.debug("p2p connect_pin_display...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        cli1output = self._computer.run_cmd("p2p_connect " + dut_mac + " pin display " + persistence + \
                                            "go_intent=" + str(go_intent) + " freq=" + str(frequency), 3)
        cli1output = cli1output['std']

        pincode = cli1output.split('\n')
        if len(pincode) < 2:
            msg = "could not get pin code from device"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._logger.debug("client pin code: " + pincode)
        pincode = pincode[1].strip()

        return pincode

    def connect_pin_enter(self, dut_mac, pin_code, go_intent, persistent=False, frequency=2412):
        """
        Accept a connection from the DUT on PIN request mode.

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

        self._logger.debug("p2p connect_pin_enter...")

        persistence = ""
        if persistent:
            persistence = "persistent "

        self._computer.run_cmd("p2p_connect " + dut_mac + " " + pin_code + " keypad " + persistence + \
                               "go_intent=" + str(go_intent) + " freq=" + str(frequency))

    def disconnect(self):
        """
        disconnect the other DUT
        """
        if not self._p2pclient_ongoing:
            self._logger.warning("P2P not started")
            return

        if self._lan_interface is not None:
            self._logger.debug("p2p p2p_group_remove " + self._lan_interface)
            self._computer.run_cmd("p2p_group_remove " + self._lan_interface)
