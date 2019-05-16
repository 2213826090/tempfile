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
:summary: implementation of Cisco1250 family configurable AP
:since:06/11/2011
:author: jpstierlin
"""

import os
import sys
import telnetlib
import time
import re
import ftplib
from threading import Thread

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiAuthenticationTypes

import UtilitiesFWK.Utilities as Util
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Utilities.NetworkingUtilities import is_valid_mac_address


class Cisco1250(EquipmentBase, IConfigurableAP):

    """
    Implementation of Cisco1250 configurable AP
    """

    # Define list of global for the equipment
    WIFI_RADIOS = {"2.4G": 0, "5G": 1}
    WIFI_WEP_KEYS = [0, 1, 2, 3]

    # Power values
    POWER_VALUES_2G = ["-1", "2", "5", "8", "11", "14", "17", "20", "max"]
    POWER_VALUES_5G = ["-1", "2", "5", "8", "11", "14", "max"]

    SUPPORTED_WIFI_STANDARD = ['a', 'b', 'g', 'n', 'an', 'bg', 'gb', 'bgn', 'ngb', 'n2.4G', 'n5G', 'off']
    SUPPORTED_WIFI_AUTHENTICATION = ['OPEN', 'WEP64', 'WEP64-OPEN', 'WEP128',
                                     'WEP128-OPEN', 'WPA-PSK-TKIP',
                                     'WPA2-PSK-AES', 'WPA2-PSK-TKIP',
                                     'WPA2-PSK-TKIP-AES',
                                     'EAP-WPA', 'EAP-WPA2']

    DHCP_POOL_NAME = "dhcp-pool"
    DEFAULT_BEACON_PERIOD = 100
    TIME_AP_RESTART = 120

    # Regularity Domain correspondence table between AP and DUT
    RD = {"(-E)": "FR",
          "(-A)": "US",
          "(-J)": "JP",
          "(-I)": "IL",
          "(-C)": "CN",
          "(-S)": "SG",
          "(-T)": "TW"}

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self._bench_params = bench_params
        self.__handle = None
        self._ap_prompt = '#'

        self._ssids = []
        self._ssid = None
        self._wep_keys = [[None, None, None, None], [None, None, None, None]]
        self._standard = None
        self._hidden = False
        self._radio_status = "undef"
        self._beacon_period = self.DEFAULT_BEACON_PERIOD

        # Attribute to store the regularity domain of the Access Point
        self._regularitydomain = None

        self._console_timeout = str(self._bench_params.get_param_value("ConsoleTimeout", "0"))
        if self._console_timeout.isdigit():
            self._console_timeout = int(self._console_timeout)

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()

    def load_config_file(self, kpi_test, srv_address=None):
        """
        load the configuration file to the AP.
        The file contains commands to set AP configuration.

        :type config_file: str
        :param config_file: local path of the file to upload

        :type srv_address: str
        :param srv_address: The ip address of the computer where the file is
        """
        # Read config_file path from BenchConfig
        try:
            config_file = self._bench_params._BenchConfigParameters__dict["Config_file_list"][kpi_test]["value"]
        except Exception as exception:
            self.get_logger().error("Unable to find AP config file in BenchConfig for this test : %s" % str(exception))
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Unable to find AP config file in BenchConfig for this test : %s"
                                         % str(exception))

        # Extract credentials from Bench_config
        username = self._bench_params.get_param_value("username", "Cisco")
        password = self._bench_params.get_param_value("password", "Cisco")

        # Get local ip address is not given
        if srv_address is None:
            srv_address = self.__get_local_ip()

        self.get_logger().info("Starting to load config file for Cisco AP")
        # Start FTP server on a new thread, in order to transfer file to AP
        thread = Thread(target=self.__start_ftp_server, args=(srv_address,))
        thread.start()
        try:
            # Place file on FTP server. If it fails, abort.
            if not self.__upload_file_to_ftp(srv_address, config_file):
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                             "The config file could not be send to the AP")

            # Initialize connexion with Cisco AP and send command for
            self.init()
            self._send_cmd("copy ftp://{0}:{1}@{2}/config.txt nvram:startup-config".format(username,
                                                                                           password,
                                                                                           srv_address), 0)
            result = self._send_cmd("startup-config", 0)
            loading = False
            loading_ok = False
            # Result should be a list, we need to parse it and check if error occurred
            for item in result:
                if "loading config.txt" in item.lower():
                    loading = True
                if "[ok - " in item.lower():
                    loading_ok = True
            if not loading and not loading_ok:
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                             "Fail to load AP config : %s" % result)
            # Restart AP to load new configuration - '\n' is used to valid 'reload' function
            self._send_cmd("reload", 0)
            self._send_cmd("\n", 0)
            # Wait for AP reboot and try to reconnect
            self.release()
            self.get_logger().info("Wait %ss for AP reboot..." % self.TIME_AP_RESTART)
            time.sleep(self.TIME_AP_RESTART)
            self.init()
            self.get_logger().info("Config file correctly loaded to Cisco AP")
        except Exception as load_exception:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Exception during load_config_file : %s" % load_exception)
        finally:
            # closes ftp server and thread after transfer
            self.server.close_all()
            thread.join()

    def __start_ftp_server(self, srv_address):
        """
        Starts a FTP server in order to send file to Cisco AP

        :type srv_address: str
        :param srv_address: The ip address of the computer where the file is
        """
        # The import is done inside the method in order to avoid changing file dependencies
        from pyftpdlib import authorizers, handlers, servers
        if not os.path.isdir('C:\\Cisco'):
            os.mkdir('C:\\Cisco')
        # Set Authorized user list (Here, only Cisco)
        authorizer = authorizers.DummyAuthorizer()
        authorizer.add_user(self._bench_params.get_param_value("username", "Cisco"),
                            self._bench_params.get_param_value("password", "Cisco"),
                            'C:\\Cisco',
                            perm='elradfmwM')
        handler = handlers.FTPHandler
        handler.authorizer = authorizer
        address = (srv_address, 21)
        try:
            # Instantiate server, and launch it.
            self.server = servers.FTPServer(address, handler)
            self.server.serve_forever()
        except Exception, e:
            print Exception, e
            self.get_logger().error("FTP server already started")

    def __get_local_ip(self):
        """
        Get the local ip address of the bench

        :return: the local ip address
        :rtype: str
        """
        # Get Cisco ip configured in bench config
        host = str(self._bench_params.get_param_value("IP"))

        found_addresses = []
        if sys.platform == "win32":
            # Launch 'ipconfig' command line function and parse each line of output
            ipconfig = os.popen("ipconfig").readlines()
            for line in ipconfig:
                # If result contains 'IPv4 Address', get the address
                if "IPv4 Address".lower() in line.lower():
                    # Parse line to find an IPV4 address format XXX.XXX.XXX.XXX (X is a digit)
                    ip_address = re.findall(r'((?:[\d]+\.){3}[\d]+)', line)
                    if len(ip_address) > 0:
                        for address in ip_address:
                            if NetworkingUtil.is_valid_ipv4_address(address):
                                found_addresses.append(address)
        elif sys.platform == "linux2":
            pass
            # TODO : implement

        # Check if we have found addresses
        if len(found_addresses) == 0:
            msg = "Failed to find IPV4 address on local computer"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # For each found address, check if it can be on the same network - we use 255.255.255.0 netmask
        valid_ip = []
        list_cisco = str(host).split(".")
        for address in found_addresses:
            list_address = str(address).split(".")
            is_ip_valid = True
            for index, item in enumerate(list_address):
                # Check each item except the latest item (netmask : 255.255.255.0)
                if index != 3:
                    if item != list_cisco[index]:
                        is_ip_valid = False
            # If IP is valid, add it to the list
            if is_ip_valid:
                valid_ip.append(address)

        # Check if we have found valid IP
        if len(valid_ip) == 0:
            msg = "Failed to find compatible IPV4 address with Cisco address : %s" % host
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # If we have found multiple valid IP, just take one
        self._logger.debug("Found IP : %s - Use IP : %s" % (valid_ip, valid_ip[0]))

        return valid_ip[0]

    def __upload_file_to_ftp(self, srv_address, config_file):
        """
        Uploads the given File to a fix place on FTP server (=local machine)

        :type srv_address: str
        :param srv_address: The ip address of the computer where the file is

        :type config_file: str
        :param config_file: local path of the file to upload

        :return: a flag for method output status
        :rtype: boolean
        """
        session = None
        try:
            # Open a ftp connexion to the local ftp server
            session = ftplib.FTP(srv_address,
                                 self._bench_params.get_param_value("username", "Cisco"),
                                 self._bench_params.get_param_value("password", "Cisco"))
            with open(config_file, 'rb') as current_file:
                # Store the file on the server
                session.storbinary('STOR config.txt', current_file)
            return True
        except Exception as exception:
            self.get_logger().error("File could not be reached by ftp server : %s" % str(exception))
            return False
        finally:
            if session is not None:
                session.quit()

    def __connect_via_telnet(self, host, username, password):
        """
        connect Cisco1250 via telnet thus config the equipment

        :type host: str
        :param host: access point IP address

        :type username: str
        :param username: access point admin login

        :type password: str
        :param password: access point admin password

        :raise: TestEquipmentException
        """
        self.get_logger().debug("Open telnet connection to equipment.")

        # Initialize telnet session
        telnet_session = telnetlib.Telnet()
        # debug level: 0->disable / 1->enable
        telnet_session.set_debuglevel(0)

        try:
            telnet_session.open(host)
            telnet_session.read_until("Username:", 5)
            telnet_session.write(str(username) + "\n")
            telnet_session.read_until("Password:", 5)
            telnet_session.write(str(password) + "\n")
            telnet_session.read_until(">", 5)
            telnet_session.write("enable\n")
            telnet_session.read_until("Password:", 5)
            telnet_session.write(str(password) + "\n")
            self._ap_prompt = telnet_session.read_until("#", 5).strip()
        except:
            msg = "Connection via telnet failed."
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        try:
            # get running config
            telnet_session.write("terminal length 0\n")
            telnet_session.read_until(self._ap_prompt, 5)
            telnet_session.write("show running-config\n")
            config_file = telnet_session.read_until(self._ap_prompt, 10)

            # extract SSIDs
            config_file_lines = config_file.split('\r\n')
            for line in config_file_lines:
                match = "dot11 ssid "
                if line.startswith(match):
                    self._ssids.append(line[len(match):].strip())

            # extract wep keys

            for i in Cisco1250.WIFI_RADIOS.values():
                found = None
                for line in config_file_lines:
                    if found is None:
                        match = "interface Dot11Radio" + str(i)
                        if line.startswith(match):
                            found = line
                    elif line.startswith(' '):
                        match = "encryption key "
                        if line.strip().startswith(match):
                            key_id = int(line.strip()[len(match):].strip().split(' ')[0])
                            self._wep_keys[i][key_id - 1] = True
                    else:
                        break

        except:
            msg = "Read configuration failed."
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

        # Update handle value
        self._set_handle(telnet_session)

    def __disconnect_via_telnet(self):
        """
        disconnect equipment from telnet
        """
        self.get_logger().debug("Close telnet connection to the equipment.")
        if self.get_handle() is not None:
            self.get_handle().close()
            self._set_handle(None)

    def _send_cmd(self, cmd, lines=2, reply='#'):
        """
        Send command to the equipment

        :type cmd: str
        :param cmd: Command to send

        :type lines: integer
        :param lines: max number of lines to be read, 0 to ignore this param

        :type reply: str
        :param reply: reply str to trigger

        :rtype: str
        :return: The output str returned after the command
        """
        self.get_logger().debug("Send command %s" % str(cmd))

        self.get_handle().write(str(cmd) + "\n")
        ret = self.get_handle().read_until(reply, 5).strip().split('\r\n')
        if len(ret) == 0:
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, "Command timeout.")
        if 0 < lines < len(ret):
            ret[0] = ' ' * len(ret[-1].strip()) + ret[0].strip()
            error_msg = "Command error: \n" + '\n'.join(ret[:-1])
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        return ret

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def _set_handle(self, handle):
        """
        Sets the connection handle of the equipment
        """
        self.__handle = handle

    def _delete_ssids(self):
        """
        Delete all ssids from the Equipment
        """
        if len(self._ssids) != 0:
            self._send_cmd("configure terminal", 3)
            try:
                for ssid in self._ssids:
                    self._send_cmd("no dot11 ssid " + ssid)
                self._ssids = []
            finally:
                self._send_cmd("end")  # exit configure

    def __delete_keys(self, radio):
        """
        Clear all security keys on the equipment

        :type radio : int
        :param radio : radio to use
        """
        i = int(radio)
        for k in Cisco1250.WIFI_WEP_KEYS:
            if self._wep_keys[i][k] is not None:
                self._send_cmd("no encryption key " + str(k + 1))
                self._wep_keys[i][k] = None

    def _set_wifi_authentication_OPEN(self):
        """
        Set AP Encryption type: Open
        """
        self.get_logger().debug("Set wifi authentication to OPEN")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("no encryption mode")
            self._send_cmd("exit")  # exit interface
        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("exit")  # exit dot11

    def _set_wifi_authentication_WEP64(self, authentication, passphrase):
        """
        Set AP Encryption type to WEP64

        :type authentication: String
        :param authentication: WEP64 or WEP64-OPEN

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 64 bits")

        # WEP64 uses a 40-bit key = 5 bytes
        if len(passphrase) == 5:
            # Then we should transform ascii chars into hexadecimal values
            passphrase = Util.char2hexa(passphrase)
        elif not re.match("^[0-9a-fA-F]{10}$", passphrase):
            msg = "WEP64 passphrase should be 5 chars or 10 hexadecimal " + \
                "chars. Currently [%s]" % passphrase
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("no encryption mode ciphers")
            self._send_cmd("encryption mode wep mandatory")
            # Passphrase set AFTER "encryption mode wep mandatory" instruction
            self._send_cmd("encryption key 1 size 40bit 0 " + passphrase +
                           " transmit-key")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        if authentication == "WEP64":
            self._send_cmd("authentication shared")
        else:
            self._send_cmd("authentication open")
        self._send_cmd("exit")

    def _set_wifi_authentication_WEP128(self, authentication, passphrase):
        """
        Set AP Encryption type to WEP128

        :type authentication: String
        :param authentication: WEP128 or WEP128-OPEN

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WEP 128 bits")

        # WEP128 uses a 104-bit key = 13 bytes
        if len(passphrase) == 13:
            # Then we should transform ascii chars into hexadecimal values
            passphrase = Util.char2hexa(passphrase)
        elif not re.match("^[0-9a-fA-F]{26}$", passphrase):
            msg = "WEP128 passphrase should be 13 chars or 26 hexadecimal " + \
                "chars. Currently [%s]" % passphrase
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("no encryption mode ciphers")
            self._send_cmd("encryption mode wep mandatory")
            # Passphrase set AFTER "encryption mode wep mandatory" instruction
            self._send_cmd("encryption key 1 size 128bit 0 " + passphrase +
                           " transmit-key")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        if authentication == "WEP128":
            self._send_cmd("authentication shared")
        else:
            self._send_cmd("authentication open")
        self._send_cmd("exit")

    def _set_wifi_authentication_WPA_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK TKIP")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("encryption mode ciphers tkip")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("authentication key-management wpa version 1")
        self._send_cmd("wpa-psk ascii 0 " + passphrase)
        self._send_cmd("exit")

    def _set_wifi_authentication_WPA_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA PSK AES")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("encryption mode ciphers aes-ccm")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("authentication key-management wpa version 1")
        self._send_cmd("wpa-psk ascii 0 " + passphrase)
        self._send_cmd("exit")

    def _set_wifi_authentication_WPA2_PSK_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK AES")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("encryption mode ciphers aes-ccm")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("authentication key-management wpa version 2")
        self._send_cmd("wpa-psk ascii 0 " + passphrase)
        self._send_cmd("exit")

    def _set_wifi_authentication_WPA2_PSK_TKIP(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("encryption mode ciphers tkip")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("authentication key-management wpa version 2")
        self._send_cmd("wpa-psk ascii 0 " + passphrase)
        self._send_cmd("exit")

    def _set_wifi_authentication_WPA2_PSK_TKIP_AES(self, passphrase):
        """
        Set AP Encryption type to WPA2 PSK TKIP AES

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication
        """
        self.get_logger().debug("Set wifi authentication to WPA2 PSK TKIP AES")

        for radio in Cisco1250.WIFI_RADIOS.values():
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd("encryption mode ciphers tkip aes-ccm")
            self._send_cmd("exit")

        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open")
        self._send_cmd("authentication key-management wpa version 2")
        self._send_cmd("wpa-psk ascii 0 " + passphrase)
        self._send_cmd("exit")

    def _set_wifi_authentication_EAP_WPA(self,
                                         radiusip, radiusport, radiussecret,
                                         standard_type):
        """
        set AP Encryption type: EAP-WPA

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP
                       and Radius server (optional)

        :type standard_type: str
        :param standard_type: The wifi standard used to determine if the \
                            channel has to be set for 2.4GHz or 5GHz
        """
        self.get_logger().debug("Set wifi authentication to WPA EAP")

        if radiusip is None or radiusport is None or radiussecret is None or \
                radiusip == "" or radiusport == "" or radiussecret == "":
            msg = "Radius configuration is missing in BenchConfig"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Guess Accounting port
        acct_port = int(radiusport) + 1

        # Radius server configuration
        self._send_cmd("aaa new-model")
        self._send_cmd("aaa group server radius rad_eap")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_mac")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_acct")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_admin")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server tacacs+ tac_admin")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_pmip")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius dummy")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap1")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_acct1")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap2")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap3")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa authentication login eap_methods group rad_eap")
        self._send_cmd("aaa authentication login mac_methods local")
        self._send_cmd("aaa authentication login eap_methods1 group rad_eap1")
        self._send_cmd("aaa authentication login eap_methods2 group rad_eap2")
        self._send_cmd("aaa authentication login eap_methods3 group rad_eap3")
        self._send_cmd("aaa authorization exec default local")
        self._send_cmd("aaa accounting network acct_methods start-stop group rad_acct")
        self._send_cmd("aaa accounting network acct_methods1 start-stop group rad_acct1")
        self._send_cmd("aaa session-id common")

        # SSID configuration
        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open eap eap_methods3")
        self._send_cmd("authentication key-management wpa version 1")
        self._send_cmd("accounting acct_methods1")
        self._send_cmd("exit")

        # Radio configuration
        if standard_type not in AcsWifiFrequencies.WIFI_STANDARD_5G:
            radio = self.WIFI_RADIOS['2.4G']
        else:
            radio = self.WIFI_RADIOS['5G']
        self._send_cmd("interface dot11radio " + str(radio))
        self._send_cmd("encryption mode ciphers tkip")
        self._send_cmd("ssid " + self._ssid)
        self._send_cmd("station-role root access-point")
        self._send_cmd("no dot11 extension aironet")
        self._send_cmd("bridge-group 1")
        self._send_cmd("bridge-group 1 subscriber-loop-control")
        self._send_cmd("bridge-group 1 block-unknown-source")
        self._send_cmd("no bridge-group 1 source-learning")
        self._send_cmd("no bridge-group 1 unicast-flooding")
        self._send_cmd("bridge-group 1 spanning-disabled")
        self._send_cmd("exit")

        # Radius server configuration
        self._send_cmd("ip radius source-interface BVI1")
        self._send_cmd("radius-server attribute 32 include-in-access-req format %h")
        self._send_cmd("radius-server host %s auth-port %s acct-port %s key %s"
                       % (str(radiusip), str(radiusport), str(acct_port),
                          str(radiussecret)))
        self._send_cmd("radius-server vsa send accounting")

    def _set_wifi_authentication_EAP_WPA2(self,
                                          radiusip, radiusport, radiussecret,
                                          standard_type):
        """
        set AP Encryption type: EAP-WPA2

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP
                       and Radius server (optional)

        :type standard_type: str
        :param standard_type: The wifi standard used to determine if the \
                            channel has to be set for 2.4GHz or 5GHz
        """
        self.get_logger().debug("Set wifi authentication to WPA2 EAP")

        if radiusip is None or radiusport is None or radiussecret is None or \
                radiusip == "" or radiusport == "" or radiussecret == "":
            msg = "Radius configuration is missing in BenchConfig"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Guess Accounting port
        acct_port = int(radiusport) + 1

        # Radius server configuration
        self._send_cmd("aaa new-model")
        self._send_cmd("aaa group server radius rad_eap")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_mac")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_acct")
        self._send_cmd("server %s auth-port %s acct-port %s"
                           % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_admin")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server tacacs+ tac_admin")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_pmip")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius dummy")
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap1")
        self._send_cmd("server %s auth-port %s acct-port %s"
                       % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_acct1")
        self._send_cmd("server %s auth-port %s acct-port %s"
                           % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap2")
        self._send_cmd("server %s auth-port %s acct-port %s"
                        % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa group server radius rad_eap3")
        self._send_cmd("server %s auth-port %s acct-port %s"
                        % (str(radiusip), str(radiusport), str(acct_port)))
        self._send_cmd("exit")
        self._send_cmd("aaa authentication login eap_methods group rad_eap")
        self._send_cmd("aaa authentication login mac_methods local")
        self._send_cmd("aaa authentication login eap_methods1 group rad_eap1")
        self._send_cmd("aaa authentication login eap_methods2 group rad_eap2")
        self._send_cmd("aaa authentication login eap_methods3 group rad_eap3")
        self._send_cmd("aaa authorization exec default local")
        self._send_cmd("aaa accounting network acct_methods start-stop group rad_acct")
        self._send_cmd("aaa accounting network acct_methods1 start-stop group rad_acct1")
        self._send_cmd("aaa session-id common")

        # SSID configuration
        self._send_cmd("dot11 ssid " + self._ssid)
        self._send_cmd("authentication open eap eap_methods3")
        self._send_cmd("authentication key-management wpa version 2")
        self._send_cmd("accounting acct_methods1")
        self._send_cmd("exit")

        # Radio configuration
        if standard_type not in AcsWifiFrequencies.WIFI_STANDARD_5G:
            radio = Cisco1250.WIFI_RADIOS['2.4G']
        else:
            radio = Cisco1250.WIFI_RADIOS['5G']
        self._send_cmd("interface dot11radio " + str(radio))
        self._send_cmd("encryption mode ciphers aes-ccm")
        self._send_cmd("ssid " + self._ssid)
        self._send_cmd("station-role root access-point")
        self._send_cmd("no dot11 extension aironet")
        self._send_cmd("bridge-group 1")
        self._send_cmd("bridge-group 1 subscriber-loop-control")
        self._send_cmd("bridge-group 1 block-unknown-source")
        self._send_cmd("no bridge-group 1 source-learning")
        self._send_cmd("no bridge-group 1 unicast-flooding")
        self._send_cmd("bridge-group 1 spanning-disabled")
        self._send_cmd("exit")

        # Radius server configuration
        self._send_cmd("ip radius source-interface BVI1")
        self._send_cmd("radius-server attribute 32 include-in-access-req format %h")
        self._send_cmd("radius-server host %s auth-port %s acct-port %s key %s"
                       % (str(radiusip), str(radiusport), str(acct_port),
                              str(radiussecret)))
        self._send_cmd("radius-server vsa send accounting")

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        if not str(self._console_timeout).isdigit():
            msg = "Wrong value for ConsoleTime in BenchConfig: " + str(self._console_timeout)
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Retrieve parameters from BenchConfig for connection
        host = str(self._bench_params.get_param_value("IP"))
        username = str(self._bench_params.get_param_value("username"))
        password = str(self._bench_params.get_param_value("password"))

        # Open telnet session
        connection_attempts = 5
        while connection_attempts > 0:
            try:
                self.__connect_via_telnet(host, username, password)
                break
            except TestEquipmentException as e:
                # In case of Telnet connection failure, wait for the Telnet timeout to expire,
                # in order to be able to reconnect during next attempt
                msg = "Waiting for the Telnet connection to reset from the AP "
                msg += "(%d min)" % self._console_timeout
                self._logger.warning(msg)
                time.sleep(self._console_timeout * 60 + 10)

                self._logger.info(e)
                connection_attempts -= 1
                if connection_attempts <= 0:
                    raise e
                self.release()

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")

        self.__disconnect_via_telnet()

    def create_ssid(self, ssid, hidden=False):
        """
        Create ssid on the equipment

        :type ssid: str
        :param ssid: SSID to create

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled
        """
        self.get_logger().info("Create ssid '%s'" % str(ssid))

        self._ssid = str(ssid)
        self._ssids.append(self._ssid)

        self._send_cmd("configure terminal", 3)
        try:
            # Send commands to equipment
            self._send_cmd("dot11 ssid " + self._ssid)
            if hidden:
                self._send_cmd("no guest-mode")
            else:
                self._send_cmd("guest-mode")
            # save 'hidden' value for specific 1260 settings
            self._hidden = hidden
            self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_authentication(self, authentication_type, passphrase="",
                                radiusip=None, radiusport=None,
                                radiussecret=None,
                                standard_type=None):
        """
        Set the authentication on the equipment

        :type authentication_type: String
        :param authentication_type: Authentication supported by the equipment

        :type passphrase: String
        :param passphrase: Passphrase used for the authentication

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP and Radius server (optional)

        :type standard_type: str
        :param standard_type: The wifi standard used to determine if the \
                            channel has to be set for 2.4GHz or 5GHz
        """
        self.get_logger().info("Set wifi authentication to '%s'" % str(authentication_type))

        # SSID should set before setting the authentication
        if self._ssid in (None, "None", ""):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "SSID should be set before setting wifi authentication.")

        self._send_cmd("configure terminal", 3)
        try:
            if authentication_type == WifiAuthenticationTypes.OPEN:
                self._set_wifi_authentication_OPEN()

            elif authentication_type == WifiAuthenticationTypes.WEP_64 \
                    or authentication_type == WifiAuthenticationTypes.WEP_64_OPEN:
                self._set_wifi_authentication_WEP64(authentication_type, passphrase)

            elif authentication_type == WifiAuthenticationTypes.WEP_128 \
                    or authentication_type == WifiAuthenticationTypes.WEP_128_OPEN:
                self._set_wifi_authentication_WEP128(authentication_type, passphrase)

            elif authentication_type == WifiAuthenticationTypes.WPA_PSK_TKIP:
                self._set_wifi_authentication_WPA_PSK_TKIP(passphrase)

            elif authentication_type == WifiAuthenticationTypes.WPA_PSK_AES:
                self._set_wifi_authentication_WPA_PSK_AES(passphrase)

            elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_AES:
                self._set_wifi_authentication_WPA2_PSK_AES(passphrase)

            elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP:
                self._set_wifi_authentication_WPA2_PSK_TKIP(passphrase)

            elif authentication_type == WifiAuthenticationTypes.WPA2_PSK_TKIP_AES:
                self._set_wifi_authentication_WPA2_PSK_TKIP_AES(passphrase)

            elif authentication_type == WifiAuthenticationTypes.EAP_WPA:
                self._set_wifi_authentication_EAP_WPA(radiusip, radiusport,
                                                      radiussecret, standard_type)

            elif authentication_type == WifiAuthenticationTypes.EAP_WPA2:
                self._set_wifi_authentication_EAP_WPA2(radiusip, radiusport,
                                                       radiussecret, standard_type)

            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Unsupported wifi authentication '%s'" % str(authentication_type))
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_standard(self, standard, enable_mimo=False):
        """
        Set wifi standard

        :type standard: String
        :param standard: Wifi standard to set
        :type enable_mimo: Boolean
        :param enable_mimo: enable Multiple Input Multiple Output feature on the AP
        """
        self.get_logger().info("Set wifi standard to '%s'" % str(standard))

        mimo_rates = ""
        antenna = "right-a"
        # is MIMO enabled ?
        if enable_mimo:
            if "n" not in standard:
                # MIMO is available only from standard N
                self._logger.warning("MIMO feature is not available with '%s' standard" % standard)
            else:
                # MIMO enabled
                mimo_rates = " m8. m9. m10. m11. m12. m13. m14. m15."
                antenna = "diversity"

        self._send_cmd("configure terminal", 3)
        try:
            if standard == "a":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("speed basic-6.0 basic-9.0 basic-12.0 basic-18.0 basic-24.0 basic-36.0 basic-48.0 " +
                               "basic-54.0")
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface

            elif standard == "b":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("speed basic-1.0 basic-2.0 basic-5.5 basic-11.0")
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            elif standard == "g":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("speed 1.0 2.0 5.5 11.0 basic-6.0 basic-9.0 basic-12.0 basic-18.0 basic-24.0 " +
                               "basic-36.0 basic-48.0 basic-54.0")
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            elif standard == "gb" or standard == "bg":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("speed basic-1.0 basic-2.0 basic-5.5 basic-11.0 basic-6.0 basic-9.0 basic-12.0 " +
                               "basic-18.0 basic-24.0 basic-36.0 basic-48.0 basic-54.0")
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            elif standard == "ngb" or standard == "bgn":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("speed basic-1.0 basic-2.0 basic-5.5 basic-11.0 basic-6.0 basic-9.0 basic-12.0 " +
                               "basic-18.0 basic-24.0 basic-36.0 basic-48.0 basic-54.0 m0-7" + mimo_rates)
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            elif standard == "an":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("speed basic-6.0 basic-9.0 basic-12.0 basic-18.0 basic-24.0 basic-36.0 basic-48.0 " +
                               "basic-54.0 m0. m1. m2. m3. m4. m5. m6. m7." + mimo_rates)
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface

            elif standard == "n2.4G" or standard == "n":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("speed basic-1.0 basic-2.0 basic-5.5 basic-11.0 basic-6.0 basic-9.0 basic-12.0 " +
                               "basic-18.0 basic-24.0 basic-36.0 basic-48.0 basic-54.0 " +
                               "m0. m1. m2. m3. m4. m5. m6. m7." + mimo_rates)
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            elif standard == "n5G":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("speed basic-6.0 m0. m1. m2. m3. m4. m5. m6. m7." + mimo_rates)
                self._send_cmd("ssid " + self._ssid)
                self._send_cmd("no shutdown")
                self._send_cmd("antenna transmit " + antenna)
                self._send_cmd("antenna receive " + antenna)
                self._send_cmd("exit")  # exit interface

            elif standard == "off":
                self._send_cmd("interface dot11radio 0")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface
                self._send_cmd("interface dot11radio 1")
                self._send_cmd("shutdown")
                self._send_cmd("exit")  # exit interface

            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Unsupported wifi standard '%s'." % str(standard))
        finally:
            self._send_cmd("end")  # exit configure

        self._standard = standard

    def set_wifi_channel(self, standard, channel):
        """
        Set wifi channel

        :type standard: str
        :param standard: The wifi standard used to determine if the channel \
                         has to be set for 2.4GHz or 5GHz
        :type channel: integer or Digit-String
        :param channel: The wifi channel to set, 2.4GHz:1-14(0:auto), 5GHz:36-165(0:auto)
        """
        channel = str(channel)
        self._send_cmd("configure terminal", 3)
        try:
            if standard not in AcsWifiFrequencies.WIFI_STANDARD_5G:
                self.get_logger().info("Set wifi 2.4GHz channel to " + channel)

                if int(channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                        "channel is out of range.")

                self._send_cmd("interface dot11radio 0")
                self._send_cmd("channel " + channel)
                self._send_cmd("exit")

            else:
                self.get_logger().info("Set wifi 5GHz channel to " + channel)

                if int(channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                        "channel is out of range.")
                self._send_cmd("interface dot11radio 1")
                frequency = AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G[int(channel)]

                nbr_try = 2
                while nbr_try > 0:
                    nbr_try -= 1
                    try:
                        self._send_cmd("channel " + frequency)
                        nbr_try = 0
                        break
                    except TestEquipmentException as e:
                        if nbr_try <= 0:
                            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Channel can't be used: %s" % str(e))
                        # Retrieve the error message and extract the delay to wait before next connection
                        time_to_wait = 0
                        found_str = re.search("Non-occupancy period of ([0-9]+) .*", str(e))
                        if found_str is None:
                            # if the Exception message is not the error we are expecting, the error is raise to upper level
                            raise

                        time_to_wait = int(found_str.group(1))
                        time_to_wait = (time_to_wait + 1) * 60

                        # release connection to ap
                        self._send_cmd("exit")
                        self._send_cmd("end")  # exit configure
                        self.release()

                        # wait delay but warn the user every 5Mn.
                        sub_time_to_wait = 5 * 60
                        while time_to_wait > 0:
                            if sub_time_to_wait > time_to_wait:
                                sub_time_to_wait = time_to_wait
                            self._logger.info("Waiting for (%dMn) DFS channel to be available" % int(time_to_wait / 60))
                            time.sleep(sub_time_to_wait)
                            time_to_wait -= sub_time_to_wait

                        # reinitialize telnet connection to ap
                        self.init()
                        self._send_cmd("configure terminal", 3)
                        self._send_cmd("interface dot11radio 1")
                        self._logger.info("Retrying frequency setting")

                self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_dtim(self, dtim):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim:
        """
        self.get_logger().info("Set wifi DTIM to '%s'" % str(dtim))
        self._send_cmd("configure terminal", 3)
        try:
            for radio in Cisco1250.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("beacon dtim-period " + str(dtim))
                self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_beacon(self, beacon):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        """
        self.get_logger().info("Set wifi beacon to '%s'" % str(beacon))
        self._send_cmd("configure terminal", 3)
        try:
            for radio in Cisco1250.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("beacon period " + str(beacon))
                self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure
        self._beacon_period = beacon

    def set_wifi_wmm(self, mode):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        if mode in ("on", "1", 1):
            self._logger.info("Set wifi wmm to on")
            mode = 1
        elif mode in ("off", "0", 0):
            self._logger.info("Set wifi wmm to off")
            mode = 0
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Parameter mode is not valid !")

        self._send_cmd("configure terminal", 3)
        try:
            for radio in Cisco1250.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                if mode == 0:
                    self._send_cmd("no dot11 qos mode wmm")
                else:
                    self._send_cmd("dot11 qos mode wmm")
                self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_bandwidth(self, bandwidth, standard):
        """
        Set wifi channel bandwidth

        :type bandwidth: integer
        :param bandwidth: The wifi bandwidth: 20 or 40MHz

        :type standard: str
        :param standard: The wifi standard used to control the validity of
                         the bandwidth value or to select the good antenna.
        """
        bandwidth = int(bandwidth)

        self.get_logger().info("Set wifi bandwidth to '%d'" % bandwidth)

        if bandwidth == 20:
            cmd = "channel width 20"
        elif bandwidth == 40:
            cmd = "channel width 40-below"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi bandwidth '%s'." % str(bandwidth))

        if (standard not in ['an', 'n', 'n2.4G', 'n5G']) and (int(bandwidth) == 40):
            msg = "Trying to set 40Mhz bandwitdh with a non N standard."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # Select the good radio
        if standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
            radio = self.WIFI_RADIOS['5G']
        else:
            radio = self.WIFI_RADIOS['2.4G']

        # Set the bandwitdh
        self._send_cmd("configure terminal", 3)
        try:
            self._send_cmd("interface dot11radio " + str(radio))
            self._send_cmd(cmd)
            self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def set_wifi_power(self, standard, wifi_power):
        """
        Set wifi transmit power in dBm

        :type standard: str
        :param standard: The wifi standard used to control the validity of \
                         the power value
        :type power: str or int
        :param power: wifi transmit power:
            2.4GHz: POWER_VALUES_2G
            5GHz:   POWER_VALUES_5G
        """

        # Control of the value to set
        if standard not in AcsWifiFrequencies.WIFI_STANDARD_5G \
                and str(wifi_power) not in self.POWER_VALUES_2G:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi power value for 5GHz '%s'" % str(wifi_power))
        elif standard in AcsWifiFrequencies.WIFI_STANDARD_5G \
                and str(wifi_power) not in self.POWER_VALUES_5G:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Unsupported wifi power value for 2.4GHz '%s'"
                                % str(wifi_power))

        # Set the power value
        self._send_cmd("configure terminal", 3)
        try:
            cmd = 'power local ' + str(wifi_power)
            if wifi_power in self.POWER_VALUES_2G:
                self._send_cmd("interface dot11radio " + '0')
                self._send_cmd(cmd)
                self._send_cmd("exit")  # exit interface
            if wifi_power in self.POWER_VALUES_5G:
                self._send_cmd("interface dot11radio " + '1')
                self._send_cmd(cmd)
                self._send_cmd("exit")  # exit interface
        finally:
            self._send_cmd("end")  # exit configure

    def _is_supported_config(self, standard_type, authentication_type):
        """
        Check if standard and authentication type combination
        is supported by Equipment

        :type standard_type: str
        :param standard_type: wifi standard type

        :type authentication_type: str
        :param authentication_type: wifi authentication type

        :rtype: boolean
        :return: if supported or not
        """
        if (standard_type in ['n', 'n2.4G', 'n5G'] and authentication_type in
                ['WEP64', 'WEP128']):
            return False
        return True

    def clean_wifi_config(self):
        """
        Clear all WiFi configurations, turn Off radios and disable aironet extensions
        """
        self._send_cmd("configure terminal", 3)
        try:
            for radio in Cisco1250.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("shutdown")
                self.__delete_keys(radio)
                self._send_cmd("no dot11 extension aironet")
                self._send_cmd("exit")  # exit interface

        finally:
            self._send_cmd("end")  # exit configure

        # clear all existing ssids
        self._delete_ssids()

    def set_wifi_config(self,
                        ssid,
                        hidden,
                        standard_type,
                        authentication_type,
                        passphrase,
                        channel=None,
                        dtim=None,
                        beacon=None,
                        wmm=None,
                        bandwidth=None,
                        mimo=False,
                        radiusip=None,
                        radiusport=None,
                        radiussecret=None,
                        configindex=0):
        """
        set wifi config, include standard and authentication

        :type ssid: str
        :param ssid: access point SSID

        :type hidden: Boolean
        :param hidden: True if SSID broadcast is disabled

        :type standard_type: str
        :param standard_type: wifi standard type

        :type authentication_type: str
        :param authentication_type: wifi authentication type

        :type passphrase: str
        :param passphrase: wifi passphrase

        :type channel: integer
        :param channel: wifi channel number (optional)

        :type dtim: integer
        :param dtim: wifi DTIM interval (optional)

        :type beacon: integer
        :param beacon: beacon interval in ms (optional)

        :type wmm: integer
        :param wmm: Enable/Disable wifi wmm (optional)

        :type bandwidth: str
        :param bandwidth: bandwidth to use (optional)

        :type mimo: Boolean
        :param mimo: enable/disable Multiple Input Multiple Output feature on the AP (Optional)

        :type radiusip: str
        :param radius: Address of the radius server (optional)

        :type radiusport: str
        :param radius: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radius: Password to communicate between AP and Radius server (optional)

        :type configindex: integer
        :param configindex: Configuration index - can be 0 to 3 - 0 is the default SSID data (optional)
        """
        self.get_logger().info("Set wifi configuration")

        if configindex != 0:
            msg = "Configuration index can be only zero : %s not correct" % configindex
            raise TestEquipmentException(TestEquipmentException.DEFAULT_ERROR_CODE, msg)

        # Extract time to wait for configuration of the equipment
        configuration_timer = float(self._bench_params.get_param_value("ConfigurationWaitingTime"))

        # Check parameter values
        if standard_type not in Cisco1250.SUPPORTED_WIFI_STANDARD:
            msg = "wifi standard type: %s not correct" % standard_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if authentication_type not in Cisco1250.SUPPORTED_WIFI_AUTHENTICATION:
            msg = "wifi authentication type: %s not correct" % authentication_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        if not self._is_supported_config(standard_type, authentication_type):
            msg = "wifi standard type: %s with authentication type: %s not supported" % (standard_type, authentication_type)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # turn radios off and disable aironet extensions
        self._send_cmd("configure terminal", 3)
        try:
            for radio in Cisco1250.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("shutdown")
                self.__delete_keys(radio)
                self._send_cmd("no dot11 extension aironet")
                self._send_cmd("exit")  # exit interface

        finally:
            self._send_cmd("end")  # exit configure

        # clear all existing ssids
        self._delete_ssids()
        # create specified ssid
        self.create_ssid(ssid, hidden)

        # Set the wifi authentication
        self.set_wifi_authentication(authentication_type, passphrase,
                                     radiusip, radiusport, radiussecret,
                                     standard_type)

        if channel is not None:
            # Set the wifi channel if exists
            self.set_wifi_channel(standard_type, channel)
        if bandwidth is not None:
            # Set the wifi bandwidth if exists
            self.set_wifi_bandwidth(bandwidth, standard_type)
        if dtim is not None:
            # Set the wifi dtim if exists
            self.set_wifi_dtim(dtim)
        if beacon is None:
            beacon = self.DEFAULT_BEACON_PERIOD

        # Set the wifi beacon interval
        self.set_wifi_beacon(beacon)
        if wmm is not None:
            # Enable/disable the wifi wmm if exists
            self.set_wifi_wmm(wmm)

        # set_wifi_standard restarts the needed radios
        self.set_wifi_standard(standard_type, mimo)

        # Take the chance to read and save regulatory domain \
        # after standard has been set
        self.get_regulatorydomain()

        # Configure world-mode to activate regulatory domain sending
        self._enable_regulatorydomain_sent()

        # Wait for configuration time end
        time.sleep(configuration_timer)

    def enable_wireless(self, standard=None):
        """
        enable wireless network

        :type standard: str
        :param standard: type of frequency to enable

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Enable wireless")
        status = True

        if self._radio_status != "ON":
            if self._standard is None:
                msg = "Cannot call enable_wireless() before calling " \
                      + "set_wifi_standard() method"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

            self._send_cmd("configure terminal", 3)
            try:
                if self._standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
                    self._send_cmd("interface dot11radio " +
                                   str(Cisco1250.WIFI_RADIOS["5G"]))
                else:
                    self._send_cmd("interface dot11radio " +
                                   str(Cisco1250.WIFI_RADIOS["2.4G"]))

                self._send_cmd("no shutdown")
                self._send_cmd("exit")
                self._radio_status = "ON"
            except TestEquipmentException:
                self._logger.error("Enable wireless fails")
                status = False
            finally:
                self._send_cmd("end")  # exit configure
                configuration_timer = float(self._bench_params.
                                            get_param_value("ConfigurationWaitingTime"))
                time.sleep(configuration_timer)
        else:
            self._logger.info("Wireless already enabled")

        return status

    def disable_wireless(self):
        """
        disable wireless

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Disable wireless")
        status = True

        if not self.is_radio_off():
            self._send_cmd("configure terminal", 3)
            try:
                for radio in self.WIFI_RADIOS.values():
                    self._send_cmd("interface dot11radio " + str(radio))
                    self._send_cmd("shutdown")
                    self._send_cmd("exit")
                self._radio_status = "OFF"
            except TestEquipmentException:
                self._logger.error("Disable wireless fails")
                status = False
            finally:
                self._send_cmd("end")  # exit configure
                configuration_timer = float(self._bench_params.
                                            get_param_value("ConfigurationWaitingTime"))
                time.sleep(configuration_timer)
        else:
            self._logger.info("Wireless already disabled")

        return status

    def set_acl_mode(self, mode):
        """
        Enable/Disable wifi MAC address filtering

        :type mode: str
        :param mode: can be ("disable", "enable")
        """
        marker = "dot11 association mac-list 777"

        # get running config
        self.get_handle().write("show running-config\n")
        status = self.get_handle().read_until(self._ap_prompt, 10).strip().split('\r\n')
        # Check if acl mode is already set
        if (marker in status) and (mode == "enable"):
            self._logger.info("acl mode already set")
            return
        if (marker not in status) and (mode == "disable"):
            self._logger.info("acl mode already not set")
            return

        if mode in ("disable", "enable"):
            self.get_logger().info("Set acl mode to %s" % mode)
            cmd = ""
            if mode == "enable":
                cmd = marker
            else:
                cmd = "no " + marker
            self._send_cmd("configure terminal", 3)
            try:
                self._send_cmd(cmd)
            except:
                msg = 'set config "%s" failed' % marker
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, cmd)
            finally:
                # exit configure
                self._send_cmd("end")
        else:
            msg = "Parameter mode %s is not valid !" % mode
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        # control that the acl mode is correctly set
        self.get_handle().write("show running-config\n")
        status = self.get_handle().read_until(self._ap_prompt, 10).strip().split('\r\n')
        if (marker not in status) and (mode == "enable"):
            msg = "Unable to enable acl status"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        if (marker in status) and (mode == "disable"):
            msg = "Unable to disable acl status"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def add_mac_address_to_acl(self, mac_addr):
        """
        Add mac address to AP access control list

        :type mac_addr: str
        :param mac_addr: can be "08:00:28:xx:xx:xx" where x is between [0-F]
        """
        # Convert Mac Address to Cisco's format
        mac_addr = str(mac_addr).lower()
        mac = self._convert_mac_format_to_cisco(mac_addr)

        # Check if mac address is already in the list
        self.get_handle().write("show access-lists\n")
        status = self.get_handle().read_until(self._ap_prompt, 5).strip().split('\r\n')
        if any(mac.lower() in s.lower() for s in status):
            self._logger.info("mac address already in mac address acl list")
            return

        self.get_logger().info("add %s to acl" % mac)
        self._send_cmd("configure terminal", 3)
        try:
            self._send_cmd("access-list 777 deny %s 0000.0000.0000" % mac)
            # Default Action is "Forward All"
            self._send_cmd("access-list 777 permit 0000.0000.0000 ffff.ffff.ffff")
        except:
            self._send_cmd("end")
            msg = 'set config "access-list 777 deny %s 0000.0000.0000" failed' % mac
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        try:
            for radio in self.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("bridge-group 1 input-address-list 777")
                self._send_cmd("exit")
        except:
            msg = 'set config "bridge-group 1 input-address-list 777" failed'
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        finally:
            self._send_cmd("end")  # exit configure

        # control that the mac address is in the acl list
        self.get_handle().write("show access-lists\n")
        status = self.get_handle().read_until(self._ap_prompt, 5).strip().split('\r\n')
        if not any(mac.lower() in s.lower() for s in status):
            msg = "Unable to add mac address to acl list"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def del_mac_address_from_acl(self, mac_addr):
        """
        Remove mac address from AP access control list

        :type mac_addr: str
        :param mac_addr: can be "08:00:28:xx:xx:xx" where x is between [0-F]
        """
        # Convert Mac Address to Cisco's format
        mac_addr = str(mac_addr).lower()
        mac = self._convert_mac_format_to_cisco(mac_addr)

        # Check if mac address is already in the list
        self.get_handle().write("show access-lists\n")
        status = self.get_handle().read_until(self._ap_prompt, 5).strip().split('\r\n')
        if not any(mac.lower() in s.lower() for s in status):
            self._logger.info("mac address is not present in acl list")
            return

        self.get_logger().info("removing ACL ID 777 which contains all mac addresses added before")

        self._send_cmd("configure terminal", 3)

        try:
            for radio in self.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio " + str(radio))
                self._send_cmd("no bridge-group 1 input-address-list")
                self._send_cmd("exit")
        except:
            self._send_cmd("end")
            msg = 'set config "no bridge-group 1 input-address-list" failed'
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        try:
            # remove the access list id 777
            self._send_cmd("no access-list 777")
        except:
            msg = 'set configuration "no access-list 777" failed'
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        finally:
            self._send_cmd("end")  # exit configure

    def _convert_mac_format_to_cisco(self, mac_addr):
        """
        Converts a Mac address with format xx:xx:xx:xx:xx:xx to Cisco's xxxx.xxxx.xxxx format

        :type mac_addr: str
        :param mac_addr: for instance "08:00:28:xx:xx:xx" where x is between [0-F]

        :rtype: str
        :return: MAC address using Cisco format
        """
        mac_addr = mac_addr.strip()
        # Checks if the MAC address is already in the Cisco format
        if re.match("^([0-9A-Fa-f]{4}\.){2}[0-9A-Fa-f]{4}$", mac_addr) is not None:
            return mac_addr

        # Checks the format of the mac_addr parameter
        if not is_valid_mac_address(mac_addr):
            msg = "Parameter %s is not a valid mac address" % mac_addr
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        mac = ""
        items = mac_addr.split(":")
        loop_count = 0
        for item in items:
            loop_count += 1
            mac += item
            if loop_count % 2 == 0:
                mac += "."
        return mac[:-1]

    def _convert_mac_format_to_std(self, cisco_format_mac):
        """
        Converts a Mac address with Cisco's xxxx.xxxx.xxxx format to std format xx:xx:xx:xx:xx:xx

        :type cisco_format_mac: str
        :param cisco_format_mac: for instance "0800.28xx.xxxx" where x is between [0-F]

        :rtype: str
        :return: MAC address using standard format (in lower case)
        """
        # Checks if the MAC address is already in the standard format
        if is_valid_mac_address(cisco_format_mac):
            return cisco_format_mac

        # Checks if the MAC address is well in the Cisco format
        cisco_format_mac = cisco_format_mac.strip()
        if re.match("^([0-9A-Fa-f]{4}\.){2}[0-9A-Fa-f]{4}$", cisco_format_mac) is None:
            msg = "MAC address is not in Cisco format: " + cisco_format_mac
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        std_format_mac = cisco_format_mac[:2] + ':'
        std_format_mac += cisco_format_mac[2:4] + ':'
        std_format_mac += cisco_format_mac[5:7] + ':'
        std_format_mac += cisco_format_mac[7:9] + ':'
        std_format_mac += cisco_format_mac[10:12] + ':'
        std_format_mac += cisco_format_mac[12:14]

        return std_format_mac.lower()

    def set_dhcp(self, dhcp_status,
                 low_excluded_ip=None,
                 high_excluded_ip=None,
                 subnet=None,
                 subnet_mask=None,
                 lease=None,
                 default_gateway_address=None):
        """
        set dhcp server on the Configurable AP

        :type dhcp_status: str or int
        :param dhcp_status: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :type low_excluded_ip: str
        :param low_excluded_ip: An ip address "x.x.x.x" corresponding to the lower end of the excluded ip range

        :type high_excluded_ip: str
        :param high_excluded_ip: An ip address "x.x.x.x" corresponding to the higher end of the excluded ip range

        :type subnet: str
        :param subnet: An ip address "x.x.x.x" corresponding to the subnet address to be masked

        :type subnet_mask: str
        :param subnet_mask: An ip mask "x.x.x.x"

        :type lease: str
        :param lease:  { days [ hours ] [ minutes ] | infinite }, only numbers and spaces or "infinite"

        :type default_gateway_address: str
        :param default_gateway_address: An ip address "x.x.x.x" corresponding to the router address
        """
        if dhcp_status in ("ON", "on", "1", 1):
            self._logger.info("Set dhcp status to on")
            dhcp_status = 1
            if not NetworkingUtil.is_valid_ipv4_address(low_excluded_ip):
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter low_excluded_ip is not valid !")
            if not NetworkingUtil.is_valid_ipv4_address(high_excluded_ip):
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter high_excluded_ip is not valid !")
            if not NetworkingUtil.is_valid_ipv4_address(subnet):
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter subnet is not valid !")
            if not NetworkingUtil.is_valid_ipv4_address(subnet_mask):
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter subnet_mask is not valid !")
            if not NetworkingUtil.is_valid_ipv4_address(default_gateway_address):
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter default_gateway_address is not valid !")
            if lease != "infinite" and re.match("\d+[ ]+\d+[ ]+\d+", lease) is None:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Parameter lease is not valid !")
        elif dhcp_status in ("OFF", "off", "0", 0):
            self._logger.info("Set dhcp status to off")
            dhcp_status = 0
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Parameter dhcp_status is not valid !")

        configuration_timer = float(self._bench_params.
                                    get_param_value("ConfigurationWaitingTime"))

        if dhcp_status == 0:
            self.get_logger().info("stop dhcp server")
            # Check if an excluded address parameter exists
            self.get_handle().write("show running-config\n")
            status = self.get_handle().read_until(self._ap_prompt, 10).strip().split('\r\n')
            matching_exl_addr = [s for s in status if "ip dhcp excluded-address" in s]
            matching_pool = [s for s in status if "ip dhcp pool" in s]

            for exl_addr in matching_exl_addr:
                # remove excluded address
                self._send_cmd("configure terminal", 3)
                try:
                    self._send_cmd("no %s" % exl_addr)
                except:
                    msg = "Failed to remove excluded address"
                    self._logger.error(msg)
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
                finally:
                    self._send_cmd("end")  # exit configure

            # remove dhcp
            for existing_pool in matching_pool:
                self._send_cmd("configure terminal", 3)
                try:
                    self._send_cmd("no %s" % existing_pool)
                except:
                    msg = "Failed to stop dhcp service"
                    self._logger.error(msg)
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
                finally:
                    self._send_cmd("end")  # exit configure
            time.sleep(configuration_timer)

        if dhcp_status == 1:
            self.get_logger().info("start dhcp server")
            self._send_cmd("configure terminal", 3)
            try:
                self._send_cmd("ip dhcp excluded-address %s %s" % (low_excluded_ip, high_excluded_ip))
                self._send_cmd("ip dhcp pool %s" % Cisco1250.DHCP_POOL_NAME)
                self._send_cmd("network %s %s" % (subnet, subnet_mask))
                self._send_cmd("lease %s" % lease)
                self._send_cmd("default-router %s" % default_gateway_address)
            except:
                msg = "Failed to start dhcp service"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            finally:
                self._send_cmd("end")  # exit configure
                time.sleep(configuration_timer)

    def is_regulatorydomain_configurable(self):
        """
        Function used to know if it is possible to configure the Regulatory
        Domain for this Configuable Access Point
        If True, the function set_regulatorydomain() must be implemented
        If False, the function get_regulatorydomain() should be implemented

        :rtype: boolean
        :return: True if the Regulatory Domain is configurable
        """
        return False

    def is_regulatorydomain_sent(self):
        """
        Function used to know if the Access Point automatically sends the
        Regulatory Domain to their clients.

        :rtype: boolean
        :return: True if the AP automatically sends the regulatory domain
        """
        return True

    def _enable_regulatorydomain_sent(self):
        """
        Set the world mode in order to send, in the beacon,
        the current regularity domain
        """
        reg_domain = self.get_regulatorydomain()

        self._logger.info("Set AP world-mode to %s" % reg_domain)

        self._send_cmd("configure terminal", 3)
        try:
            for radio in self.WIFI_RADIOS.values():
                self._send_cmd("interface dot11radio %s" % str(radio))
                self._send_cmd("world-mode dot11d country-code %s both"
                               % str(reg_domain), 4)
                self._send_cmd("exit")
        finally:
            self._send_cmd("end")  # exit configure

    def simulate_dfs(self, from_freq, to_freq, nbr_beacon_to_wait, dfs_mode="enable"):
        """
        Set AP DFS simulation from a channel to another

        :type from_freq: str
        :param from_freq: The current channel you want to switch from

        :type to_freq: str
        :param to_freq: The channel you want to switch to

        :type nbr_beacon_to_wait: str
        :param nbr_beacon_to_wait: The delay in number of beacon before do the switch

        :type dfs_mode: str
        :param dfs_mode: the dfs mode set ap into
        """
        self._logger.info("Set AP DFS simulation from channel %s to channel %s"
                          % (from_freq, to_freq))

        self._send_cmd("configure terminal", 3)
        try:
            self._send_cmd("interface dot11radio 1")
            self._send_cmd("DFS channel %s" % from_freq)
        finally:
            self._send_cmd("end")
            self._send_cmd("debug dot11 dot11Radio 1 dfs simulate %s %s"
                           % (nbr_beacon_to_wait, to_freq))

    def get_regulatorydomain(self):
        """
        Return current regulatory domain for the Access Point

        Note for Cisco1250: The first time this function is called,
        the telnet connection to the AP should be activated

        :rtype: String
        :return: The current regulatory domain set
        """
        self.get_logger().info("Get the regulatory domain")

        if self._regularitydomain is None:
            if self._standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
                out = self._send_cmd("show controllers Dot11Radio %s"
                                     % Cisco1250.WIFI_RADIOS["5G"], 0)
            else:
                out = self._send_cmd("show controllers Dot11Radio %s"
                                     % Cisco1250.WIFI_RADIOS["2.4G"], 0)

            rd = None
            # Parse the output to retrieve the regulatory domain
            for line in out:
                match = re.search(r'Carrier Set:.*(\(-[A-Za-z]\))', line)
                if match is not None and match.group(1) is not None:
                    rd = match.group(1)
                    if rd not in Cisco1250.RD:
                        msg = "Unknown regularity domain read from AP: %s" % rd
                        self._logger.error(msg)
                        raise TestEquipmentException(
                            TestEquipmentException.READ_PARAMETER_ERROR, msg)
                    # Get the regulatory domain as an ISO country code
                    rd = Cisco1250.RD[rd]
                    break
            del out
            if rd is None:
                msg = "Unable to read regularity domain from AP"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)

            if self._standard is None:
                # Do not save the read regulatory domain, \
                # in case of standard has not already been set
                return rd
            self._regularitydomain = rd

        return self._regularitydomain

    def get_beacon_period(self):
        """
        Return the latest current beacon period set
        :rtype: String
        :return: The latest current beacon period set
        """
        return self._beacon_period

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF"

    def get_client_macaddr_list(self):
        """
        Get the Client MAC addresses list
        The MAC Addresses returned are in lower case.
        The format of MAC addresses is: xx:xx:xx:xx:xx:xx where x is 0-9 or a-f.

        :rtype: list
        :return: list of MAC addresses of the clients connected to the Access Point
        """
        self.get_logger().info("Get Client MAC addresses list")

        local_connection = False

        if self.get_handle() is None:
            # Connect to the AP if no connection ongoing
            local_connection = True
            self.init()

        try:
            # Run the command
            out = self._send_cmd("show dot11 association", 0)
            self._logger.debug("output=" + str(out))

            # Find all MAC addresses from 'out' list of lines
            client_list = re.findall(r'(?:[0-9a-fA-F]{4}\.){2}[0-9a-fA-F]{4}', str(out))

            # format the MAC addresses strings
            for index in range(len(client_list)):
                client_list[index] = self._convert_mac_format_to_std(client_list[index])
            self._logger.debug("client list=" + str(client_list))

        finally:
            if local_connection:
                # Disconnection from AP
                self.release()

        return client_list

    def get_selected_channel(self, standard=None):
        """
        Get the selected WiFi channel

        :type standard: str
        :param standard: the standard used on the AP.

        :rtype: int
        :return: the selected channel number
        """
        self.get_logger().info("Get selected Wifi channel")
        channel = -1
        start = time.time()
        timeout = 120

        while (channel == -1 or channel == 0) and time.time() < start + timeout:
            if self._standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
                out = self._send_cmd("show controllers Dot11Radio %s" % Cisco1250.WIFI_RADIOS["5G"], 0)
            else:
                out = self._send_cmd("show controllers Dot11Radio %s" % Cisco1250.WIFI_RADIOS["2.4G"], 0)

            # Parse the output to retrieve the regulatory domain
            for line in out:
                match = re.findall(r'Configured Frequency.*Channel ([0-9]+)', line)

                if len(match) > 0:
                    channel = int(match[0])
                    break

            if channel == -1 or channel == 0:
                self._logger.debug("No channel found (%d). Waiting 2 seconds..." % channel)
                time.sleep(2)

        if channel == -1 or channel == 0:
            msg = "Unable to retrieve Channel (%d) used by AccessPoint." % channel
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._logger.debug("Selected channel is : %d" % channel)
        return channel
