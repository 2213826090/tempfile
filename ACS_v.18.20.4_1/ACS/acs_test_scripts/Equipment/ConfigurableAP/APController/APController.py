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
:summary: implementation of APController used on Conformance bench.
        APController is able to control a set of AP
        (Broadcom, Atheros, Ralink11n, Marvell)
:since:22/11/2012
:author: apairex RTC32431
"""

from Core.FileParsingManager import FileParsingManager
from acs_test_scripts.Equipment.ConfigurableAP.Interface.IConfigurableAP import IConfigurableAP
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
import UtilitiesFWK.Utilities as Util
import telnetlib
import time


class APController(EquipmentBase, IConfigurableAP):

    """
    Implementation of APController which is use to configure a set of
    configurable APs.
    """

    WIFI_CHANNELS_5G = ['0', '36', '40', '44', '48', '52', '56', '60', '64',
                        '100', '104', '108', '112', '116', '120', '124',
                        '128', '132', '136', '140']

    _MODES = {"a": "11a",
              "b": "11b",
              "g": "11g",
              "an": "11na",
              "na": "11na",
              "bg": "11g",
              "gb": "11g",
              "gn": "11ng",
              "ng": "11ng",
              "bgn": "11ng",
              "gbn": "11ng",
              "nbg": "11ng",
              "ngb": "11ng",
              "n2.4G": "11ng",
              "n5G": "11na"}

    _SECURITIES = {"OPEN": "ACS-OPEN",
                   "NONE": "ACS-OPEN",
                   "WEP64": "ACS-WEP-SHARED",
                   "WEP64-OPEN": "ACS-WEP-OPEN",
                   "WEP128": "ACS-WEP-SHARED",
                   "WEP128-OPEN": "ACS-WEP-OPEN",
                   "WPA2-PSK-AES": "ACS-WPA2-PSK-AES",
                   "WPA-PSK-TKIP": "ACS-WPA-PSK-TKIP",
                   "WPA2-PSK-TKIP": "ACS-WPA2-PSK-TKIP",
                   "WPA-PSK-AES": "ACS-WPA-PSK-AES",
                   "WPA-PSK": "ACS-WPA-PSK-TKIP-AES",
                   "WPA-PSK-TKIP-AES": "ACS-WPA-PSK-TKIP-AES",
                   "WPA2-PSK": "ACS-WPA2-PSK-TKIP-AES",
                   "WPA2-PSK-TKIP-AES": "ACS-WPA2-PSK-TKIP-AES",
                   "WPA-WPA2-PSK": "ACS-WPA-WPA2-PSK-TKIP-AES",
                   "WPA-WPA2-PSK-TKIP-AES": "ACS-WPA-WPA2-PSK-TKIP-AES",
                   "WPA-WPA2-PSK-TKIP": "ACS-WPA-WPA2-PSK-TKIP",
                   "WPA-WPA2-PSK-AES": "ACS-WPA-WPA2-PSK-AES",
                   "EAP-WPA2": "ACS-WPA2-ENT-AES",
                   "EAP-WPA": "ACS-WPA-ENT-TKIP",
                   "EAP-WPA-WPA2": "ACS-WPA-WPA2-ENT-TKIP-AES"}

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        IConfigurableAP.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self._bench_params = bench_params
        self.__handle = None

        # Extract time to wait for configuration of the equipment
        self._configuration_timer = float(bench_params.
                                          get_param_value("ConfigurationWaitingTime"))

        # Retrieve parameters from BenchConfig for connection
        self._host = str(bench_params.get_param_value("IP"))
        self._username = str(bench_params.get_param_value("username"))
        self._password = str(bench_params.get_param_value("password"))

        # Retrieve the parameter of the AP to control through the AP controller
        self._ap_name = str(bench_params.get_param_value("APC_AccessPoint"))
        self._ap_ip = str(bench_params.get_param_value("APC_IP"))
        self._ap_user = str(bench_params.get_param_value("APC_UserName"))
        self._ap_pass = str(bench_params.get_param_value("APC_Password"))
        self._ap_serial_ip = str(bench_params.
                                 get_param_value("APC_SerialPortIP"))
        self._ap_hostname = str(bench_params.get_param_value("APC_HostName"))
        self._ap_power_ip = str(bench_params.
                                get_param_value("APC_PowerSwitchIPAddress"))
        self._ap_power_hostname = str(bench_params.
                                      get_param_value("APC_PowerSwitchHostname"))
        self._ap_power_username = str(bench_params.
                                      get_param_value("APC_PowerSwitchUsername"))
        self._ap_power_password = str(bench_params.
                                      get_param_value("APC_PowerSwitchPassword"))
        self._ap_power_port = str(bench_params.
                                  get_param_value("APC_PowerSwitchPort"))
        self._ap_power_port = self._ap_power_port.strip()

        # We consider that the power switch is opened by default
        self._ap_powered_on = False
        self._radio_status = "undef"

        self._channel = None
        self._standard = None
        self._crda = None

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
            return

        # Open telnet session
        connection_attempts = 5
        while connection_attempts > 0:
            try:
                self.__connect_via_telnet(self._host,
                                          self._username,
                                          self._password)
                break
            except TestEquipmentException as e:
                self._logger.info(e)
                connection_attempts -= 1
                if connection_attempts <= 0:
                    raise e
                self.release()

        # Register the AP name
        self._send_cmd(("AccessPoint,%s,IPAddress,%s," +
                       "UserName,%s,Password,%s,SerialPortIP,%s," +
                       "HostName,%s,PowerSwitchPort,%s")
                       % (self._ap_name,
                          self._ap_ip,
                          self._ap_user,
                          self._ap_pass,
                          self._ap_serial_ip,
                          self._ap_hostname,
                          self._ap_power_port))

        # Register the Power switch controller
        self._send_cmd("PowerSwitch,PowerSwitch," +
                       "IPAddress,%s,UserName,%s,Password,%s,HostName,%s"
                       % (self._ap_power_ip,
                          self._ap_power_username,
                          self._ap_power_password,
                          self._ap_power_hostname))

        # Power ON access point
        self._power_on()

    def release(self):
        """
        Release the equipment and all associated resources
        """
        self.get_logger().info("Release")

        self.__disconnect_via_telnet()

    def __connect_via_telnet(self, host, username="none", password="none"):
        """
        connect AP Controller via telnet thus config the equipment

        :type host: str
        :param host: access point IP address and port (ex: 192.168.0.10:7000)

        :type username: str
        :param username: access point admin login

        :type password: str
        :param password: access point admin password

        :raise: TestEquipmentException
        """
        self.get_logger().debug("Open telnet connection to equipment.")

        # get the port from the host str:
        host = str(host)
        if ":" in host:
            part = host.split(":")
            host = part[0]
            port = part[1]
        else:
            # default port # is 7000
            msg = "Use port 7000 as default port to connect to AP Controller"
            self.get_logger().debug(msg)
            port = 7000

        # Initialize telnet session
        telnet_session = telnetlib.Telnet()

        try:
            telnet_session.open(host, port)
            if username != "none" and password != "none":
                telnet_session.read_until("Username:", 5)
                telnet_session.write(str(username) + "\n")
                telnet_session.read_until("Password:", 5)
                telnet_session.write(str(password) + "\n")
        except:
            msg = "Connection via telnet failed."
            raise TestEquipmentException(TestEquipmentException.TELNET_ERROR, msg)

        # Update handle value
        self._set_handle(telnet_session)

    def __disconnect_via_telnet(self):
        """
        disconnect equipment from telnet
        """
        self.get_logger().debug("Close telnet connection from the equipment.")
        if self.get_handle() is not None:
            self.get_handle().close()
            self._set_handle(None)

    def _send_cmd(self,
                  cmd,
                  lines=0,
                  reply='status,COMPLETE',
                  timeout=(-1)):
        """
        Send command to the equipment

        :type cmd: str
        :param cmd: Command to send
        :type lines: integer
        :param lines: max number of lines to be read, 0 to ignore (default)
        :type reply: str
        :param reply: reply str to trigger
        :type timeout: int
        :param timeout: time to wait for the reply message, default value is
                calculate from the number of parameters in the cmd
        :rtype: str
        :return: The output str returned after the command
        """
        self.get_logger().debug("Send command # %s" % str(cmd))

        if timeout == -1:
            # 15sec timeout per ',' found in the cmd
            timeout = 15 * cmd.count(',')
            if timeout <= 0:
                timeout = 30

        self.get_handle().write(str(cmd) + "\n")
        ret = self.get_handle().read_until(reply, timeout).strip()
        ret += self.get_handle().read_until("\n", 3).strip()
        ret = ret.split('\r\n')

        if len(ret) == 0 or reply not in ret[-1]:
            msg = "Command timeout. > %s" % str(ret)
            self.get_logger().error("cmd # %s" % str(cmd))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

        if "status,COMPLETE,ERROR" in ret[-1]:
            msg = "Telnet cmd fails:" + str(ret)
            self.get_logger().error("cmd # %s" % str(cmd))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        else:
            self.get_logger().debug("APController returns < %s" % ret[-1])

        if 0 < lines < len(ret):
            ret[0] = ' ' * len(ret[-1].strip()) + ret[0].strip()
            error_msg = "Command error: \n" + '\n'.join(ret[:-1])
            self.get_logger().error("cmd # %s" % str(cmd))
            self.get_logger().error(error_msg)
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
        :param standard_type: wifi standard type. Available values are the keys
                            of APController._MODES.

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
        :param radiusip: Address of the radius server (optional)

        :type radiusport: str
        :param radiusport: port to connect to the radius server (optional)

        :type radiussecret: str
        :param radiussecret: Password to communicate between AP
                            and Radius server (optional)
        :type configindex: integer
        :param configindex: Configuration index - can be 1 to 3 -
                            0 is the default SSID data (optional)
        """
        self.get_logger().info("Set wifi configuration")

        if channel in [None, 0]:
            # Autochannel selection not yet available
            self._logger.warning("Auto Channel is not yet functional. " +
                                 "Set channel to the 1st one available.")
            channel = 1

        # Convert the standard into APController mode
        if standard_type not in APController._MODES:
            msg = "Unknown standard for this AP: %s" % standard_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        mode = APController._MODES[standard_type]

        # Convert the authentication_type into APController key_management
        if authentication_type not in APController._SECURITIES.keys():
            msg = "Unknown authentication: %s" % authentication_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        key_mgnt = APController._SECURITIES[authentication_type]

        # Create the SSID
        self._channel = int(channel)
        self._standard = mode
        cmd = "ap_set_wireless,NAME,%s,CHANNEL,%d,SSID,%s,MODE,%s" \
            % (self._ap_name, self._channel, ssid, mode)
        if self._crda is not None:
            # If set_regulatorydomain has been called, set the CRDA in the same time
            cmd += ",COUNTRY,%s,CHANNELNOCC,%d,width,20" % (self._crda, self._channel)
        self._send_cmd(cmd)

        # Configure the security
        cmd = "ap_set_security,NAME,%s,KEYMGNT,%s" % (self._ap_name, key_mgnt)
        if "PSK" in key_mgnt:
            cmd += ",PSK,%s" % passphrase
        elif "WEP" in key_mgnt:
            cmd += ",WEPKey,%s" % passphrase
        elif "-ENT" in key_mgnt:
            self._send_cmd(("ap_set_radius,NAME,%s," +
                            "IPADDR,%s,PORT,%s,PASSWORD,%s")
                           % (self._ap_name, radiusip, radiusport, radiussecret))

        self._send_cmd(cmd)

        # Send commit to the AP controller
        self._send_cmd("ap_config_commit,NAME,%s" % self._ap_name,
                       timeout=100)

    def enable_wireless(self):
        """
        enable wireless network

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Enable wireless")
        status = True
        if not self._ap_powered_on:
            self._power_on()

        if self._channel is None or self._standard is None:
            msg = "Cannot disable wireless before setting the Wifi config."
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        if self._radio_status != "ON":
            cmd = "ap_set_wireless,NAME,%s,CHANNEL,%d,MODE,%s,RADIO,on" \
                % (self._ap_name, self._channel, self._standard)
            if self._crda is not None:
                # If set_regulatorydomain has been called, set the CRDA in the same time
                cmd += ",COUNTRY,%s,CHANNELNOCC,%d,width,20" % (self._crda, self._channel)
            try:
                self._send_cmd(cmd)
                self._radio_status = "ON"
                time.sleep(self._configuration_timer)
            except TestEquipmentException:
                # EnableWireless should not stop the TC.
                # If the radio was already ON, the test is able to continue
                self._logger.error("Enable wireless fails")
                status = False
        else:
            self._logger.info("Wireless already enabled")
        return status

    def disable_wireless(self):
        """
        disable wireless network

        :rtype: Boolean
        :return: True in case of success, False in case of error
        """
        self.get_logger().info("Disable wireless")
        status = True

        if not self.is_radio_off():

            if self._channel is None or self._standard is None:
                msg = "You should run set_wifi_config before disable wireless."
                msg += " disable both radios"
                self._logger.warning(msg)
                cmd = "ap_set_wireless,NAME,%s,CHANNEL,%d,MODE,%s,RADIO,off" \
                    % (self._ap_name, 1, "11g")
                cmd2 = "ap_set_wireless,NAME,%s,CHANNEL,%d,MODE,%s,RADIO,off" \
                    % (self._ap_name, 36, "11na")
            else:
                cmd = "ap_set_wireless,NAME,%s,CHANNEL,%d,MODE,%s,RADIO,off" \
                    % (self._ap_name, self._channel, self._standard)
                cmd2 = ""
            try:
                self._send_cmd(cmd)
                if cmd2 != "":
                    self._send_cmd(cmd2)
                self._radio_status = "OFF"
                time.sleep(self._configuration_timer)
            except TestEquipmentException:
                # DisableWireless should not stop the TC.
                # If the radio was already ON, the test is able to continue
                self._logger.error("Disable wireless fails")
                status = False
        else:
            self._logger.info("Wireless already disabled")
        return status

    def is_regulatorydomain_configurable(self):
        """
        Function used to know if it is possible to configure the Regulatory
        Domain for this Configuable Access Point
        If True, the function set_regulatorydomain() must be implemented
        If False, the function get_regulatorydomain() should be implemented

        :rtype: boolean
        :return: True if the Regulatory Domain is configurable
        """
        return True

    def is_regulatorydomain_sent(self):
        """
        Function used to know if the Access Point automatically sends the
        Regulatory Domain to their clients.

        :rtype: boolean
        :return: True if the AP automatically sends the regulatory domain
        """
        return False

    def get_regulatorydomain(self):
        """
        Return current regulatory domain for the Access Point
        .. todo:: write code for this method

        :rtype: String
        :return: The current regulatory domain set
        """
        return None

    def set_regulatorydomain(self, reg_domain):
        """
        Set the regulatory domain to the Access Point

        :type reg_domain: String
        :param reg_domain: The current regulatory domain to set
        """
        self._crda = reg_domain

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

        set_dfs_channel_cmd = "ap_set_wireless,NAME,%s,dfs_chan,%s" % (self._ap_name, from_freq)
        set_dfs_mode_cmd = "ap_set_wireless,NAME,%s,dfs_mode,%s" % (self._ap_name, dfs_mode)
        tigger_dfs_cmd = "ap_set_wireless,NAME,%s,sim_dfs, %s" % (self._ap_name, to_freq)

        self._send_cmd(set_dfs_channel_cmd)
        self._send_cmd(set_dfs_mode_cmd)
        self._send_cmd(tigger_dfs_cmd)
        # Send commit to AP controller
        self._send_cmd("ap_config_commit,NAME,%s" % self._ap_name, timeout=100)

    def _power_on(self):
        """
        Use the power supply controller to power ON the Access Point that
        is linked to this APController
        This command takes between 15 and 30 seconds
        """
        self._logger.info("Power ON AP")
        if not self._ap_powered_on:
            self._send_cmd("power_switch_ctrl,name,PowerSwitch," +
                           "device,%s,state,On" % self._ap_power_port,
                           timeout=135)
            self._ap_powered_on = True
        else:
            self._logger.info("AP already powered ON")

    def _power_off(self):
        """
        Use the power supply controller to power ON the Access Point that
        is linked to this APController
        """
        self._logger.info("Power OFF AP")
        if self._ap_powered_on:
            self._send_cmd("power_switch_ctrl,name,PowerSwitch," +
                           "device,%s,state,Off" % self._ap_power_port)
            self._ap_powered_on = False
        else:
            self._logger.info("AP already powered OFF")

    def is_radio_off(self):
        """
        Tells if the radio is OFF

        :rtype: boolean
        :return: True if the radio is OFF
        """
        return self._radio_status == "OFF" and self._ap_powered_on

# Logger class for unit tests


class UnitTestLogger():

    """
    Class to stub Logger
    """

    def __init__(self):
        pass

    def debug(self, msg):
        """
        Debug function
        """
        print msg

    def info(self, msg):
        """
        Info function
        """
        print msg

    def error(self, msg):
        """
        error function
        """
        print msg

    def warning(self, msg):
        """
        warning function
        """
        print msg

# Unit tests


class SuperAPController(APController):

    """
    Super class in order to create set_logger accessor
    """

    def set_logger(self, logger):
        """
        Set logger accessor
        """
        self._logger = logger

if __name__ == "__main__":
    bench_config = "bench_config_conformance"
    equipment_catalog = "Equipment_Catalog"
    global_config = Util.Dictionary(attribute1='benchConfig',
                                    attribute2='deviceConfig',
                                    attribute3='campaignConfig',
                                    attribute4='equipmentCatalog',
                                    attribute5='campaignReportTree')
    global_config.benchConfig = {}
    global_config.deviceConfig = {}
    global_config.campaignConfig = {}
    global_config.equipmentCatalog = {}
    global_config.campaignReportTree = None
    flash_file_path = None

    file_parsing_manager = FileParsingManager(
        bench_config,
        equipment_catalog,
        global_config)
    file_parsing_manager.parse_bench_config()

    bench_name = "CONFIGURABLE_AP1"
    equipment_model = "AP_CONTROLLER"
    equipment_dict = {u'CISCO_WRVS4400N': {}, u'CISCO_AP541N': {},
                      u'DLINK_DAP2553': {}, u'CISCO_WAP4410N': {},
                      u'CISCO_1260': {}, u'CISCO_1250': {},
                      u'CONFIGURABLE_AP1': {}}

    ap = SuperAPController(bench_name, equipment_model, equipment_dict,
                           global_config['benchConfig'].get_parameters("CONFIGURABLE_AP1"))

    ap.set_logger(UnitTestLogger())

    ap.init()

    # Change following method parameters to test different configuration
    ap.set_wifi_config(ssid="UNIT_TEST7",
                       hidden=False,
                       standard_type="gn",
                       authentication_type="EAP-WPA",
                       passphrase=None,
                       radiusip="192.168.0.150",
                       radiusport="1812",
                       radiussecret="RadiusPass")
    ap.release()
