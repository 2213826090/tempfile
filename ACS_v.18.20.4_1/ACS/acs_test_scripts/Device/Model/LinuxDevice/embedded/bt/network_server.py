from subprocess import call, Popen, PIPE
from decoration import *
import dbus
from bt.constants import Constants as Consts
import logging
import os
import subprocess
import shlex
import fnmatch
import signal

class NetworkServer():
    """
    """

    NAP_UUID = "00001116-0000-1000-8000-00805f9b34fb"

    UDHCPD_CONF_FILE_PATH = "/etc/udhcpd.conf"

    NAP_BRIDGE_NAME = "EdBr0"
    NAP_BRIDGE_IP_ADDR = "192.168.10.1"
    NAP_SUB_NETWORK_START = "192.168.10.100"
    NAP_SUB_NETWORK_END = "192.168.10.110"

    DNS_SERVER_IP_ADDR = "8.8.8.8"

    DHCP_SERVER = "/usr/sbin/udhcpd"

    def __init__(self, adapter, *args, **kwargs):
        """
        """
        self._dbus_object = adapter.get_dbus_object()
        self._dbus_server_iface = dbus.Interface(adapter.get_dbus_object(), Consts.Bt.NETWORK_SRV_IFACE)
        self._log = self.set_logger()

    @staticmethod
    def set_logger():
        # Set logging
        # Log format
        log_fmt = '%(asctime)s   BtNetworkServer\t%(levelname)5s   %(message)s'
        # Log level => default log level is INFO
        log_lvl = logging.DEBUG
        log_file = "/tmp/bt_edison.log"
        # Retrieve logger
        logging.basicConfig(filename=log_file, level=log_lvl, format=log_fmt)
        log = logging.getLogger('BtNetworkServer')
        log.name = 'BtNetworkServer'
        return log

    def _create_dhcp_server_conf_file(self):
        """
        Create DHCP server configuration file
        """
        with open(self.UDHCPD_CONF_FILE_PATH, 'w') as fd:
            # Add IP start
            fd.write("start %s\n" % self.NAP_SUB_NETWORK_START)
            # Add IP end
            fd.write("end %s\n" % self.NAP_SUB_NETWORK_END)
            # Add server network interface
            fd.write("interface %s\n" % self.NAP_BRIDGE_NAME)
            # Add DNS server IP address
            fd.write("option dns %s\n" % self.DNS_SERVER_IP_ADDR)
            # Add default DHCP client gateway
            fd.write("option router %s\n" % self.NAP_BRIDGE_IP_ADDR)
            fd.write("option subnet 255.255.255.0")

    def _get_running_dhcp_server_pids(self):
        """
        Get PIDs of running DHCP servers.

        @rtype: list
        @return: PIDs list of running DHCP servers
        """
        p = Popen(['ps'], stdout=PIPE, stderr=PIPE)
        stdoutdata, stderrdata = p.communicate()

        pids = []
        data = fnmatch.filter(stdoutdata.split('\n'), "*%s*" % self.DHCP_SERVER)
        for line in data:
            pids.append(int(line.split()[0]))

        return pids

    def _start_dhcp_server(self):
        """
        Start DHCP server
        """
        # Kill running DHCP servers
        pids = self._get_running_dhcp_server_pids()
        if len(pids) > 0:
            self._stop_dhcp_servers(pids)

        # Run DHCP server
        subprocess.Popen([self.DHCP_SERVER])

    def _stop_dhcp_servers(self, pids=None):
        """
        Stop running DHCP servers
        @type pids: list
        @param param: PIDs list of running DHCP servers or None if list not yet built
        """
        # Get running DHCP servers pids list if needed
        if pids is None:
            pids = self._get_running_dhcp_server_pids()

        for pid in pids:
            os.kill(pid, signal.SIGTERM)

    @executor
    def register(self, *args, **kwargs):
        """
        Start NAP service
        """
        # Register PAN service
        try:
            self._log.debug("Register PAN service")
            self._dbus_server_iface.Register("nap", self.NAP_BRIDGE_NAME)
            self._log.debug("PAN service registered")
        except dbus.exceptions.DBusException as E:
            err_name = E.get_dbus_name()
            if err_name == "org.bluez.Error.AlreadyExists":
                self._log.debug(E)
            else:
                msg = E.get_dbus_message()
                self._log.error(E.get_dbus_message())
                return FAILURE, msg

        return SUCCESS, "NAP registered"

    @executor
    def unregister(self, *args, **kwargs):
        """
        Stop NAP service
        """
        try:
            # Unregister service
            self._dbus_server_iface.Unregister(NetworkServer.NAP_UUID)
            # Remove bridge
            call(["ip", "link", "set", self.NAP_BRIDGE_NAME, "down"])
            call(["brctl", "delbr", self.NAP_BRIDGE_NAME])
        except dbus.DBusException as E:
            msg = E.get_dbus_message()
            self._log.error(msg)
            return FAILURE, E.get_dbus_message()

        return SUCCESS, "NAP unregistered"

    def _create_bridge(self):
        """
        Create bridge for PAN only if needed
        """
        try:
            os.stat("/sys/devices/virtual/net/%s" % self.NAP_BRIDGE_NAME)
        except:
            # Bridge doesn't exist => create it
            call(["brctl", "addbr", "%s" % self.NAP_BRIDGE_NAME])
        finally:
            # Set bridge IP address
            call(["ip", "addr", "add", self.NAP_BRIDGE_IP_ADDR, "dev", self.NAP_BRIDGE_NAME])

    def _enable_network_bridge(self):
        """
        Enable network bridge
        """
        self._log.info("Enable network bridge")
        # Create bridge if needed
        self._create_bridge()
        # ifup bridge network interface
        call(["ip", "link", "set", self.NAP_BRIDGE_NAME, "up"])
        call(["ip", "route", "add", "192.168.10.0/24", "via", self.NAP_BRIDGE_IP_ADDR])
        self._create_dhcp_server_conf_file()
        self._start_dhcp_server()
        self.enable_ip_forwarding()
        return SUCCESS, "Tethering over PAN enabled"

    def _disable_network_bridge(self):
        """
        Disable network bridge
        """
        self._log.info("Disable network bridge")
        self._stop_dhcp_servers()
        # ifdown bridge network interface
        call(["ip", "link", "set", self.NAP_BRIDGE_NAME, "down"])
        return SUCCESS, "Tethering over PAN disabled"

    @executor
    def set_network_bridge(self, state, *args, **kwargs):
        """
        Set network bridge state
        @type state: str
        @param state: network bridge desired state
        """
        if state.lower() == 'on':
            return self._enable_network_bridge()
        elif state.lower() == 'off':
            return self._disable_network_bridge()
        else:
            msg = "bad state {0}".format(state)
            self._log.error("[NetworkServer.set_network_bridge]: {0}".format(msg))
            return FAILURE, msg

    @executor
    def get_network_bridge_state(self, *args, **kwargs):
        """
        Get network bridge state
        """
        self._log.info("Get network bridge state")
        cmd = ["ifconfig", self.NAP_BRIDGE_NAME]
        proc = Popen(cmd, stdout=PIPE, stderr=PIPE)
        stdoutdata, stderrdata = proc.communicate()
        result = "OFF"
        if "UP BROADCAST" in stdoutdata:
            # Bridge network interface is up
            result = "ON"

        self._log.info("Network bridge is %s" % result)
        return SUCCESS, result

    @executor
    def enable_ip_forwarding(self, *args, **kwargs):
        """
        Enable IP forwarding: needed after PAN user configuration
        """
        with open('/proc/sys/net/ipv4/ip_forward', 'w') as fd:
            fd.write("1")
        call(shlex.split("iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE"))
