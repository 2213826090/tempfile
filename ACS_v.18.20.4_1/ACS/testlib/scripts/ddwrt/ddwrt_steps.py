#!/usr/bin/env python

##############################################################################
#
# @filename:    ddwrt-steps.py
# @description: ddwrt router test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.scripts.ddwrt import ddwrt_utils
from testlib.utils.defaults.wifi_defaults import RADVD_CONF_FILE
import time
import os


class configure_ap_with_shell_scripts(base_step):

    """ description:
            sets a new configuration on the ddwrt router
                - <config> = the AP configuration needed to be set
                valid <config> = "mode", "security"
                - <value> = the AP value to be set for the given
                <config>
                valid <value> for "mode" = "b",
                                           "bg",
                                           "g",
                                           "mixed",
                                           "n",
                                           "ng"
                              for "security" = "none",
                                               "wep_128",
                                               "wep_64",
                                               "wpa2_eap",
                                               "wpa2_psk",
                                               "wpa_eap",
                                               "wpa_mixed_psk",
                                               "wpa_psk"

        usage:
            steps.configure_ap_with_shell_scripts(config = "mode",
                          value = "n",
                          ssh_host = "10.237.100.219",
                          ssh_user = "root",
                          media_path = /path/to/sh/files)()
            steps.configure_ap_with_shell_scripts(config = "security",
                          value = "wep_128",
                          ssh_host = "10.237.100.219",
                          ssh_user = "root",
                          media_path = /path/to/sh/files)()

        tags:
            ddwrt, config, mode, security
    """

    def __init__(self, config, value, ssh_host, ssh_user, ssh_pwd = None,
            **kwargs):
        self.config = config
        self.value = value
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        base_step.__init__(self, **kwargs)

    def do(self):
        sh_file = self.config + "_" + self.value + ".sh"
        sh_file_path =  self.media_path + "ap_" + self.config + "/" + sh_file
        local_steps.scp_command(local = sh_file_path,
                                remote = ".",
                                ssh_host = self.ssh_host,
                                ssh_user = self.ssh_user,
                                ssh_pass = self.ssh_pwd)()
        local_steps.ssh_command(command = "chmod +x " + sh_file,
                                ssh_host = self.ssh_host,
                                ssh_user = self.ssh_user,
                                ssh_pass = self.ssh_pwd)()
        local_steps.ssh_command(command = "./" + sh_file,
                                ssh_host = self.ssh_host,
                                ssh_user = self.ssh_user,
                                ssh_pass = self.ssh_pwd)()

    def check_connection(self):
        #TODO
        return True


class set_ap_wireless(local_step):
    """ description:
        Enables / disabled the wifi interface.
        Possible states:
            up --> enable
            down --> disable

        usage:
            wifi_steps.set_ap_wireless(state = "up")()

        tags:
            ui, android, settings, airplane
    """
    def __init__(self, interface5g='0',ssh_host = "192.168.1.1", ssh_user = "root", ssh_pwd = None, state = "up", **kwargs):
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        self.state = state
        self.interface5g = interface5g
        local_step.__init__(self, **kwargs)

    def do(self):
        #local_steps.ssh_command(command = "wl radio " + self.state, ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()
        #added code to disable/enable the wifi interface 
        """ please refer the link for disable and eanble the radio , https://www.dd-wrt.com/phpBB2/viewtopic.php?t=263538"""
        local_steps.ssh_command(command = "startservice radio_" + self.state+"_"+self.interface5g, ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()
        time.sleep(5) 
        local_steps.ssh_command(command = "stopservice radio_" + self.state+"_"+self.interface5g, ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()

    def check_condition(self):
        #TODO
        return True



class reboot(local_step):

    def __init__(self, ssh_host = "192.168.1.1" , ssh_user = "root", ssh_pwd = None , **kwargs):
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        local_step.__init__(self, **kwargs)

    def do(self):
        local_steps.ssh_command(command = "reboot", ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()

    def check_condition(self):
        #wait until AP starts rebooting (and stops responding to ping)
        waiting = 0
        timeout = 20
        while waiting < timeout:
            if not self.local_connection.check_ping(self.ssh_host):
                break
            else:
                time.sleep(2)
                waiting += 2
        else:
            self.set_errorm("", "AP did not stop running")
            return False
        #wait until AP finishes the reboot
        self.set_errorm("", "AP started rebooting")
        return local_steps.wait_for_ping(timeout = 35, ip = self.ssh_host)


class reload_ap(local_step):

    def __init__(self, ssh_host = "192.168.1.1" , ssh_user = "root", ssh_pwd = None , **kwargs):
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        local_step.__init__(self, **kwargs)

    def do(self):
        local_steps.ssh_command(command = "rc restart", ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()

    def check_condition(self):
        #wait until AP starts restarting services (and stops responding to ping)
        waiting = 0
        timeout = 30
        while waiting < timeout:
            if not self.local_connection.check_ping(self.ssh_host):
                break
            else:
                time.sleep(2)
                waiting += 2
        else:
            self.set_errorm("", "AP did not stop running")
            #return False
        #wait until AP finishes the reboot
        self.set_errorm("", "AP started restarting services")
        return local_steps.wait_for_ping(timeout = 35, ip = self.ssh_host)



class setup_virtual_interface(base_step):

    def __init__(self, enable, security=None, ssh_host = "192.168.1.1", ssh_user = "root",
                 encryption=None, wifi_password=None, radius_ip = None,
                 radius_secret = None, hidden_ssid = "0", new_ssid = None,
                 ssh_pwd = None, interface5ghz = "0", virt_if_idx = "1",
                 reload_ap = True,  **kwargs):
        self.enable = enable
        self.hidden_ssid = hidden_ssid
        self.new_ssid = new_ssid
        self.security = security
        self.encryption = encryption
        self.wifi_password = wifi_password
        self.radius_ip = radius_ip
        self.radius_secret = radius_secret
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        self.interface5ghz = interface5ghz
        self.virt_if_idx = virt_if_idx
        self.reload_ap = reload_ap
        base_step.__init__(self, **kwargs)


    def do(self):

        if not local_steps.wait_for_ping(timeout = 30, ip = self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        commands = ddwrt_utils.generate_virtap_config(self.enable, self.security,
                        self.encryption, self.wifi_password, self.radius_ip,
                        self.radius_secret, self.hidden_ssid, self.new_ssid,
                        self.interface5ghz, self.virt_if_idx, self.reload_ap)

        for command_group in ddwrt_utils.split_config(commands):
            local_steps.ssh_command(command = command_group,
                                    ssh_host = self.ssh_host,
                                    ssh_user = self.ssh_user,
                                    ssh_pass = self.ssh_pwd)()

    def check_condition(self):
        if self.reload_ap:
            return reload_ap(self.ssh_host, self.ssh_user, self.ssh_pwd)()
        else:
            return True

class setup(base_step):
    """ description:
            sets a new configuration on the ddwrt router
                - <mode> = the operation mode of the AP
                - <security> = the security mode of the AP
                Valid values for "mode" = "b",
                                           "bg",
                                           "g",
                                           "mixed",
                                           "n",
                                           "ng"
                             for "security" = "none",
                                               "wpa_psk_mixed",
                                               "wpa_psk",
                                               "wpa2",
                                               "wpa_enterprise",
                                               "wpa2_enterprise",
                                               "wep64",
                                               "wep128"
        usage:
            steps.setup(mode = "n",
                          security = "wpa2",
                          encryption = "tkip",
                          wifi_password = "PassPhrase",
                          ssh_host = "192.168.1.1",
                          ssh_user = "root")()


        tags:
            ddwrt, config, mode, security
    """
    def __init__(self, mode, security, ssh_host='192.168.1.1', ssh_user="root",
                 encryption=None, wifi_password=None, radius_ip=None,
                 radius_secret=None, hidden_ssid="0", new_ssid=None,
                 channel_bw="20", channel_no=None, ssh_pwd=None, interface5ghz='0',
                 ipv6_enable=0, radvd_enable=0, dhcp_lease=None, txpwr=None, ipv4_class='192.168.1.1', **kwargs):
        self.hidden_ssid = hidden_ssid
        self.new_ssid = new_ssid
        self.mode = mode
        self.security = security
        self.encryption = encryption
        self.wifi_password = wifi_password
        self.radius_ip = radius_ip
        self.radius_secret = radius_secret
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        self.channel_bw = channel_bw
        self.channel_no = channel_no
        self.interface5ghz = interface5ghz
        self.ipv6_enable = ipv6_enable
        self.radvd_enable = radvd_enable
        self.dhcp_lease = dhcp_lease
        self.txpwr = txpwr
        self.ipv4_class = ipv4_class
        base_step.__init__(self, **kwargs)

    def do(self):

        if not local_steps.wait_for_ping(timeout = 30, ip = self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        commands = ddwrt_utils.generate_ap_config(self.mode, self.security,
                                                  encryption=self.encryption, wifi_password=self.wifi_password,
                                                  radius_ip=self.radius_ip, radius_secret=self.radius_secret,
                                                  hidden_ssid=self.hidden_ssid, new_ssid=self.new_ssid,
                                                  channel_bw=self.channel_bw, channel_no=self.channel_no,
                                                  interface5ghz=self.interface5ghz, ipv6_enable=self.ipv6_enable,
                                                  radvd_enable=self.radvd_enable, dhcp_lease=self.dhcp_lease, txpwr=self.txpwr, ipv4_class=self.ipv4_class)

        for command_group in ddwrt_utils.split_config(commands):
            local_steps.ssh_command(command=command_group,
                                    ssh_host=self.ssh_host,
                                    ssh_user=self.ssh_user,
                                    ssh_pass=self.ssh_pwd)()

    def check_condition(self):
        return reload_ap(self.ssh_host, self.ssh_user, self.ssh_pwd)()


class radvd_enable(base_step):
    """ description:
            Sends a conf file to the ddwrt router and starts the radvd daemon with that file as input

        usage:
            steps.radvd_enable(radvd_conf_file = some.local.file,
                               remote_file = "radvd.conf",
                               ssh_host="192.168.1.1",
                               ssh_user="root")()


        tags:
            ddwrt, config, radvd
    """
    def __init__(self, ssh_host="192.168.1.1", ssh_user="root", **kwargs):
        self.radvd_conf_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), RADVD_CONF_FILE)
        self.remote_file = RADVD_CONF_FILE
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.step_data = None
        base_step.__init__(self, **kwargs)

    def do(self):

        if not local_steps.wait_for_ping(timeout=30, ip=self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        # Copy the radvd conf file to the ap
        local_steps.scp_command(local=self.radvd_conf_file, remote=self.remote_file,
                                ssh_host=self.ssh_host, ssh_user=self.ssh_user)()
        #kill if the process is running & restart with new radvd config
        kill_command = "killall radvd - C / tmp / radvd.conf"
        self.step_data = local_steps.ssh_command(command=kill_command, ssh_host=self.ssh_host, ssh_user=self.ssh_user)()

        # Enable the radvd daemon and check if the process is running
        command = "radvd -C {0}".format(self.remote_file)
        self.step_data = local_steps.ssh_command(command=command, ssh_host=self.ssh_host, ssh_user=self.ssh_user,
                                                 check_command="ps | grep radvd", check_command_output=command)()

    def check_condition(self):
        return self.step_data


class set_ipv6(base_step):
    """ description:
            Sets the ipv6 address on the test AP

        usage:
            steps.radvd_enable(ipv6_ip="2001:1234:5678:9abc::1",
                               ssh_host="192.168.1.1",
                               ssh_user="root")()


        tags:
            ddwrt, config, ipv6
    """
    def __init__(self, ipv6_ip, ssh_host="192.168.1.1", ssh_user="root", **kwargs):
        self.ipv6_ip = ipv6_ip
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.step_data = None
        base_step.__init__(self, **kwargs)

    def do(self):

        if not local_steps.wait_for_ping(timeout=30, ip=self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        command = "ip addr add {0} dev br0".format(self.ipv6_ip)
        self.step_data = local_steps.ssh_command(command=command, ssh_host=self.ssh_host, ssh_user=self.ssh_user,
                                                 check_command="ifconfig br0 | grep inet6",
                                                 check_command_output=self.ipv6_ip)

    def check_condition(self):
        return self.step_data
