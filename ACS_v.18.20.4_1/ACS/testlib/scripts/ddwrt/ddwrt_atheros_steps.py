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
    def __init__(self, ssh_host = "192.168.1.1", ssh_user = "root", ssh_pwd = None, state = "up", **kwargs):
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_pwd = ssh_pwd
        self.state = state
        local_step.__init__(self, **kwargs)

    def do(self):
        local_steps.ssh_command(command = "ifconfig ath0 " + self.state, ssh_host = self.ssh_host, ssh_user = self.ssh_user, ssh_pwd = self.ssh_pwd)()
        if self.state == "up":
            reload_ap(self.ssh_host, self.ssh_user, self.ssh_pwd)()

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
                 radius_secret = None, hidden_ssid = "0", new_ssid = None, ssh_pwd = None, interface5ghz = '0', **kwargs):
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
        base_step.__init__(self, **kwargs)


    def do(self):

        if not local_steps.wait_for_ping(timeout = 30, ip = self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        commands = ddwrt_utils.generate_virtap_config_atheros(self.enable, self.security,
                        self.encryption, self.wifi_password, self.radius_ip,
                        self.radius_secret, self.hidden_ssid, self.new_ssid, self.interface5ghz)

        for command_group in ddwrt_utils.split_config(commands):
            local_steps.ssh_command(command = command_group,
                                    ssh_host = self.ssh_host,
                                    ssh_user = self.ssh_user,
                                    ssh_pass = self.ssh_pwd)()

    def check_condition(self):
        return reload_ap(self.ssh_host, self.ssh_user, self.ssh_pwd)()


class setup(base_step):

    def __init__(self, mode, security, ssh_host="192.168.1.1", ssh_user="root",
                 encryption=None, wifi_password=None, radius_ip=None,
                 radius_secret=None, hidden_ssid="0", new_ssid=None, channel_bw="20",
                 channel_no=None, ssh_pwd=None, interface5ghz='0',
                 ipv6_enable=0, radvd_enable=0, dhcp_lease=None, **kwargs):
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
        base_step.__init__(self, **kwargs)


    def do(self):

        if not local_steps.wait_for_ping(timeout = 30, ip = self.ssh_host):
            self.set_errorm("", "AP did not respond to ping")
            return False

        commands = ddwrt_utils.generate_ap_config_atheros(self.mode, self.security,
                                                          encryption=self.encryption, wifi_password=self.wifi_password,
                                                          radius_ip=self.radius_ip, radius_secret=self.radius_secret,
                                                          hidden_ssid=self.hidden_ssid, new_ssid=self.new_ssid,
                                                          channel_bw=self.channel_bw, channel_no=self.channel_no,
                                                          interface5ghz=self.interface5ghz, ipv6_enable=self.ipv6_enable,
                                                          radvd_enable=self.radvd_enable, dhcp_lease=self.dhcp_lease)

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
            ONLY WORKS ON BROADCOM AP
    """
    def __init__(self, ssh_host="192.168.1.1", ssh_user="root", **kwargs):
        base_step.__init__(self, **kwargs)

    def do(self):
        raise Exception("IPv6 steps can be used on Broadcom AP only")

    def check_condition(self):
        pass


class set_ipv6(base_step):
    """ description:
            Sends a conf file to the ddwrt router and starts the radvd daemon with that file as input
            ONLY WORKS ON BROADCOM AP
    """
    def __init__(self, **kwargs):
        base_step.__init__(self, **kwargs)

    def do(self):
        raise Exception("IPv6 steps can be used on Broadcom AP only")

    def check_condition(self):
        pass
