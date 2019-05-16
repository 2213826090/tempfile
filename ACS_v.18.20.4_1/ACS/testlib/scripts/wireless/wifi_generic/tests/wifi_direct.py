#!/usr/bin/env python

######################################################################
#
# @filename:    wifi_direct.py
# @description: Implements WiFi direct test cases.
#
# @run example:
#
#    python wifi_direct.py -s 0BA8F2A0
#                                       --script-args
#                                           wifi_connected=True
#                                           wifi_traffic=True
#                                           amode=False
#                                           p2p_connected=True
#                                           p2p_traffic=False
#                                           scenario=CLI-Activate-AM
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.base.base_step import step as base_step
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.ParallelSteps import ParallelSteps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.ap import ap_steps

import sys
import time

from testlib.utils.statics.android import statics
import subprocess

class run_scenario(base_step):

    def __init__(self, **kwargs):
        base_step.__init__(self, **kwargs)
        self.available_states_to_config = {
                '1_am': self.config_am,
                '2_wifi_connected': self.config_wifi_connected,
                '3_p2p_connected': self.config_p2p_connected,
                '4_wifi_traffic': self.config_wifi_traffic,
                '5_p2p_traffic': self.config_p2p_traffic,}

        self.scenario_action = {"Owner-Turn-WiFi-OFF": self.turn_wifi_off,
                                "CLI-Turn-WiFi-OFF": self.turn_wifi_off,
                                "Owner-Disconnect-Wifi": self.disconnect_wifi,
                                "CLI-Disconnect-Wifi": self.disconnect_wifi,
                                "Owner-Deactivate-AM": self.deactivate_am,
                                "CLI-Deactivate-AM": self.deactivate_am,
                                "Owner-Activate-AM": self.activate_am,
                                "CLI-Activate-AM": self.activate_am,
                                "CLI-Leave-Group": self.leave_group,
                                "Owner-Dismiss-CLI": self.dismiss_cli,
                                "Owner-Remove-Group": self.remove_group,
                                "Owner-Generate-P2P-Traffic": self.generate_p2p_traffic,
                                "CLI-Generate-P2P-Traffic": self.generate_p2p_traffic,
                                "CLI-Connect-P2P": self.connect_p2p,
                                "CLI-Connect-WiFi": self.connect_wifi,
                                "Owner-Connect-WiFi": self.connect_wifi,
                                "Owner-Reject-Connect": self.reject_connect,
                                "Owner-Accept-Connect": self.accept_connect,
                                "CLI-Generate-WiFi-Traffic": self.generate_wifi_traffic,
                                "Owner-Generate-WiFi-Traffic": self.generate_wifi_traffic,
                                "CLI-Scan-WiFi": self.scan_wifi,
                                "Owner-Scan-WiFi": self.scan_wifi,
                                "CLI-Connect-timeout": self.connect_timeout,
                                "Owner-Connect-timeout": self.connect_timeout,
                                "CLI-Reconnect-Persistent-P2P": self.reconnect_persistent,
                                "Owner-Reconnect-Persistent-P2P": self.reconnect_persistent,
                                "Owner-P2P-Twoway-Traffic": self.config_p2p_twoway_traffic
                                }

        self.scenario = scenario

    def configure_scenario(self, scenario, am, wifi_connected, wifi_traffic, p2p_connected, p2p_traffic, serial, serial2):
        if "cli-" in scenario.lower():
            self.go_serial = serial2
            self.platform_go = ref_platform
            self.platform_cli = dut_platform
            self.cli_serial = serial
            self.ping_serial = self.cli_serial
            self.ip_ping = "go_ip"
        else:
            self.go_serial = serial
            self.platform_go = dut_platform
            self.platform_cli = ref_platform
            self.cli_serial = serial2
            self.ping_serial = self.go_serial
            self.ip_ping = "slave_ip"

        self.scenario = scenario

        if scenario in ["Owner-Turn-WiFi-OFF", "Owner-Disconnect-Wifi",
                        "Owner-Deactivate-AM", "Owner-Activate-AM",
                        "Owner-Dismiss-CLI", "Owner-Generate-P2P-Traffic",
                        "Owner-Connect-WiFi", "Owner-Reject-Connect",
                        "Owner-Accept-Connect", "Owner-Generate-WiFi-Traffic",
                        "Owner-Scan-WiFi", "Owner-Connect-timeout",
                        "Owner-Reconnect-Persistent-P2P",
                        "Owner-Remove-Group", "Owner-P2P-Twoway-Traffic"]:
            self.am_serial = self.go_serial
            self.action_serial = self.go_serial
            self.wifi_trafic_serial = self.go_serial

        elif scenario in ["CLI-Turn-WiFi-OFF", "CLI-Disconnect-Wifi",
                          "CLI-Deactivate-AM", "CLI-Activate-AM",
                          "CLI-Connect-timeout", "CLI-Leave-Group",
                          "CLI-Generate-P2P-Traffic", "CLI-Connect-P2P",
                          "CLI-Connect-WiFi", "CLI-Generate-WiFi-Traffic",
                          "CLI-Scan-WiFi", "CLI-Reconnect-Persistent-P2P",]:
            self.am_serial = self.cli_serial
            self.action_serial = self.cli_serial
            self.wifi_trafic_serial = self.cli_serial

        else:
            raise Exception("invalid scenario")

        self.states = {'1_am':am,
                       '2_wifi_connected':wifi_connected,
                       '3_p2p_connected':p2p_connected,
                       '4_wifi_traffic':wifi_traffic,
                       '5_p2p_traffic':p2p_traffic}

        # register final states(scenario)
        self.register_final_states(scenario, am, wifi_connected, wifi_traffic, p2p_connected, p2p_traffic)


    def register_final_states(self, scenario, am, wifi_connected, wifi_traffic, p2p_connected, p2p_traffic):
        if scenario in ["Owner-Turn-WiFi-OFF", "CLI-Turn-WiFi-OFF"]:
            self.final_states = {'1_am':am,
                              '2_wifi_connected':False,
                              '3_p2p_connected':False,
                              '4_wifi_traffic':False,
                              '5_p2p_traffic':False}
        elif scenario in ["Owner-Activate-AM", "CLI-Activate-AM"]:
            self.final_states = {'1_am':True,
                              '2_wifi_connected':False,
                              '3_p2p_connected':False,
                              '4_wifi_traffic':False,
                              '5_p2p_traffic':False}
        elif scenario in ["Owner-Disconnect-Wifi", "CLI-Disconnect-Wifi",
                          "Owner-Scan-WiFi", "CLI-Scan-WiFi"]:
                    self.final_states = {'1_am':am,
                              '2_wifi_connected':False,
                              '3_p2p_connected':p2p_connected,
                              '4_wifi_traffic':False,
                              '5_p2p_traffic':p2p_traffic}
        elif scenario in ["Owner-Deactivate-AM", "CLI-Deactivate-AM"]:
                    self.final_states = {'1_am':False,
                              '2_wifi_connected':wifi_connected,
                              '3_p2p_connected':p2p_connected,
                              '4_wifi_traffic':wifi_traffic,
                              '5_p2p_traffic':p2p_traffic}
        elif scenario in ["CLI-Connect-timeout", "CLI-Leave-Group",
                          "Owner-Dismiss-CLI", "Owner-Reject-Connect",
                          "Owner-Connect-timeout"]:
                    self.final_states = {'1_am':am,
                              '2_wifi_connected':wifi_connected,
                              '3_p2p_connected':p2p_connected,
                              '4_wifi_traffic':wifi_traffic,
                              '5_p2p_traffic':False}
        elif scenario in ["Owner-Generate-P2P-Traffic", "CLI-Generate-P2P-Traffic",
                          "Owner-Generate-WiFi-Traffic",
                          "CLI-Generate-WiFi-Traffic", "Owner-Remove-Group",
                          "Owner-P2P-Twoway-Traffic"]:
                    self.final_states = {'1_am':am,
                              '2_wifi_connected':wifi_connected,
                              '3_p2p_connected':True,
                              '4_wifi_traffic':wifi_traffic,
                              '5_p2p_traffic':p2p_traffic}
        elif scenario in ["CLI-Connect-P2P", "Owner-Accept-Connect",
                          "CLI-Reconnect-Persistent-P2P",
                          "Owner-Reconnect-Persistent-P2P"]:
                    self.final_states = {'1_am':am,
                              '2_wifi_connected':wifi_connected,
                              '3_p2p_connected':True,
                              '4_wifi_traffic':wifi_traffic,
                              '5_p2p_traffic':False}
        elif scenario in ["Owner-Connect-WiFi", "CLI-Connect-WiFi"]:
                    self.final_states = {'1_am':am,
                              '2_wifi_connected':True,
                              '3_p2p_connected':p2p_connected,
                              '4_wifi_traffic':False,
                              '5_p2p_traffic':p2p_traffic}


    def create_states(self):
        for key in sorted(self.states):
            if key in self.available_states_to_config:
                self.available_states_to_config[key](state=self.states[key])
            else:
                raise Exception("invalid config")


    def config_am(self, state):
        print "====================== config_am ======================"
        if state:
            wifi_generic_steps.set_airplane_mode(state = "ON", serial = self.am_serial)()
        else:
            wifi_generic_steps.set_airplane_mode(state = "OFF", serial = self.am_serial)()

    def config_wifi_connected(self, state, toggle_off = True):
        print "====================== config_wifi_connected ======================"
        # make sure there are no saved networks
        wifi_generic_steps.clear_saved_networks(serial = self.wifi_trafic_serial)()
        if toggle_off:
            wifi_generic_steps.set_wifi(state="OFF",serial = serial)()
            wifi_generic_steps.set_wifi(state="OFF",serial = serial2)()
        wifi_generic_steps.set_wifi(state="ON",serial = serial)()
        wifi_generic_steps.set_wifi(state="ON",serial = serial2)()

        if interface5ghz == "1":
            ap_steps.setup(mode="n", security = "wpa2",
                           wifi_password= attrs["passphrase"],
                           new_ssid= attrs["ap_name"] if str(attrs["new_ssid"]).lower() == "none" else attrs["new_ssid"],
                           interface5ghz=interface5ghz, encryption="aes",
                           channel_no=channel_no)()
        if state:
            # add the Wi-Fi network
            wifi_generic_steps.add_network(ssid = attrs["ap_name"] if str(attrs["new_ssid"]).lower() == "none" else attrs["new_ssid"],
                                           security = attrs["dut_security"],
                                           password = attrs["passphrase"],
                                           identity = attrs["radius_identity"],
                                           EAP_method = attrs["EAP_method"],
                                           phase_2_auth = attrs["phase_2_auth"],
                                           user_certificate = attrs["user_certificate"],
                                           serial = self.wifi_trafic_serial)()
            # wait until the device connects to a wifi network
            wifi_generic_steps.wait_until_connected(serial = self.wifi_trafic_serial)()


    def config_wifi_traffic(self, state):
        print "====================== config_wifi_traffic ======================"
        if state:
            # start download
            negative = True
            if self.final_states["4_wifi_traffic"]:
                negative = False
            if self.scenario in ["CLI-Connect-timeout", "Owner-Connect-timeout"]:
                attrs['trycount'] = 120
            device_ip = wifi_utils.get_host_ip(serial=self.ping_serial)
            self.step_id_wifi = psteps.add_step(wifi_steps.ping_ip,
                                      negative = negative,
                                      serial = self.ping_serial,
                                      trycount = attrs['trycount'],
                                      timeout = attrs['trycount'] + 20,
                                      transfer_type="wifi",
                                      target_percent = 10,
                                      ip = device_ip,
                                      )

    def preset_p2p(self):
        self.go_name = "go_" + self.go_serial
        self.slave_name = "cli_" + self.cli_serial

        wifi_generic_steps.p2p_disconect_all(serial = self.go_serial)()
        #wifi_generic_steps.p2p_disconect_all(serial = self.cli_serial)() #disconnect from one side only

        wifi_generic_steps.p2p_forget_all_groups(serial = self.go_serial)()
        wifi_generic_steps.p2p_forget_all_groups(serial = self.cli_serial)()


        wifi_generic_steps.p2p_rename_device(serial = self.go_serial, new_name = self.go_name)()
        wifi_generic_steps.p2p_rename_device(serial = self.cli_serial, new_name = self.slave_name)()

        #wifi_generic_steps.p2p_scan_devices(serial = self.go_serial )()
        #wifi_generic_steps.p2p_scan_devices(serial = self.cli_serial)()


    def config_p2p_connected(self, state, accept_connect=True, no_response=False):
        print "====================== config_p2p_connected ======================"

        if state:
            if not no_response:
                state_to_check = "CONNECTED/CONNECTED"
                if not accept_connect:
                    #state_to_check = "DISCONNECTED/DISCONNECTED"
                    state_to_check = "DISCONNECTED/"
            else:
                # connection states cli/go timeout scenario
                if scenario.lower() == "cli-connect-timeout":
                    state_to_check = dut_platform.cli_timeout_status
                elif scenario.lower() == "owner-connect-timeout":
                    state_to_check = dut_platform.go_timeout_status

            self.preset_p2p()
            time.sleep(3)
            """
            if accept_connect and False:      # p2p_connect_devices_cli method no use now
                wifi_generic_steps.p2p_scan_devices(serial=self.go_serial)()
                wifi_generic_steps.p2p_scan_devices(serial=self.cli_serial)()
                wifi_generic_steps.p2p_connect_devices_cli(serial = self.go_serial,
                                          slave_name = self.slave_name,
                                          master_name = self.go_name,
                                          serial2 = self.cli_serial,
                                          accept_connect=accept_connect,
                                          known_device=False,
                                          platform_go=self.platform_go if self.platform_go else None,
                                          platform_cli=self.platform_cli if self.platform_cli else None,
                                          )()
            """
            if accept_connect:
                wifi_generic_steps.p2p_connect_devices_prompt(serial = self.go_serial,
                                          slave_name = self.slave_name,
                                          master_name = self.go_name,
                                          serial2 = self.cli_serial,
                                          accept_connect=accept_connect,
                                          known_device=False,
                                          platform_go=self.platform_go if self.platform_go else None,
                                          platform_cli=self.platform_cli if self.platform_cli else None,
                                          )()

            else:
                wifi_generic_steps.p2p_connect_devices(serial=self.go_serial,
                                           slave_name=self.slave_name,
                                           master_name=self.go_name,
                                           serial2=self.cli_serial,
                                           accept_connect=accept_connect,
                                           known_device=False,
                                           no_response=no_response)()

            wifi_generic_steps.p2p_check_connection_info(go_serial = self.go_serial,
                                                 slave_serial = self.cli_serial,
                                                 regex=True,
                                                 state=state_to_check)()


    def config_p2p_traffic(self, state):
        print "====================== config_p2p_traffic ======================"
        if state:
            # start download
            negative = True
            if self.final_states["5_p2p_traffic"]:
                negative = False

            if self.scenario in ["CLI-Connect-timeout", "Owner-Connect-timeout"]:
                attrs['trycount'] = 120
            print 'configuring p2p traffic'
            peer_ip = wifi_utils.get_peer_ip(go_serial = self.go_serial, slave_serial = self.cli_serial, peer_type = self.ip_ping)

            if not peer_ip:
                raise Exception("Could not get p2p peer ip address.")
            self.step_id_p2p = psteps.add_step(wifi_steps.ping_ip,
                                      negative = negative,
                                      serial = self.ping_serial,
                                      trycount = attrs['trycount'],
                                      timeout = attrs['trycount'] + 20,
                                      transfer_type="p2p",
                                      target_percent = 10,
                                      ip = peer_ip,
                                      )

    def config_p2p_twoway_traffic(self, serial):
        negative = False
        peer_type = "go_ip"
        peer_ip = wifi_utils.get_peer_ip(go_serial=self.go_serial,
                                         slave_serial=self.cli_serial,
                                         peer_type=self.ip_ping)
        if self.ip_ping == "go_ip":
            peer_type = "slave_ip"

        peer_ip1 = wifi_utils.get_peer_ip(go_serial=self.go_serial,
                                          slave_serial=
                                          self.cli_serial, peer_type=peer_type)

        if not peer_ip:
            raise Exception("Could not get p2p peer: {} ip address.".format(
                self.cli_serial))
        if not peer_ip:
            raise Exception("Could not get p2p peer: {} ip address.".format(
                self.go_serial))
        wifi_steps.ping_ip(negative=negative, serial=self.ping_serial,
                           trycount=attrs['trycount'], timeout=attrs['trycount'] + 20,
                           transfer_type="p2p", target_percent=10,
                           ip=peer_ip)()
        wifi_steps.ping_ip(negative=negative, serial=self.cli_serial,
                           trycount=attrs['trycount'],
                           timeout=attrs['trycount'] + 20, transfer_type="p2p",
                           target_percent=10, ip=peer_ip1)()

    # define actions for each scenario
    def turn_wifi_off(self, serial):
        wifi_generic_steps.set_wifi(serial=serial, state="OFF")()
        wifi_generic_steps.set_wifi(serial=serial2, state="OFF")()

    def disconnect_wifi(self, serial):
        wifi_generic_steps.clear_saved_networks(serial=serial)()

    def deactivate_am(self, serial):
        wifi_generic_steps.set_airplane_mode(state = "OFF", serial = serial)()

    def activate_am(self, serial):
        wifi_generic_steps.set_airplane_mode(state = "ON", serial = serial)()

    def leave_group(self, serial):
        #wifi_generic_steps.p2p_forget_all_groups(serial = serial)()
        wifi_generic_steps.p2p_disconect_all(serial = serial)()

    def dismiss_cli(self, serial):
        wifi_generic_steps.p2p_disconect_all(serial = serial)()

    def remove_group(self, serial):
        wifi_generic_steps.p2p_forget_all_groups(serial=serial)()

    def generate_p2p_traffic(self, serial):
        if serial == self.go_serial:
            server_serial = self.cli_serial
            download_serial = self.go_serial
            server_on_go = False
        else:
            server_serial = self.go_serial
            download_serial = self.cli_serial
            server_on_go = True

        # make sure no other ftpd processes are running
        wifi_generic_steps.p2p_kill_ftpd(serial = server_serial)()

        # start the server
        wifi_generic_steps.p2p_start_ftpd(port_number = attrs["p2p_port_number"],
                                          serial = server_serial)()


        # generate the URL and push download file on the DUT acting as ftp server
        (URL, device_IP) = wifi_generic_steps.p2p_create_download_url(file_name = attrs["p2p_file_name"],
                                        file_size = attrs["file_size"],
                                        device_path = attrs["root_dir"],
                                        protocol = "ftp",
                                        port_number = attrs["p2p_port_number"],
                                        server_on_go = server_on_go,
                                        serial = server_serial,
                                        serial2 = download_serial)()

        # downloading the file
        wifi_generic_steps.download_file(URL, file_name = attrs["p2p_file_name"],
                                         file_size = attrs["file_size"],
                                         protocol = "ftp",
                                         serial = serial,
                                         serial2 = serial2)()

        #check download integrity
        wifi_generic_steps.check_file_integrity(mode = attrs["compare_method"],
                            local_file = attrs["p2p_file_name"],
                            remote_file = dut_platform.download_path + attrs["p2p_file_name"],
                            serial = serial)()

    def generate_wifi_traffic(self, serial):
        if serial == self.go_serial:
            server_serial = self.cli_serial
            server_on_go = False
        else:
            server_serial = self.go_serial
            server_on_go = True

        # generate URL
        (URL, IP) = wifi_generic_steps.create_download_url(file_name = attrs["wifi_file_name"],
                                        file_size = attrs["file_size"],
                                        local_path = ".",
                                        protocol = "ftp",
                                        port = attrs["port_number_wifi"],
                                        serial = serial)()

        # start download
        wifi_generic_steps.download_file(URL, file_name = attrs["wifi_file_name"],
                                         file_size = attrs["file_size"],
                                         protocol = "ftp",
                                         serial = serial)()

        #check download integrity
        wifi_generic_steps.check_file_integrity(mode = attrs["compare_method"],
                            local_file = attrs["wifi_file_name"],
                            remote_file = dut_platform.download_path + attrs["wifi_file_name"],
                            serial = serial)()


    def connect_p2p(self, serial):
        self.config_p2p_connected(True)

    def connect_wifi(self, serial):
        self.config_wifi_connected(state=True,toggle_off=False)

    def reject_connect(self, serial):
        self.config_p2p_connected(True, accept_connect=False)

    def accept_connect(self, serial):
        self.preset_p2p()
        time.sleep(10)
        wifi_generic_steps.p2p_connect_devices(serial = self.go_serial,
                                      slave_name = self.slave_name,
                                      master_name = self.go_name,
                                      serial2 = self.cli_serial,
                                      accept_connect=True,
                                      known_device=False)()

        wifi_generic_steps.p2p_check_connection_info(go_serial = self.go_serial,
                                             slave_serial = self.cli_serial,
                                             state="CONNECTED/CONNECTED")()

    def scan_wifi(self, serial):
        wifi_generic_steps.scan_and_check_ap(attrs["ap_name"],
                                             option = "refresh",
                                             serial = serial)()


    def connect_timeout(self, serial):
        if self.scenario == "CLI-Connect-timeout":
            group_state = attrs["cli_group_state"]
            group_key = "group_state"
            inviting_serial = self.go_serial
            receiving_serial = self.cli_serial
        else:
            group_state = attrs["go_group_state"]
            group_key = "group_state"
            inviting_serial = self.cli_serial
            receiving_serial = self.go_serial

        # connect p2p no response
        self.config_p2p_connected(True, accept_connect=False, no_response=True)

        # wait for timeout (group state changes from
        # GroupNegotiationState/UserAuthorizingNegotiationRequestState -> InactiveState)
        wifi_steps.p2p_wait_for_parameter_state(param_name = group_key,
                                                param_state = group_state,
                                                go_serial = inviting_serial,
                                                timeout=30,
                                                slave_serial = receiving_serial)()

        # wait for InactiveState state -> timeout reached
        wifi_steps.p2p_wait_for_parameter_state(param_name = group_key,
                                                param_state = "InactiveState",
                                                go_serial = inviting_serial,
                                                timeout=125,
                                                slave_serial = receiving_serial)()

        # get rid of the connection invite popup
        adb_steps.command(serial = inviting_serial, timeout = 10,
                          command = "input keyevent 4")()
        adb_steps.command(serial = receiving_serial, timeout = 10,
                          command = "input keyevent 4")()

        # initiate a again the connection
        self.config_p2p_connected(True)

    def reconnect_persistent(self, serial):
        if self.scenario == "CLI-Reconnect-Persistent-P2P":
            slave_name = self.go_name
            master_name = self.slave_name
            inviting_serial = self.cli_serial
            receiving_serial = self.go_serial
        else:
            slave_name = self.slave_name
            master_name = self.go_name
            inviting_serial = self.go_serial
            receiving_serial = self.cli_serial

        # disconnet p2p
        wifi_generic_steps.p2p_disconect_all(serial = serial)()

        # reconnect persistent
        wifi_generic_steps.p2p_connect_devices(serial=inviting_serial,
                                               slave_name=slave_name,
                                               master_name=master_name,
                                               serial2=receiving_serial,
                                               known_device=True)()

    def execute_scenario(self):
        print "====================== execute scenario ======================"
        time.sleep(attrs["pre_action_transition_wait_time"])
        self.scenario_action[self.scenario](serial=self.action_serial)
        time.sleep(attrs["post_action_transition_wait_time"])

    @property
    def verify_disconnect(self):
        stdout, stderr = subprocess.Popen("adb -s {} shell dumpsys wifi|grep 'isConnected=true mState=CONNECTED'"
                                          .format(serial), stderr=subprocess.PIPE,
                                          stdout=subprocess.PIPE, shell=True).communicate()
        print stdout
        return stdout

    def verify_scenario(self):
        if self.scenario in ["Owner-Turn-WiFi-OFF", "CLI-Turn-WiFi-OFF",
                             "Owner-Activate-AM", "CLI-Activate-AM"]:
            wifi_state_to_check = "DISCONNECTED/"
            p2p_state_to_check = "DISCONNECTED/"

        elif self.scenario in ["Owner-Disconnect-Wifi", "CLI-Disconnect-Wifi",
                               "Owner-Scan-WiFi", "CLI-Scan-WiFi"]:
            wifi_state_to_check = "DISCONNECTED"
            p2p_state_to_check = "CONNECTED/CONNECTED"

        elif self.scenario in ["Owner-Deactivate-AM", "CLI-Deactivate-AM",
                               "Owner-Generate-P2P-Traffic", "CLI-Generate-P2P-Traffic",
                               "CLI-Connect-P2P", "Owner-Accept-Connect",
                               "Owner-Generate-WiFi-Traffic", "CLI-Generate-WiFi-Traffic",
                               "CLI-Connect-timeout", "Owner-Connect-timeout",
                               "CLI-Reconnect-Persistent-P2P",
                               "Owner-Reconnect-Persistent-P2P",
                               "Owner-Remove-Group", "Owner-P2P-Twoway-Traffic"]:
            if self.final_states['2_wifi_connected']:
                wifi_state_to_check = "CONNECTED/CONNECTED"
            else:
                wifi_state_to_check = "DISCONNECTED/"
            p2p_state_to_check = "CONNECTED/CONNECTED"

        elif self.scenario in ["CLI-Leave-Group","Owner-Dismiss-CLI",
                               "Owner-Reject-Connect"]:
            if self.final_states['2_wifi_connected']:
                wifi_state_to_check = "CONNECTED/CONNECTED"
            else:
                wifi_state_to_check = "DISCONNECTED/"
            p2p_state_to_check = "DISCONNECTED/"

        elif self.scenario in ["Owner-Connect-WiFi", "CLI-Connect-WiFi"]:
            wifi_state_to_check = "CONNECTED/CONNECTED"
            p2p_state_to_check = "CONNECTED/CONNECTED"


        # check wifi transfer
        if self.states['4_wifi_traffic']:
            psteps.interpret_step(self.step_id_wifi)

        # check p2p transfer
        if self.states['5_p2p_traffic']:
            psteps.interpret_step(self.step_id_p2p)

        while True:
            if self.verify_disconnect:
                time.sleep(10)
            else:
                break

        # check wifi connection

        wifi_generic_steps.check_connection_info(state=wifi_state_to_check,
                                                 regex = True,
                                                 serial = self.action_serial)()

        # check p2p connection
        wifi_generic_steps.p2p_check_connection_info(state=p2p_state_to_check,
                                                go_serial = self.go_serial,
                                                timeout = 10,
                                                regex = True,
                                                slave_serial = self.cli_serial)()

        # check AM
        if self.final_states['1_am'] != wifi_utils.check_airplane_mode_on(serial = self.am_serial):
            return False
        return True

    def do(self):
        self.configure_scenario(self.scenario,
                                am=amode,
                                wifi_connected=wifi_connected,
                                p2p_connected=p2p_connected,
                                wifi_traffic=wifi_traffic,
                                p2p_traffic=p2p_traffic,
                                serial = serial,
                                serial2 = serial2)
        self.create_states()

        self.execute_scenario()


    def check_condition(self):
        return self.verify_scenario()



#use_control_process=False
psteps = ParallelSteps(use_control_process=False)
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
serial2 = args["serial2"]
wifi_connected = eval(args['wifi_connected'])
wifi_traffic = eval(args['wifi_traffic'])
amode = eval(args['amode'])
p2p_connected = eval(args['p2p_connected'])
p2p_traffic = eval(args['p2p_traffic'])
scenario = args['scenario']

dut_platf = None
if "dut_platf" in args:
    dut_platf = args['dut_platform']

ref_platf = None
if "ref_platf" in args:
    ref_platf = args['ref_platform']

interface5ghz = args.get('interface5ghz', None)
channel_no = args.get("channel_no", 'Auto')

attrs = {}

# device definitions
sophia_devices = ["oars7", "s3gr10m6s", "slti20mr6"]
cht_devices = ["cht_ffd", "cht_cr_rvp", "r2_cht_ffd", "r2_cht_cr"]
no_p2p_if_devices = sophia_devices + cht_devices

for key,val in wifi_defaults.wifi.items():
    attrs[key] = val
    if key in args.keys():
        attrs[key] = args[key]

for key,val in wifi_defaults.p2p.items():
    attrs[key] = val
    if key in args.keys():
        attrs[key] = args[key]

dut_platform = statics.Device(serial=serial)
ref_platform = statics.Device(serial=serial2)

# set devices to root
adb_steps.root_connect_device(serial = serial)()
adb_steps.root_connect_device(serial = serial2)()

# workaround the issue with "delete group"
adb_steps.input_back(serial = serial)()
adb_steps.input_back(serial = serial2)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()
ui_steps.wake_up_device(serial = serial2)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()
ui_steps.unlock_device(serial = serial2, pin=wifi_defaults.wifi['pin'])()

# turn WiFi ON
wifi_generic_steps.set_wifi(state="ON",serial = serial)()
wifi_generic_steps.set_wifi(state="ON",serial = serial2)()

# disconnect reference device from AP, in order to avoid potential frequency conflicts
wifi_generic_steps.clear_saved_networks(serial = serial2)()

# execute the scenario
run_scenario(scenario = scenario)()

