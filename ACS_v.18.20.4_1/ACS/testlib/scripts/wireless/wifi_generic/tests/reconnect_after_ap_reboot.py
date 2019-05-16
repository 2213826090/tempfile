#!/usr/bin/env python

######################################################################
#
# @filename:    reconnect_after_ap_reboot.py
# @description: Tests that the device will reconnect to the AP after
#               AP restart.
#
# @run example:
#
#            python reconnect_after_ap_reboot.py -s 0BA8F2A0
#                                                --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
#
# @author:      corneliu.stoicescu@intel.com
# @Parameter for wifi-toggle added by: srinidhi.s@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.ap import ap_steps
from testlib.scripts.ddwrt import ddwrt_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ddwrt_ap_name =  args["ap_name"]


# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
radius_ip = None
if "radius_ip" in args.keys():
    radius_ip = args["radius_ip"]
radius_secret = None
if "radius_secret" in args.keys():
    radius_secret = args["radius_secret"]
radius_identity = None
if "radius_identity" in args.keys():
    radius_identity = args["radius_identity"]
EAP_method = None
if "EAP_method" in args.keys():
    EAP_method = args["EAP_method"]
phase_2_auth = None
if "phase_2_auth" in args.keys():
    phase_2_auth = args["phase_2_auth"]
user_certificate = None
if "user_certificate" in args.keys():
    user_certificate = args["user_certificate"]
interface5ghz = None
if "interface5ghz" in args.keys():
    interface5ghz = args["interface5ghz"]
if interface5ghz == '1':
    ddwrt_ap_name+='5'

#Select parameter for ca_certificate
ca_certificates = None
if "ca_certificate" in args.keys():
    ca_certificates = 'Do not validate'

# Including a optional parameter for WIFi switch toggle
wifi_toggle = None
if "wifi_toggle" in args.keys():
        wifi_toggle = args["wifi_toggle"]
        wifi_toggle = int(wifi_toggle)

iterations = 1
if "iterations" in args.keys():
    iterations = args["iterations"]

hidden = "0"
if "hidden" in args.keys():
        hidden = "1"

ip_settings = None
if "static_ip" in args.keys():
    ip_settings = "Static"

ip_address = None
gateway = None
network_prefix_length = None
static_ip_range = None
if "static_ip_range" in args.keys():
    static_ip_range = args["static_ip_range"]

##### test start #####
#adb_steps.connect_device(serial = serial,
#                         port = adb_server_port)()

# configure ap
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid = ddwrt_ap_name,
               hidden_ssid = hidden,
               interface5ghz = interface5ghz,
               serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

##### for static IP TCs find the network mask, gateway and static IP address
if ip_settings:
    # make sure there are no saved networks
    wifi_generic_steps.clear_saved_networks(serial = serial)()

    # connect to the AP via DHCP
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                                   security = dut_security,
                                   password = ddwrt_ap_pass,
                                   identity = radius_identity,
                                   EAP_method = EAP_method,
                                   phase_2_auth = phase_2_auth,
                                   ca_certificate = ca_certificates,
                                   serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the correct network
    wifi_generic_steps.check_connection_info(serial = serial,
                                             SSID = ddwrt_ap_name,
                                             state='CONNECTED/CONNECTED')()
    wifi_generic_steps.ping_gateway(serial = serial)()

    # get connection info from current DHCP connection
    content = wifi_utils.get_connection_content(serial = serial)
    connection_info = wifi_utils.get_connection_info(content)
    network_prefix_length = connection_info["net_mask"]
    gateway= connection_info["Gateway"]

    # find a free IP address
    ip_address = wifi_generic_steps.find_available_ip(serial = serial,ip_range=static_ip_range)()


# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial=serial)()

# add the Wi-Fi network

wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               ip_settings = ip_settings,
                               ip_address = ip_address,
                               gateway = gateway,
                               ca_certificate = ca_certificates,
                               network_prefix_length = network_prefix_length,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED')()
count = 0
while count < int(iterations):
    # set wireless interface down
    if wifi_toggle == 1:
        print 'Toggling wifi switch to ON'
        adb_steps.command(serial=serial, command="su 0 svc wifi enable")()
    #ap_steps.set_ap_wireless(state="down")()
    ap_steps.setup("disabled", security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               hidden_ssid = hidden,
               interface5ghz = interface5ghz,
               serial = serial)()

        # wait disconnected
    wifi_generic_steps.check_wifi_state_disconnected(ap_name=ddwrt_ap_name, serial = serial)()

    # set wireless interface up
    #ap_steps.set_ap_wireless(state="up")()
    ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               hidden_ssid = hidden,
               interface5ghz = interface5ghz,
               serial = serial)()

    wifi_generic_steps.set_wifi(state="OFF", serial = serial)()
    time.sleep(1)
    wifi_generic_steps.set_wifi(state="ON", serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the correct network
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = ddwrt_ap_name,
                                            state='CONNECTED/CONNECTED')()
    if wifi_toggle == 1:
        print 'Toggling wifi switch to OFF'
        adb_steps.command(serial=serial, command="su 0 svc wifi disable")()
    count += 1

##### test end #####
