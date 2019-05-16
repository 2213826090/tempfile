#!/usr/bin/env python

######################################################################
#
# @filename:    ip_class_configure.py
# @description: Tests that when correctly adding a network the device will
#               connect to it; changes ipv4 class.
#
# @run example:
#
#            python add_network_and_connect.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
#
# @author:      srinidhi.s@intel.com
#
#######################################################################

##### imports #####
import sys
import time
import os
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps

from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
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
EAP_method=None
if "EAP_method" in args.keys():
    EAP_method = args["EAP_method"]
phase_2_auth = None
if "phase_2_auth" in args.keys():
    phase_2_auth = args["phase_2_auth"]
user_certificate = None
if "user_certificate" in args.keys():
    user_certificate = args["user_certificate"]
new_ssid = None
if "new_ssid" in args.keys():
    new_ssid = args["new_ssid"]
channel_bw = None
if "channel_bw" in args.keys():
    channel_bw = args["channel_bw"]
interface5ghz = None
if "interface5ghz" in args.keys():
    interface5ghz = args["interface5ghz"]
if interface5ghz == '1':
    ddwrt_ap_name+='5'
# adding below parameters to customize 'channel no' in Wifi router and 'ping
#  count' after connection
# By doing this, this script can be re-used for TC involves connection based on
#  specific channel.
channel_no = None
if "channel_no" in args.keys():
    channel_no = args["channel_no"]
# default ping try count as per ddwrt_steps.ping_gateway
trycount = '2'
if "trycount" in args.keys():
    trycount = args["trycount"]
#Select parameter for ca_certificate
ca_certificates = None
if "ca_certificate" in args.keys():
    ca_certificates = 'Do not validate'
ipv4_class = None
if "ipv4_class" in args.keys():
    ipv4_class = args["ipv4_class"]

reset_ssh_ip = '192.168.1.1'
##### test start #####

# configure ap
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz,
               serial = serial,
               channel_no=channel_no,
               ipv4_class = ipv4_class)()

# Reload the ETH1 interface on host to execute further TCs without effect bafter change in IP address class

os.system('ifconfig eth1 down')
time.sleep(10)
os.system('ifconfig eth1 up')
time.sleep(10)
ip_address = os.popen('ifconfig eth1 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
ip=ip_address.read()
ip_split_01=ip[:3]
ip_split_02=ipv4_class[:3]

if ip_split_01 == ip_split_02:
    print 'IP Successfully assigned'
else:
    print 'IP Change unsuccessful'


# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               ca_certificate = ca_certificates,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network.
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                                        state='CONNECTED/CONNECTED')()

# check connection
wifi_generic_steps.ping_gateway(trycount=trycount, serial = serial)()

ap_steps.setup(mode, security,
               ssh_host = ipv4_class,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid=ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz,
               serial = serial,
               channel_no=channel_no,
               ipv4_class = reset_ssh_ip)()

os.system('ifconfig eth1 down')
time.sleep(10)
os.system('ifconfig eth1 up')
time.sleep(10)


##### test end #####