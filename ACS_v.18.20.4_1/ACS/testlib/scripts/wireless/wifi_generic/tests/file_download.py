#!/usr/bin/env python

######################################################################
#
# @filename:    ftp_download.py
# @description: Dow
#
# @run example:
#
#            python test1.py -s 0BA8F2A0
#                            --script-args
#                               protocol=ftp
#                               port_number=20121
#                               file_name=generated.bin
#                               device_path="/storage/sdcard0/Download/"
#                               file_size=10000
#                               mode=n
#                               security=wpa_psk
#                               encryption=aes
#                               ap_name=ddwrt
#                               passphrase=test1234
#                               dut_security=wpa
#                               airplane_mode="ON"
#                               compare_method="md5"
#
# @author:      alexandru.i.nemes@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.ap import ap_steps
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.adb import adb_steps
from testlib.utils.statics.android import statics
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
protocol = args["protocol"]
port_number = args["port_number"]
#device_path = args["device_path"]
file_name = args["file_name"]
file_size = args["file_size"]
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ddwrt_ap_name =  args["ap_name"]
if "option" in args.keys():
    option = args["option"]
else:
    option = ""


# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
conf_security = None
if "conf_security" in args.keys():
    conf_security = args["conf_security"]
pairwise_cipher = None
if "pairwise_cipher" in args.keys():
    pairwise_cipher = args["pairwise_cipher"]
airplane_mode = "ON"
if "airplane_mode" in args.keys():
    airplane_mode = args["airplane_mode"]
compare_method = "cmp"
if "compare_method" in args.keys():
    compare_method = args["compare_method"]

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
channel_bw = None
if "channel_bw" in args.keys():
    channel_bw = args["channel_bw"]
if "interface5ghz" in args.keys():
    interface5ghz = args["interface5ghz"]
else:
    interface5ghz = "0"
if "negativetest" in args.keys():
    negativetest = args["negativetest"]
else:
    negativetest = "None"
if "addnetwork" in args.keys():
    addnetwork = args["addnetwork"]


##### test start #####
platform = statics.Device(serial=serial)

adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()
# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# airplane mode on, if configured so

if airplane_mode == "ON":
    wifi_generic_steps.set_airplane_mode(state = "ON", serial = serial)()

# wifi ON
wifi_generic_steps.set_wifi(state="ON", serial = serial)()

# configure ap with the first settings set
ap_steps.setup(mode, security,
               new_ssid = ddwrt_ap_name,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz,
               serial = serial)()

ap_steps.setup(mode, security,
               new_ssid = ddwrt_ap_name,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               channel_bw = channel_bw,
               serial = serial)()
# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add the Wi-Fi network
if "connect_from_ui" in args.keys():
    time.sleep(2)
    wifi_steps.connect_with_password(ap_name=ddwrt_ap_name, scroll=True, open_settings=True, security=True,
                                 password=ddwrt_ap_pass, serial=serial)()
else:
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()
time.sleep(2)
# verify the ping
wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()
# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED',
                                        Security=conf_security,pairwise_cipher=pairwise_cipher)()

(URL, IP) = wifi_generic_steps.create_download_url(file_name, file_size, local_path = ".",protocol = protocol, port = port_number, serial = serial)()

# start download
wifi_steps.download_file(URL, file_name, file_size, protocol, serial = serial, option=option , interface5ghz=interface5ghz , ap_name=ddwrt_ap_name, negativetest=negativetest)()

#check download integrity
wifi_generic_steps.check_file_integrity(mode = compare_method, local_file = file_name, remote_file = platform.download_path + file_name, serial = serial)()

##### test end #####
