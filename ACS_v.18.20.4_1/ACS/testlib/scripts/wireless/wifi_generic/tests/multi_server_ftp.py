#!/usr/bin/env python

# #############################################################################
#
# @filename:    multi_server_ftp.py
#
# @description: Crreates a FTP server on multiple reference devices and initiate
#               download of the generated file using chrome browser.
#               Add multiple serverX to increase the device count.
#               The device serial numbers are passed as arguments for serverX.
#               "Serial" parameter of ref devices are considered as "serverX"
#               for easy handling in the script. File size to be given in MBs.
#
# @author:      srinidhi.s@intel.com
#
##############################################################################

##### imports #####
import sys
import re
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

server_list = []
server_key = re.compile('serial')
for key in args.keys():
    if(server_key.match(key)):
        server_list.append(args[key])

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
trycount = '5'
if "trycount" in args.keys():
    trycount = args["trycount"]
#Select parameter for ca_certificate
ca_certificates = None
if "ca_certificate" in args.keys():
    ca_certificates = 'Do not validate'

ipv4_class = None
if "ipv4_class" in args.keys():
    ipv4_class = args["ipv4_class"]

ftp_client_apk = os.path.expanduser("~")+"/.acs/Artifacts/Connectivity/Comms_WiFi/ftp_client.apk"
ftp_server_apk = os.path.expanduser("~")+"/.acs/Artifacts/Connectivity/Comms_WiFi/ftp_server.apk"

#present_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), FTP_SERVER_APK_FILE)
#append_path = present_path + ' /../../../../../acs_binaries'
#print path
# append_path = '/home/intel/acs_new_repo'
#
# ftp_server_apk_path = None
# if "ftp_server_apk_path" in args.keys():
#     ftp_server_apk_path = args["ftp_server_apk_path"]
# ftp_server_apk = append_path + ftp_server_apk_path
# print ftp_server_apk

pwd = os.getcwd()
number_of_servers = len(server_list)

ftp_port = None
if "ftp_port" in args.keys():
    ftp_port = args["ftp_port"]

file_name = None
if "file_name" in args.keys():
    file_name = args["file_name"]

file_size = None
if "file_size" in args.keys():
    file_size = 1024 * int(args["file_size"])

# setup main access point where all servers & multiple DUTs will get connected

ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz,
               #serial = server+str(dev_no),
               channel_no=channel_no,
               ipv4_class = ipv4_class)()

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

url_list = []
for device in server_list:

    # turn display on, if turned off
    ui_steps.wake_up_device(serial = device)()

    # ensure the device is unlocked
    ui_steps.unlock_device(serial = device, pin=wifi_defaults.wifi['pin'])()

    # go to home screen
    ui_steps.press_home(serial = device)()

    # make sure there are no saved networks
    wifi_generic_steps.clear_saved_networks(serial = device)()

    # add the Wi-Fi network
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                                   security = dut_security,
                                   password = ddwrt_ap_pass,
                                   identity = radius_identity,
                                   EAP_method = EAP_method,
                                   phase_2_auth = phase_2_auth,
                                   user_certificate = user_certificate,
                                   ca_certificate = ca_certificates,
                                   serial = device)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = device)()

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = device,
                                            SSID = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                                            state='CONNECTED/CONNECTED')()

    # check connection
    wifi_generic_steps.ping_gateway(trycount=trycount, serial = device)()

    # Install APK from the defined path
    #adb_steps.install_apk(serial = device, apk_path = ftp_server_apk, install_time=60)()

    # turn display on, if turned off
    ui_steps.wake_up_device(serial = device)()

    # ensure the device is unlocked
    ui_steps.unlock_device(serial = device, pin=wifi_defaults.wifi['pin'])()

    # go to home screen
    ui_steps.press_home(serial = device)()

    #Launch  menu
    ui_steps.press_all_apps(serial= device)()

    #Open FTP server app
    ui_steps.open_app_from_allapps(serial = device, view_to_find = {"text": "Ftp server"})()

    ui_steps.click_button_common(serial= device, view_to_find={
                    "text":"ALLOW"}, optional=True)()

    #Start FTP server of the APP
    ui_steps.click_button(serial=device,
                          view_to_find={"resourceId": "com.theolivetree.ftpserver:id/imageView1"},
                          view_to_check={"textContains": "Press button to stop ftp server"})()

    # Check of the device connection is established
    wifi_generic_steps.check_connection_info(serial = device,
                                           state='CONNECTED/CONNECTED')()

    # get the server IP address
    wifi_generic_steps.get_dut_ip_address(serial = device, timeout = 10)()

    # Generate URL along with IP address to download the file
    (URL, IP) = wifi_generic_steps.create_download_url_multiserver_ftp (file_name + device +'.bin', file_size , local_path = pwd, protocol = 'ftp', port = ftp_port, serial = device)()
    url_list.append(URL)

    # Push the generated payload file to ftp server for the DUT to download
    adb_steps.push_file (serial = device, local =  pwd + "/" + file_name + device + '.bin', remote ="/storage/emulated/0/", timeout=6000)()

    # Download the file from the created FTP server one after the other
for i in range(len(server_list)):
    wifi_generic_steps.download_file(url_list[i], file_name + server_list[i] + '.bin', file_size, 'ftp', serial=serial)()
time.sleep(30)
    # Check the integrity of the downloaded file by md5 checksum
for device in server_list:
    wifi_generic_steps.check_file_integrity(mode = 'md5', local_file = pwd + '/' + file_name + device + '.bin', remote_file = '/sdcard/Download/' + file_name + device + '.bin', serial = serial)()

# End