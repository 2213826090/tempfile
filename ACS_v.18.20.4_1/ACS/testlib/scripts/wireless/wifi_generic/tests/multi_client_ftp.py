#!/usr/bin/env python

# #############################################################################
#
# @filename:    multi_client_ftp.py
#
# @description: Creates a FTP server on reference device and initiate
#               upload of the generated file using chrome browser on the DUT.
#
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

device_list = []
device_key = re.compile('serial\d')
for key in args.keys():
    if(device_key.match(key)):
        device_list.append(args[key])

device_list.append(serial)
devices = device_list
device = device_list[0]

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


# Apks are already downloaed using 'GET ARTIFACTS' test step and install using below.
ftp_client_apk = os.path.expanduser("~")+"/.acs/Artifacts/Connectivity/Comms_WiFi/ftp_client.apk"
ftp_server_apk = os.path.expanduser("~")+"/.acs/Artifacts/Connectivity/Comms_WiFi/ftp_server.apk"

pwd = os.getcwd()

ftp_port = None
if "ftp_port" in args.keys():
    ftp_port = args["ftp_port"]

file_name = None
if "file_name" in args.keys():
    file_name = args["file_name"]

file_size = None
if "file_size" in args.keys():
    file_size = 1024 * int(args["file_size"])

# Hardcoded User credentials as per FTP server app.
# These credentials can be changed as per the change in use of FTP server app.
# Change these values to accommodate new credentials
ftp_username = 'francis'
ftp_password =  'francis'


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

for i in range(len(device_list)):
    #turn display on, if turned off
    ui_steps.wake_up_device(serial = devices[i])()

    # ensure the device is unlocked
    ui_steps.unlock_device(serial = devices[i], pin=wifi_defaults.wifi['pin'])()

    # go to home screen
    ui_steps.press_home(serial = devices[i])()

    # make sure there are no saved networks
    wifi_generic_steps.clear_saved_networks(serial = devices[i])()

    # add the Wi-Fi network
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                                   security = dut_security,
                                   password = ddwrt_ap_pass,
                                   identity = radius_identity,
                                   EAP_method = EAP_method,
                                   phase_2_auth = phase_2_auth,
                                   user_certificate = user_certificate,
                                   ca_certificate = ca_certificates,
                                   serial = devices[i])()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = devices[i])()

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = devices[i],
                                            SSID = ddwrt_ap_name if str(new_ssid).lower() == "none" else new_ssid,
                                            state='CONNECTED/CONNECTED')()

    # check connection
    wifi_generic_steps.ping_gateway(trycount=trycount, serial = devices[i])()

try:
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
    time.sleep(3)

    ui_steps.click_button_common(serial= device, view_to_find={
                    "text":"ALLOW"}, optional=True)()

    #Start FTP server of the APP
    ui_steps.click_button(serial=device,
                          view_to_find={"resourceId": "com.theolivetree.ftpserver:id/imageView1"},
                          view_to_check={"textContains": "Press button to stop ftp server"})()

    time.sleep(3)
    # Check of the device connection is established
    wifi_generic_steps.check_connection_info(serial = device,
                                           state='CONNECTED/CONNECTED')()

    # get the server IP address
    wifi_generic_steps.get_dut_ip_address(serial = device, timeout = 10)()

    # Generate URL along with IP address to download the file
    (URL, IP) = wifi_generic_steps.create_download_url_multiserver_ftp (file_name + device +'.bin', file_size , local_path = pwd, protocol = 'ftp', port = ftp_port, serial = device)()

    # Push the generated payload file to ftp server for the DUT to download
    adb_steps.push_file(serial=serial, local=pwd + "/" + file_name + device + '.bin', remote="/storage/emulated/0/",
                        timeout=6000)()

    # Initialize the DUT for upload
    #Install FTP client APK from the defined path
    #adb_steps.install_apk(serial = serial, apk_path = ftp_client_apk, install_time= 60)()

    # turn display on, if turned off
    ui_steps.wake_up_device(serial = serial)()

    # ensure the device is unlocked
    ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

    # go to home screen
    ui_steps.press_home(serial = serial)()

    # Open FTP client app from intent as the app is not available via apps menu in IVI
    adb_steps.am_start_command(serial=serial, component='lysesoft.andftp/lysesoft.andftp.SplashActivity')()

    time.sleep(3)
    # Close the pop up menu --optional == true
    ui_steps.click_button_common(serial = serial, view_to_find={"text": "Close"}, optional=True)()

    # Below steps are app specific. It opens the app and starts the client app for uploading the data
    ui_steps.click_button(serial=serial,
                          view_to_find= {"descriptionContains": "Add", "enabled": True},
                          view_to_check={"textContains": "Hostname:"})()

    ui_steps.edit_text(serial=serial,
                               view_to_find={"resourceId": "lysesoft.andftp:id/ftp_host"},
                               value=str(IP),
                               is_password=False)()

    ui_steps.edit_text(serial=serial,
                               view_to_find={"resourceId": "lysesoft.andftp:id/ftp_port"},
                               value=ftp_port,
                               is_password=False)()

    ui_steps.edit_text(serial=serial,
                               view_to_find={"resourceId": "lysesoft.andftp:id/ftp_username"},
                               value=ftp_username,
                               is_password=False)()

    ui_steps.edit_text(serial=serial,
                               view_to_find={"resourceId": "lysesoft.andftp:id/ftp_password"},
                               value=ftp_password,
                               is_password=True)()

    ui_steps.click_button(serial=serial,
                               view_to_find= {"textContains": "Save", "enabled": True},
                               view_to_check={"textContains": "OK"})()

    #Close the pop up menu --optional == true
    ui_steps.click_button_common(serial = serial, view_to_find={"text": "Close"}, optional=True)()

    ui_steps.edit_text(serial=serial,
                               view_to_find={"resourceId": "lysesoft.andftp:id/dialog_edit"},
                               value=URL,
                               is_password=False)()

    ui_steps.click_button(serial = serial,
                view_to_find = {"resourceId": "android:id/button1"},
                view_to_check = {"textContains": "FTP server settings saved successfully into your device."})()

    #Close the pop up menu --optional == true
    ui_steps.click_button_common(serial = serial, view_to_find={"text": "Close"}, optional=True)()

    ui_steps.click_button(serial=serial,
                               view_to_find= {"textContains": "OK", "enabled": True},
                               view_to_check={"textContains": URL})()

    #Close the pop up menu --optional == true
    ui_steps.click_button_common(serial = serial, view_to_find={"text": "Close"}, optional=True)()

    ui_steps.click_button(serial=serial,
                               view_to_find= {"textContains": URL, "enabled": True},
                               view_to_check={"textContains": "[Go up a folder]"})()

    #Close the pop up menu --optional == true
    ui_steps.click_button_common(serial = serial, view_to_find={"text": "Close"}, optional=True)()

    ui_steps.click_button(serial=serial,
                           view_to_find= {"descriptionContains": "Device file browser", "enabled": True},
                           view_to_check={"textContains": "Current: /storage/emulated/0"})()

    ui_steps.click_button(serial=serial,
                           view_to_find= {"textContains": file_name + device + '.bin', "enabled": True})()

    ui_steps.click_button(serial=serial,
                            view_to_find= {"descriptionContains": "Upload", "enabled": True},
                            view_to_check={"textContains": "1 item(s) to upload from device to / folder on FTP server, do you want to proceed?"})()

    ui_steps.click_button(serial=serial,
                                view_to_find= {"textContains": "OK", "enabled": True}, view_to_check={"textContains": 'Uploading'})()

    ui_steps.wait_for_view(serial=serial, timeout=6000000,
                                               view_to_find={"text": "Upload Completed"})()

    ui_steps.click_button(serial=serial,
                                view_to_find= {"textContains": "OK", "enabled": True}, view_to_check={"textContains": file_name + device + '.bin'})()

    wifi_generic_steps.download_file(url = URL, file_name = file_name + device + '.bin', protocol = 'ftp', file_size = file_size, serial= device)()

    wifi_generic_steps.check_file_integrity(mode = 'md5', local_file = pwd + '/' + file_name + device + '.bin', remote_file = '/sdcard/Download/' + file_name + device + '.bin', serial = serial)()
except Exception as e:
    print e.message
finally:
    #teardown

    # adb_steps.uninstall_apk(serial = device, apk_path = ftp_server_apk)()
    # adb_steps.uninstall_apk(serial= serial, apk_path = ftp_client_apk)()

    #Open FTP server app
    ui_steps.open_app_from_allapps(serial = device, view_to_find = {"text": "Ftp server"})()

    ui_steps.click_button_common(serial= device, view_to_find={
                    "text":"ALLOW"}, optional=True)()

    #Stop the server
    ui_steps.click_button(serial=device,
                          view_to_find={"resourceId": "com.theolivetree.ftpserver:id/imageView1"},
                          view_to_check={"textContains": "Press button to start ftp server"})()
#End
