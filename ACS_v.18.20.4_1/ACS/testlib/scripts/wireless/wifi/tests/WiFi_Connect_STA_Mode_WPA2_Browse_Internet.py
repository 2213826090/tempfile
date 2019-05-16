#!/usr/bin/env python

######################################################################
#
# @filename:    WiFi_Connect_STA_Mode_WPA2_Browse_Internet.py
# @description: Tests if WiFi connect to WPA2 and is able to open a
#               website in Chrome
#
# @run example:
#
#               python WiFi_Connect_STA_Mode_WPA2_Browse_Internet.py
#            --serial 10.237.100.212:5555
#            --media-path /home/sys_spjenkins/testlib/resources/ddwrt/
#            --script-args ip=10.237.100.219
#                          internal_ip=192.168.1.1
#                          ap_name=dd-wrt
#                          ssh_user=root
#                          ssh_pass=""
#                          passphrase=PassPhrase
#
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################


import sys
import time
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.ddwrt import ddwrt_steps
from testlib.scripts.android.ui.browser import browser_steps
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

ddwrt_ip = args["ip"]
ddwrt_internal_ip = args["internal_ip"]
ddwrt_ap_name =  args["ap_name"]
ddwrt_user = args["ssh_user"]
ddwrt_pwd = args["ssh_pass"]
ddwrt_ap_pass = args["passphrase"]
ddwrt_script_path = media_path


ddwrt_steps.set_cfg(config = "security",
                    value = "wpa2_psk",
                    ssh_host = ddwrt_ip,
                    ssh_user = ddwrt_user,
                    ssh_pwd = ddwrt_pwd,
                    media_path = ddwrt_script_path,
                    blocking = True)()


local_steps.ssh_command(command = "reboot",
                        ssh_host = ddwrt_ip,
                        ssh_user = ddwrt_user,
                        ssh_pass = ddwrt_pwd,
                        blocking = True)()

time.sleep(30)

local_steps.wait_for_ping(ip = ddwrt_ip, blocking = True)()

wifi_steps.set_from_wifi_settings()()

time.sleep(30)

ap_list = [
    ddwrt_ap_name,
]

wifi_steps.check_AP_list_is_scanned(ap_list = ap_list, blocking = True)()

if wifi_utils.is_known_AP(ddwrt_ap_name):
    wifi_steps.forget_wifi_network(ap_name = ddwrt_ap_name)()

time.sleep(20)

wifi_steps.connect_with_password(ap_name = ddwrt_ap_name,
                                 password = ddwrt_ap_pass,
                                 blocking = True)()

ui_steps.press_home()()

browser_steps.open_browser()()

browser_steps.open_specific_page(value = "http://" + ddwrt_internal_ip,
                                 title = "DD-WRT",
                                 wait_time = 10)()


ui_steps.press_home()()

