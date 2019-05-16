#!/usr/bin/env python

######################################################################
#
# @filename:    http_proxy.py
# @description: Browse the test Web page via the Http Proxy Server
#               in certain scenarios
#
# @run example:
#
#            python http_proxy.py -s 0BA8F2A0
#                                                  --script-args
#                                                  ap_name=ddwrt
#                                                  proxy_mode=Manual
#                                                  proxy_server=192.168.1.138
#                                                  proxy_port=3128
#                                                    proxy_bypass=192.168.1.1
#                                                    valid_server=False
#                                                    toggle_wifi=True
#                                                    dut_reboot=True
#                                                    ap_reboot=True
#                                                    new_ssid=ddwrt_proxy
#                                                    check_ping=True
#
# @author:      dragosx.nicolaescu@intel.com
#
#######################################################################

##### imports #####

import sys
import time
import wifi_defaults
from testlib.scripts.ddwrt import ddwrt_steps
from testlib.scripts.android.ui.browser import browser_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####

dut_security = wifi_defaults.wifi['dut_security']
passphrase = wifi_defaults.wifi['passphrase']
encryption = wifi_defaults.wifi['encryption']
test_page_ip = wifi_defaults.wifi['test_page_ip']

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params

ddwrt_ap_name = args["ap_name"]
proxy_mode = args["proxy_mode"]
proxy_server= args["proxy_server"]
proxy_port= args["proxy_port"]
test_page_url = "http://" + str(test_page_ip) + ":8080/testing.html"

# optional params

if "proxy_bypass" in args.keys():
    proxy_bypass = args["proxy_bypass"]
    bypass_url= "http://" + str(proxy_bypass)
else:
    proxy_bypass = None

if "valid_server" in args.keys():
    valid_server= args["valid_server"]
else:
    valid_server = True

if "toggle_wifi" in args.keys():
    toggle_wifi = args["toggle_wifi"]
else:
    toggle_wifi = False

if "dut_reboot" in args.keys():
    dut_reboot = args["dut_reboot"]
else:
    dut_reboot = False

if "ap_reboot" in args.keys():
    ap_reboot = args["ap_reboot"]
else:
    ap_reboot = False

if "new_ssid" in args.keys():
    new_ssid = args["new_ssid"]
else:
    new_ssid = None

if "check_ping" in args.keys():
    check_ping = args["check_ping"]
else:
    check_ping = False

##### test start #####

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(pin=wifi_defaults.wifi['pin'],serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# go to home screen
ui_steps.press_home(serial = serial)()

# make the HTTP Proxy configuration
wifi_steps.add_network(proxy=proxy_mode, proxy_hostname = proxy_server, proxy_port=proxy_port,proxy_bypass=proxy_bypass,
                               ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = passphrase,
                               serial = serial)()

# Test page should be opened succesfuly using chrome browser
browser_steps.open_chrome_proxy(url_to_open = test_page_url, valid_server=valid_server, serial=serial)()

# open and verify the bypass page
if proxy_bypass:
    browser_steps.open_specific_page(is_bad_url = False, url = bypass_url, url_title = "DD-WRT", wait_time = 4000, serial=serial)()

# check the Proxy configuration is kept after WiFi is turn on->off->on
if toggle_wifi:
    wifi_steps.toggle_wifi_switch_from_settings(wait_time = 3, iterations=1, serial=serial)()
    browser_steps.open_chrome_proxy(url_to_open = test_page_url, valid_server=valid_server, serial=serial)()

# check the Proxy configuration is kept after restarting DUT
if dut_reboot:
    adb_steps.reboot(command = "", no_ui = False, ip_enabled = False, disable_uiautomator = False, pin = wifi_defaults.wifi['pin'], serial=serial)()
    adb_steps.root_connect_device(serial = serial)()
    browser_steps.open_chrome_proxy(url_to_open = test_page_url, valid_server=valid_server, serial=serial)()

# check the Proxy configuration is kept after restarting  WiFi AP
if ap_reboot:
    ddwrt_steps.reload_ap()()
    browser_steps.open_chrome_proxy(url_to_open = test_page_url, valid_server=valid_server, serial=serial)()

# check the Proxy configuration is kept after AP_old to AP_new transition
if new_ssid:
    ddwrt_steps.setup(new_ssid = new_ssid, mode = "mixed", security="wpa_psk", wifi_password=passphrase, encryption=encryption)()
    wifi_steps.add_network(proxy=proxy_mode, proxy_hostname = proxy_server, proxy_port=proxy_port,proxy_bypass=proxy_bypass,
                               ssid = new_ssid,
                               security = dut_security,
                               password = passphrase,
                               serial = serial)()
    browser_steps.open_chrome_proxy(url_to_open = test_page_url, valid_server=valid_server, serial=serial)()

# check ICMP Ping to Http_Proxy server is unsuccesfuly
if check_ping:
    wifi_steps.ping_ip(ip=test_page_ip, negative = False, target_percent=100, trycount=5, serial=serial)()

# go to home screen
ui_steps.press_home(serial = serial)()

##### test end #####
