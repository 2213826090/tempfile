#!/usr/bin/env python

######################################################################
#
# @filename:    check_ssl_connection.py
# @description: Test that DUT is able to access ssl pages
#
# @run example:
#
#            check_ssl_connection.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       passphrase=test1234
#                                                       dut_security=WPA
#                                                       ssid=dd-wrt-Mihai
#
#
# @author:      mihalachex.micu@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.android.ui.browser import browser_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val



# mandatory params
net_ap_ssid = args["net_ap_ssid"]
net_ap_password = args['net_ap_password']
net_ap_security = args['net_ap_security']


##### test start #####

adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# go to home screen
ui_steps.press_home(serial = serial)()

# add a network
wifi_generic_steps.add_network(ssid = net_ap_ssid, security = net_ap_security,
                               password = net_ap_password,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()


# open the chrome and connect to a ssl site
browser_steps.open_chrome_first_time(serial = serial,
                                     intent = True,
                                     url_to_open = "https://www.google.ro/?gws_rd=ssl")()

# verify that ssl button appears
ui_steps.wait_for_view(serial = serial, view_to_find = {"resourceId":"com.android.chrome:id/security_button"},
                       timeout = 200)()

# verify that search button appears
ui_steps.wait_for_view(serial = serial, view_to_find = {"resourceId":"tsbb"},
                       timeout = 200)()

##### test end #####