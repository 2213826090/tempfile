#!/usr/bin/env python

######################################################################
#
# @filename:    Stay-connected-to-web-for-long-time.py
# @description: Stabilioty test to be connected to a website for a long time
#
# @run example:
#
#            Stay-connected-to-web-for-long-time.py -s 0BA8F2A0
#                                               --script-args
#                                                   mode=bg
#                                                   txpwr=75
#                                                   security=none
#                                                   ref_ap_name=SSG_LAB_VAL_S4
#                                                   ref_ap_pw=ssg_ba_otc_val_asuss4
#                                                   duration=60
#                                                   ref_ap_security="WPA/WPA2 PSK"
#
# @author:      srinidhi.s@intel.com
#a
#######################################################################

##### imports #####
import sys
import time
import requests
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
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
ref_ap_name = args["ref_ap_name"]
ref_ap_pw = args["ref_ap_pw"]
ref_ap_security = args["ref_ap_security"]
duration = args["duration"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
txpwr = None
if "txpwr" in args.keys():
    txpwr = args["txpwr"]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add 1st Wi-Fi network
wifi_generic_steps.add_network(ssid = ref_ap_name,
                               security = ref_ap_security,
                               password = ref_ap_pw,
                               serial = serial)()
for x in range(0, 11):

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial=serial)()

    finish = time.time() + int(duration)
    print "Runnign stability test for " + duration + " seconds"
    while finish > time.time():
        # check we are connected to the correct network.
        wifi_generic_steps.check_connection_info(serial=serial,
                                                 SSID=ref_ap_name,
                                                 state='CONNECTED/CONNECTED')()

        wifi_generic_steps.ping_gateway(serial=serial, trycount="30")()

        print "Connection status OK at " + str(finish - time.time())

    print 'Opening web page repeated count = %d' %(x)
    adb_steps.command(serial=serial, command="am start -a android.intent.action.VIEW -d http://www.google.com")()
    time.sleep(20)
    adb_steps.command(serial=serial, command="am force-stop com.android.chrome")()

    ##### test end #####
