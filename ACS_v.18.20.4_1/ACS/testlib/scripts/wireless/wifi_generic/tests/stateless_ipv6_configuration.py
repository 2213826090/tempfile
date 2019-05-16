#!/usr/bin/env python

######################################################################
#
# @filename:    stateless_ipv6_configuration.py
# @description: Test that the device can acquire an IPv6 address
#               automatically
#
# @run example:
#
#            python stateless_ipv6_configuration.py -s 0BA8F2A0
#                                                   --script-args
#                                                       mode=mixed
#                                                       hidden_ssid=0
#                                                       security=none
#                                                       dut_security=None
#                                                       ipv6_enable=1
#                                                       radvd_enable=1
#                                                       iterations=1
#
# @author:      gabriel.porumb@intel.com
# @Edit author: srinidhi.s@intel.com
#               -- Change log:
#                       1. Add browse capability with ipv6
#                       2. line 135: Replace check_ipv6_address by
#                           check_ipv6_address_condition
#
#######################################################################

# imports #
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps
import traceback
import time

# initialization #
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ddwrt_ap_name = args["ap_name"]
ipv6_enable = args["ipv6_enable"]
radvd_enable = args["radvd_enable"]
ssh_host = args["ap_ip"]
ssh_user = args["ap_username"]
google_ipv6_DNS = '2001:4860:4860::8888'

browse_ipv6 = None
if "browse_ipv6" in args.keys():
    browse_ipv6 = args["browse_ipv6"]
ipv6_ap_ssid = None
if ipv6_ap_ssid in args.keys():
    ipv6_ap_ssid = args["ipv6_ap_ssid"]
ipv6_ap_pw = None
if ipv6_ap_pw in args.keys():
    ipv6_ap_pw = args["ipv6_ap_pw"]
ipv6_ap_security = None
if ipv6_ap_security in args.keys():
    ipv6_ap_security = args["ipv6_ap_security"]

ipv6_prefix = wifi_defaults.AP_IPV6_PREFIX
ap_ipv6_address = ipv6_prefix + "::1"

# Iterations
if "iterations" in args.keys():
    iterations = int(args["iterations"])
else:
    iterations = 1

# test start #
for i in range(iterations):
    try:
        ap_steps.setup(mode, security,
                      new_ssid=ddwrt_ap_name,
                      serial=serial,
                      ipv6_enable=ipv6_enable,
                      radvd_enable=radvd_enable)()

       # wait a couple of seconds before the next step
        time.sleep(2)

       # Add the AP ipv6 address
        ap_steps.set_ipv6(serial=serial,
                         ipv6_ip=ap_ipv6_address + "/64",
                         ssh_host=ssh_host,
                         ssh_user=ssh_user)()

       # wait a couple of seconds before the next step
        time.sleep(2)

       # Enable radvd
        ap_steps.radvd_enable(serial=serial,
                             ssh_host=ssh_host,
                             ssh_user=ssh_user)()

       # turn display on, if turned off
        ui_steps.wake_up_device(serial=serial)()

       # ensure the device is unlocked
        ui_steps.unlock_device(serial=serial, pin=wifi_defaults.wifi['pin'])()

       # go to home screen
        ui_steps.press_home(serial=serial)()

       # make sure there are no saved networks
        wifi_generic_steps.clear_saved_networks(serial=serial)()

       # add the Wi-Fi network
        wifi_generic_steps.add_network(ssid=ddwrt_ap_name,
                                      security=dut_security,
                                      serial=serial)()

       # wait until the device connects to a wifi network
        wifi_generic_steps.wait_until_connected(serial=serial)()

       # check we are connected to the correct network.
        wifi_generic_steps.check_connection_info(serial=serial,
                                                SSID=ddwrt_ap_name,
                                                state='CONNECTED/CONNECTED')()

       # check connection
        wifi_generic_steps.ping_gateway(serial=serial)()

       # Check and ping ipv6
        wifi_generic_steps.check_ipv6_address(serial=serial,
                                                        ipv6_prefix=ipv6_prefix,
                                                        ap_ipv6_address=ap_ipv6_address)()

    except:
        raise Exception("Error at iteration {0} \n {1}".format(i, traceback.print_exc()))

if browse_ipv6 == 1:

       #Connect to 2nd network and verify browse
           # go to home screen
        ui_steps.press_home(serial=serial)()

           # make sure there are no saved networks
        wifi_generic_steps.clear_saved_networks(serial=serial)()

            # Connect to 2nd network and verify browse
        wifi_generic_steps.add_network(ssid=ipv6_ap_ssid,
                                       security=ipv6_ap_security,
                                       password=ipv6_ap_pw,
                                       serial=serial)()

        # Add delay
        time.sleep(5)

            # wait until the device connects to a wifi network
        wifi_generic_steps.wait_until_connected(serial=serial)()

            # check we are connected to the correct network.
        wifi_generic_steps.check_connection_info(serial=serial,
                                             SSID=ipv6_ap_ssid,
                                             state='CONNECTED/CONNECTED')()

        #wifi_generic_steps.ping_ipv6_ip(serial=serial, ip= google_ipv6_DNS, trycount = 10)()

        adb_steps.command(serial=serial,
                          command="am start -a android.intent.action.VIEW -d http://ipv6.google.com")()
        ui_steps.wait_for_view(view_to_find={"text": "https://ipv6.google.com/?gws_rd=ssl"}, timeout=20,
                            serial=serial)()
        adb_steps.command(serial=serial, command="am force-stop com.android.chrome")()

# End test #
