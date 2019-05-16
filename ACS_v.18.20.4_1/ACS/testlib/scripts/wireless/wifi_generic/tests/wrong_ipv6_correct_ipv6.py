#!/usr/bin/env python

######################################################################
#
# @filename:    wrong_ipv6_correct_ipv6.py
# @description: Test that the device allows an IPv6 address to be
#               set manually
#
# @run example:
#
#            python wrong_ipv6_correct_ipv6.py -s 0BA8F2A0
#                                                   --script-args
#                                                       mode=mixed
#                                                       hidden_ssid=0
#                                                       security=none
#                                                       dut_security=None
#                                                       ipv6_enable=1
#                                                       radvd_enable=0
#                                                       iterations=1
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# imports #
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.utils.defaults import wifi_defaults
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
ipv6_prefix = wifi_defaults.AP_IPV6_PREFIX
ap_ipv6_address = ipv6_prefix + "::1"
dut_ipv6_address = ipv6_prefix + "::2"
wrong_ipv6_address = "abcd:1234::1"

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

        # Set the dut wrong ipv6 address
        wifi_generic_steps.set_dut_ipv6(serial=serial,
                                        ipv6_ip=wrong_ipv6_address + "/64")()

        # wait a couple of seconds before the next step
        time.sleep(2)

        # Check wrong ipv6
        wifi_generic_steps.check_ipv6_address(serial=serial,
                                              ipv6_prefix=ipv6_prefix,
                                              ap_ipv6_address=ap_ipv6_address,
                                              dut_static_ip=True,
                                              negative=True)()

        # Set the dut ipv6 address
        wifi_generic_steps.set_dut_ipv6(serial=serial,
                                        ipv6_ip=dut_ipv6_address + "/64")()

        # wait a couple of seconds before the next step
        time.sleep(2)

        # Check ipv6
        wifi_generic_steps.check_ipv6_address(serial=serial,
                                              ipv6_prefix=ipv6_prefix,
                                              ap_ipv6_address=ap_ipv6_address,
                                              dut_static_ip=True)()
    except:
        raise Exception("Error at iteration {0} \n {1}".format(i, traceback.print_exc()))

# test end #
