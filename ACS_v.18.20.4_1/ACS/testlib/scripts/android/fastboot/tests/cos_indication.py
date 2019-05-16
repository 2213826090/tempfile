#!/usr/bin/env python

######################################################################
#
# @filename:    cos_indication.py
# @description: Checks if the device enters charger OS when powered off
#
# @run example:
#
#            python cos_indication.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.connections.local import local_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps

# Initialisation #
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
v_up_port = args["v_up_port"]
v_down_port = args["v_down_port"]

# Test start #
try:
    # Boot to main OS
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port,
                               wait_ui=True)()

    # Power off device
    relay_steps.gracefully_power_off_device(serial=serial,
                                            relay_type=relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port)()

    # Wait for COS
    local_steps.wait_for_cos(serial=serial)()

except:
    raise
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port,
                               wait_ui=True)()

# Test end #
