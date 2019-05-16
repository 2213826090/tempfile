#!/usr/bin/env python

######################################################################
#
# @filename:    COS-to-MOS-Watchdog.py
# @description: Checks if the device enters charger OS when powered off
#
# @run example:
#
#            python COS-to-MOS-Watchdog.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#                                               iterations=5
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
from testlib.scripts.android.fastboot import fastboot_utils

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
iterations = int(args["iterations"])

# Test start #
try:
    platform_name = fastboot_utils.get_platform_name(serial=serial)
    if platform_name == "r2_cht_mrd" or platform_name == "r2_cht_ffd":
        for i in range(2):
            relay_steps.create_panic_and_check_state(serial=serial, create_crash_mode="watchdog", wait_for_state="android",
                                                                                    use_combo_button=False, relay_type=relay_type, relay_port=relay_port,
                                                                                    power_port=power_port, v_down_port=v_down_port, v_up_port=v_up_port)()
        relay_steps.create_panic_and_check_state(serial=serial, create_crash_mode="watchdog", wait_for_state="crashmode",
                                                                                use_combo_button=False, relay_type=relay_type, relay_port=relay_port,
                                                                                power_port=power_port, v_down_port=v_down_port, v_up_port=v_up_port)()

    else:
        for i in range(iterations):
            if iterations > 1:
                print "=============== Iteration: {0} ===============".format(i)
            # Power off device
            relay_steps.gracefully_power_off_device(serial=serial,
                                                    relay_type=relay_type,
                                                    relay_port=relay_port,
                                                    power_port=power_port)()
            # Wait for COS
            local_steps.wait_for_cos(serial=serial)()
            # Kill the watchdog daemon and wait for the device to boot to MOS
            relay_steps.create_panic_and_check_state(serial=serial,
                                                     create_crash_mode="watchdog",
                                                     wait_for_state="android",
                                                     use_combo_button=False,
                                                     relay_type=relay_type,
                                                     relay_port=relay_port,
                                                     power_port=power_port,
                                                     v_down_port=v_down_port,
                                                     v_up_port=v_up_port)()

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