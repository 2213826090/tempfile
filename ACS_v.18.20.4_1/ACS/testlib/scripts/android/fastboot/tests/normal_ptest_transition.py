#!/usr/bin/env python

######################################################################
#
# @filename:    normal_ptest_transition.py
# @description: Reboots the device in ptest mode and back to normal
#               again
#
# @run example:
#
#            python normal_ptest_transition.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#                                               models_kept=False
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
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
models_kept = True if args["models_kept"] == "True" else False


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

    # Boot in ptest mode #
    adb_steps.reboot_ptest(serial=serial)()

    # Boot out of ptest mode #
    adb_steps.reboot_ptest_clear(serial=serial)()

    # If models kept is true, reboot the device to check if it boots normally #
    if models_kept:
        # Power off device
        relay_steps.power_off_device(serial=serial,
                                     relay_type=relay_type,
                                     relay_port=relay_port,
                                     power_port=power_port,
                                     v_down_port=v_down_port,
                                     v_up_port=v_up_port,
                                     except_charging=True)()
        # Power on device
        relay_steps.power_on_device(serial=serial,
                                    relay_type=relay_type,
                                    relay_port=relay_port,
                                    power_port=power_port,
                                    v_down_port=v_down_port,
                                    v_up_port=v_up_port)()
        # Wait for UI processes
        adb_steps.wait_for_ui_processes(serial=serial)()
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
