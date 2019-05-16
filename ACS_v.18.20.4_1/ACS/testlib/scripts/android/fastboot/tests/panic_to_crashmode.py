#!/usr/bin/env python

######################################################################
#
# @filename:  panic_to_crashmode.py
# @description: DUT will enter crash mode after the maximum number of
#               kernel panic events occur
#
# @run example:
#
#            python panic_to_crashmode.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               watchdog_counter_max = 2
#                                               create_crash_mode = watchdog/panic
#                                               use_delay = True/ False
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.relay import relay_steps
import datetime
import time

# Initialization #
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
watchdog_counter_max = int(args["watchdog_counter_max"])
create_crash_mode = args["create_crash_mode"]
use_delay = True if args["use_delay"] == "True" else False
use_combo_button = True if args["use_combo_button"] == "True" else False

# Default parameters
watchdog_delay = 600
wait_for_state = None
fastboot_power_off_timeout = 20

# Test start #
try:
    device_state = local_utils.get_device_boot_state(serial=serial)

    if device_state == "fastboot":
        fastboot_steps.continue_to_adb(serial=serial)()
        adb_steps.wait_for_ui_processes(serial=serial)()
    elif device_state == "recovery":
        adb_steps.reboot(serial=serial)()
    elif device_state == "crashmode":
        print "adb -s {0} reboot".format(serial)
        local_steps.command("adb -s {0} reboot".format(serial))()
        local_steps.wait_for_adb(serial=serial)()
        adb_steps.wait_for_ui_processes(serial=serial)()
    else:
        relay_steps.reboot_main_os(serial=serial,
                                   relay_type=relay_type,
                                   relay_port=relay_port,
                                   power_port=power_port)()

    # Reboot the device into bootloader
    adb_steps.reboot(command="bootloader",
                     serial=serial)()

    # Set the watchdog counter
    fastboot_steps.command(command="oem set-watchdog-counter-max {0}".format(watchdog_counter_max),
                           serial=serial,
                           stderr_grep="finished. total time:")()
    # Continue to adb
    fastboot_steps.continue_to_adb(serial=serial)()

    # Wait for UI processes
    adb_steps.wait_for_ui_processes(serial=serial)()

    # Create panic and check device state
    # The panics must be created before the counter resets
    now_old = datetime.datetime.now()

    # Cause watchdog_counter_max kernel panics
    for count in range(watchdog_counter_max + 1):

        # If there are multiple iterations, print the iteration number
        if watchdog_counter_max > 0:
            print "========== Iteration {0} ==========".format(count)

        # Check if the command is issued before the counter resets
        now_new = datetime.datetime.now()
        delta = now_new - now_old
        if delta.seconds >= watchdog_delay:
            raise Exception("The panic must be created before {} seconds elapse from the last panic".
                            format(watchdog_delay))

        # The new datetime becomes the old one
        now_old = now_new

        # Is the use_delay is set to True and this is the last iteration, sleep for watchdog_delay seconds
        if count == watchdog_counter_max and use_delay is True:
            print "========== Sleeping for {0} seconds ==========".format(watchdog_delay)
            time.sleep(watchdog_delay)

        # In all but the last iterations we will wait for the device to boot into android
        # If use_delay is True, then the device will always boot into MOS (the last panic was created outside
        # the watchdog_counter_max period)
        if count != watchdog_counter_max or use_delay is True:
            wait_for_state = "android"
        elif use_combo_button is True:
            wait_for_state = "fastboot"
        else:
            wait_for_state = "crashmode"

        # Create panic
        relay_steps.create_panic_and_check_state(serial=serial,
                                                 create_crash_mode=create_crash_mode,
                                                 wait_for_state=wait_for_state,
                                                 relay_type=relay_type,
                                                 relay_port=relay_port,
                                                 power_port=power_port,
                                                 v_down_port=v_down_port,
                                                 v_up_port=v_up_port,
                                                 use_combo_button=use_combo_button)()

except:
    raise
finally:
    if wait_for_state == "fastboot":
        local_steps.command("fastboot -s {0} continue".format(serial))()
        time.sleep(fastboot_power_off_timeout)
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               wait_ui=True)()

# Test end #
