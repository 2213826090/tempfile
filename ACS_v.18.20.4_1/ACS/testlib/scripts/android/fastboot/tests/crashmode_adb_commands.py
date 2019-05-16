#!/usr/bin/env python

######################################################################
#
# @filename:    crashmode_adb_commands.py
# @description: crash mode shall support adb commands "adb pull" and "adb reboot"
#
# @run example:
#
#            python crashmode_adb_commands.py -s 0BA8F2A0 --script-args
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
import os
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.connections.local import local_steps

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

pull_mos_file = "sys/firmware/acpi/tables/DSDT"
mos_tmp_file = "DSDT"
pull_crashmode_file = "acpi:DSDT"
crashmode_tmp_file = "acpi_DSDT"

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

    # Perform adb pull command from MOS

    adb_steps.root_connect_device(serial=serial)()
    local_steps.command(command="adb -s {0} pull {1} /tmp/{2}".format(serial, pull_mos_file, mos_tmp_file))()

    # Boot to crashmode
    os.system("adb -s {} reboot crashmode > /dev/null 2>&1".format(serial))
    time.sleep(15)
    local_steps.wait_for_crashmode(serial=serial, timeout=60)()

    # Perform adb pull command from crashmode
    local_steps.command(command="adb -s {0} pull {1} /tmp/{2}".format(serial, pull_crashmode_file, crashmode_tmp_file))()

    # Compare the two files. They should be the same
    local_steps.command(command="diff -s /tmp/{0} /tmp/{1}".format(mos_tmp_file, crashmode_tmp_file),
                        stdout_grep="are identical")()

    # Run the "adb reboot" command and wait for the device to boot to main UI
    local_steps.command(command="adb -s {0} reboot".format(serial))()
    adb_steps.wait_for_ui_processes(serial=serial)()


finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port,
                               wait_ui=True)()
# Test end #