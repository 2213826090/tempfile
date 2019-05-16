#!/usr/bin/env python

######################################################################
#
# @filename:    Get_unlock_ability_AND_Flashing_unlock.py
# @description: Boots into recovery mode via adb and locks the
#               bootloader
#
# @run example:
#
#            python Get_unlock_ability_AND_Flashing_unlock.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#
# lock_state = yes (to unlock)/ no (to lock)
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.fastboot import fastboot_steps
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

dev_oem_option = "OEM unlocking"
get_unlock_ability_command = "flashing get_unlock_ability"
unlock_disabled_output = "Unlock is disabled"
unlock_enablde_output = "The device can be unlocked"

# Test start #
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port,
                               wait_ui=True)()

    ui_steps.wake_up_device(serial=serial)()
    ui_steps.unlock_device(serial=serial)()

    dessert = adb_utils.get_android_version(serial=serial)

    # Disable OEM unlock in developer options #
    ui_steps.disable_options_from_developer_options(serial=serial,
                                                    developer_options=[dev_oem_option])()

    # Reboot into bootloader
    #adb_steps.reboot(command="fastboot", serial=serial)()
    local_steps.command(command="adb -s {0} reboot bootloader".format(serial))()
    local_steps.wait_for_fastboot(serial=serial,
                                  timeout=60)()

    # Check unlock ability (should be disabled)
    fastboot_steps.command(serial=serial,
                           command=get_unlock_ability_command,
                           stderr_grep=unlock_disabled_output,
                           timeout=20)()

    # Attempt to unlock bootloader
    relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader="yes",
                             oem_unlock_enabled="no",
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()

    # Continue to adb
    fastboot_steps.continue_to_adb(serial=serial)()

    # Wait for UI processes
    adb_steps.wait_for_ui_processes(serial=serial)()
    ui_steps.wake_up_device(serial=serial)()
    ui_steps.unlock_device(serial=serial)()

    # Enable OEM unlock in developer options #
    ui_steps.enable_oem_unlock(serial=serial, enabled=False, blocking=False)()

    # Reboot into bootloader
    # adb_steps.reboot(command="fastboot", serial=serial)()
    local_steps.command(command="adb -s {0} reboot bootloader".format(serial))()
    local_steps.wait_for_fastboot(serial=serial,
                                  timeout=60)()

    # Check unlock ability (should be enabled)
    fastboot_steps.command(serial=serial,
                           command=get_unlock_ability_command,
                           stderr_grep=unlock_enablde_output,
                           timeout=20)()

    # Attempt to unlock bootloader
    relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader="yes",
                             oem_unlock_enabled="yes",
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()
finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port)()
    adb_steps.wait_for_ui(serial=serial)()
# Test end #
